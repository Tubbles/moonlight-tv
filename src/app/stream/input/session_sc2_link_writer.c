#include "session_sc2_link_writer.h"
#include "sc2_link_words.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#include <SDL_error.h>
#include <SDL_mutex.h>
#include <SDL_thread.h>

#include "logging.h"

#define XPAD_PATH_MAX 64
// Every word uploads the effect again, which restarts this, so a session never reaches it.
#define EFFECT_LENGTH_MS 0xFFFF
// ff-memless scales every magnitude by the gain, so anything less corrupts the words.
#define FULL_GAIN 0xFFFF
// ACK_STATE read before and after ACK_ECHO, until both reads agree.
#define ACKNOWLEDGEMENT_READ_ATTEMPTS 3

struct sc2_link_writer_t {
    char xpad_path[XPAD_PATH_MAX];
    int pads_fd;
    SDL_Thread *thread;
    SDL_mutex *lock;
    // Signalled on every change of the fields lock guards.
    SDL_cond *changed;

    // Guarded by lock.
    sc2_link_output_t output;
    bool output_wanted;
    bool stop_requested;
    sc2_link_message_queue_t queue;
    // What the reader passed on last, and how many changes it passed on.
    uint16_t acknowledged_echo;
    uint16_t acknowledged_state;
    uint32_t acknowledgement_changes;

    // The writer thread's own.
    int xpad_fd;
    int effect_id;
    // A node failure was logged.
    bool node_failed;
    sc2_link_client_t client;
};

static int writer_main(sc2_link_writer_t *writer);

static bool wait_for_opening(sc2_link_writer_t *writer);

static sc2_link_client_result_t open_session(sc2_link_writer_t *writer, const sc2_link_transport_t *transport);

static sc2_link_client_result_t serve(sc2_link_writer_t *writer, const sc2_link_transport_t *transport);

static bool next_message(sc2_link_writer_t *writer, uint64_t keep_alive_due_ms, sc2_link_message_t *message);

static void close_xpad_node(sc2_link_writer_t *writer);

static void set_output(sc2_link_writer_t *writer, sc2_link_output_t output);

static bool must_close(const sc2_link_writer_t *writer);

static void stop_other_effects(sc2_link_writer_t *writer);

static bool write_word(sc2_link_writer_t *writer, uint16_t word);

static bool write_event(sc2_link_writer_t *writer, uint16_t type, uint16_t code, int32_t value);

static bool read_acknowledgement(sc2_link_writer_t *writer, uint16_t *echo, uint16_t *state);

static bool read_axis(sc2_link_writer_t *writer, unsigned int code, uint16_t *value);

static uint32_t acknowledgement_changes(sc2_link_writer_t *writer);

static bool wait_for_advance(sc2_link_writer_t *writer, uint32_t changes_seen,
                             sc2_link_acknowledgement_t *acknowledgement);

static void note_node_failure(sc2_link_writer_t *writer, const char *action, int error);

static uint64_t monotonic_time_ms(void);

static sc2_link_word_result_t transport_send_word(void *context, uint16_t word, bool in_session,
                                                  sc2_link_acknowledgement_t *acknowledgement);

static bool transport_read_state(void *context, uint16_t *state);

static bool transport_wait_silence(void *context, uint32_t milliseconds);

static bool transport_stopping(void *context);

sc2_link_writer_t *sc2_link_writer_start(const char *xpad_path, int pads_fd) {
    sc2_link_writer_t *writer = calloc(1, sizeof(*writer));
    if (writer == NULL) {
        return NULL;
    }
    snprintf(writer->xpad_path, sizeof(writer->xpad_path), "%s", xpad_path);
    writer->pads_fd = pads_fd;
    writer->xpad_fd = -1;
    writer->effect_id = -1;
    writer->output = SC2_LINK_OUTPUT_IDLE;
    writer->lock = SDL_CreateMutex();
    writer->changed = SDL_CreateCond();
    if (writer->lock != NULL && writer->changed != NULL) {
        writer->thread = SDL_CreateThread((SDL_ThreadFunction) writer_main, "sc2link-out", writer);
    }
    if (writer->thread == NULL) {
        commons_log_error("SC2Link", "Can't start the writer: %s", SDL_GetError());
        sc2_link_writer_destroy(writer);
        return NULL;
    }
    return writer;
}

void sc2_link_writer_stop(sc2_link_writer_t *writer) {
    SDL_LockMutex(writer->lock);
    writer->stop_requested = true;
    SDL_CondBroadcast(writer->changed);
    SDL_UnlockMutex(writer->lock);
    SDL_WaitThread(writer->thread, NULL);
    writer->thread = NULL;
}

void sc2_link_writer_destroy(sc2_link_writer_t *writer) {
    if (writer->changed != NULL) {
        SDL_DestroyCond(writer->changed);
    }
    if (writer->lock != NULL) {
        SDL_DestroyMutex(writer->lock);
    }
    free(writer);
}

void sc2_link_writer_want_output(sc2_link_writer_t *writer) {
    SDL_LockMutex(writer->lock);
    writer->output_wanted = true;
    if (writer->output == SC2_LINK_OUTPUT_IDLE) {
        writer->output = SC2_LINK_OUTPUT_OPENING;
    }
    SDL_CondBroadcast(writer->changed);
    SDL_UnlockMutex(writer->lock);
}

/* Only changes count: each accepted word changes the count in ACK_STATE, and pad movement changes neither field. */
void sc2_link_writer_note_acknowledgement(sc2_link_writer_t *writer, uint16_t echo, uint16_t state) {
    SDL_LockMutex(writer->lock);
    if (echo != writer->acknowledged_echo || state != writer->acknowledged_state) {
        writer->acknowledged_echo = echo;
        writer->acknowledged_state = state;
        writer->acknowledgement_changes++;
        SDL_CondBroadcast(writer->changed);
    }
    SDL_UnlockMutex(writer->lock);
}

sc2_link_output_t sc2_link_writer_output(sc2_link_writer_t *writer) {
    SDL_LockMutex(writer->lock);
    sc2_link_output_t output = writer->output;
    SDL_UnlockMutex(writer->lock);
    return output;
}

/* Taking what it drops keeps it off SDL, which would play it into the words. */
bool sc2_link_writer_offer(sc2_link_writer_t *writer, const sc2_link_message_t *message) {
    SDL_LockMutex(writer->lock);
    bool taken = writer->output == SC2_LINK_OUTPUT_OPENING || writer->output == SC2_LINK_OUTPUT_OPEN;
    if (taken && !must_close(writer)) {
        sc2_link_queue_push(&writer->queue, message);
        SDL_CondBroadcast(writer->changed);
    }
    SDL_UnlockMutex(writer->lock);
    return taken;
}

static int writer_main(sc2_link_writer_t *writer) {
    sc2_link_transport_t transport = {
            .context = writer,
            .send_word = transport_send_word,
            .read_state = transport_read_state,
            .wait_silence = transport_wait_silence,
            .stopping = transport_stopping,
    };
    while (wait_for_opening(writer)) {
        sc2_link_client_result_t result = open_session(writer, &transport);
        if (result == SC2_LINK_CLIENT_DELIVERED) {
            set_output(writer, SC2_LINK_OUTPUT_OPEN);
            commons_log_info("SC2Link", "Rumble and haptics session open on %s", writer->xpad_path);
            result = serve(writer, &transport);
        }
        close_xpad_node(writer);
        if (result == SC2_LINK_CLIENT_FAILED) {
            set_output(writer, SC2_LINK_OUTPUT_UNAVAILABLE);
            break;
        }
        set_output(writer, SC2_LINK_OUTPUT_IDLE);
    }
    return 0;
}

/** @return false when the thread is to end instead. */
static bool wait_for_opening(sc2_link_writer_t *writer) {
    SDL_LockMutex(writer->lock);
    while (!writer->stop_requested && writer->output != SC2_LINK_OUTPUT_OPENING) {
        SDL_CondWait(writer->changed, writer->lock);
    }
    bool opening = !writer->stop_requested;
    SDL_UnlockMutex(writer->lock);
    return opening;
}

/*
 * Whether the webOS app may open the node for writing is unknown, so every
 * failure here leaves the controller without output and its inputs as they
 * are.
 */
static sc2_link_client_result_t open_session(sc2_link_writer_t *writer, const sc2_link_transport_t *transport) {
    writer->node_failed = false;
    writer->effect_id = -1;
    writer->xpad_fd = open(writer->xpad_path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (writer->xpad_fd < 0) {
        note_node_failure(writer, "open for writing", errno);
        return SC2_LINK_CLIENT_FAILED;
    }
    if (!write_event(writer, EV_FF, FF_GAIN, FULL_GAIN)) {
        return SC2_LINK_CLIENT_FAILED;
    }
    stop_other_effects(writer);
    sc2_link_client_result_t result = sc2_link_client_hello(&writer->client, transport);
    if (result == SC2_LINK_CLIENT_FAILED && !writer->node_failed) {
        commons_log_warn("SC2Link", "The dongle did not answer HELLO on %s, so the controller gets no rumble or haptics",
                         writer->xpad_path);
    }
    return result;
}

/* A word without acknowledgement drops its message, and the next one opens a new session after silence. */
static sc2_link_client_result_t serve(sc2_link_writer_t *writer, const sc2_link_transport_t *transport) {
    uint64_t keep_alive_due_ms = monotonic_time_ms() + SC2_LINK_KEEP_ALIVE_MS;
    sc2_link_message_t message;
    while (next_message(writer, keep_alive_due_ms, &message)) {
        sc2_link_client_result_t result = sc2_link_client_deliver(&writer->client, transport, &message);
        if (result == SC2_LINK_CLIENT_FAILED && !writer->node_failed) {
            commons_log_warn("SC2Link", "The dongle stopped answering HELLO on %s, so the controller gets no "
                                        "rumble or haptics", writer->xpad_path);
        }
        if (result == SC2_LINK_CLIENT_FAILED || result == SC2_LINK_CLIENT_STOPPED) {
            return result;
        }
        if (result == SC2_LINK_CLIENT_DROPPED) {
            commons_log_debug("SC2Link", "A message to the controller was dropped");
        }
        keep_alive_due_ms = monotonic_time_ms() + SC2_LINK_KEEP_ALIVE_MS;
    }
    return SC2_LINK_CLIENT_STOPPED;
}

/**
 * The next message to send: the oldest queued one, or NOP once nothing went
 * out until keep_alive_due_ms.
 *
 * @return false when the session is to end instead.
 */
static bool next_message(sc2_link_writer_t *writer, uint64_t keep_alive_due_ms, sc2_link_message_t *message) {
    bool have_message = false;
    SDL_LockMutex(writer->lock);
    while (!must_close(writer)) {
        if (sc2_link_queue_pop(&writer->queue, message)) {
            have_message = true;
            break;
        }
        uint64_t now_ms = monotonic_time_ms();
        if (now_ms >= keep_alive_due_ms) {
            *message = sc2_link_nop_message();
            have_message = true;
            break;
        }
        SDL_CondWaitTimeout(writer->changed, writer->lock, (Uint32) (keep_alive_due_ms - now_ms));
    }
    SDL_UnlockMutex(writer->lock);
    return have_message;
}

/* Closing erases the effect, after which ff-memless sends one zero rumble packet. */
static void close_xpad_node(sc2_link_writer_t *writer) {
    if (writer->xpad_fd >= 0) {
        close(writer->xpad_fd);
        writer->xpad_fd = -1;
    }
    writer->client.in_session = false;
}

/* Messages wait for an open session only. */
static void set_output(sc2_link_writer_t *writer, sc2_link_output_t output) {
    SDL_LockMutex(writer->lock);
    writer->output = output;
    if (output == SC2_LINK_OUTPUT_IDLE || output == SC2_LINK_OUTPUT_UNAVAILABLE) {
        writer->queue.count = 0;
    }
    SDL_CondBroadcast(writer->changed);
    SDL_UnlockMutex(writer->lock);
}

/* With lock held. */
static bool must_close(const sc2_link_writer_t *writer) {
    return writer->stop_requested || !writer->output_wanted;
}

/*
 * ff-memless adds up the magnitudes of every playing effect on the device, so
 * a rumble another client left playing, SDL's for this pad for one, would
 * corrupt every word. An EV_FF play event reaches any effect id, whoever
 * uploaded it. Ids nobody owns are ignored.
 */
static void stop_other_effects(sc2_link_writer_t *writer) {
    int effect_count = 0;
    if (ioctl(writer->xpad_fd, EVIOCGEFFECTS, &effect_count) < 0) {
        return;
    }
    for (int effect_id = 0; effect_id < effect_count; effect_id++) {
        struct input_event stop;
        memset(&stop, 0, sizeof(stop));
        stop.type = EV_FF;
        stop.code = (uint16_t) effect_id;
        stop.value = 0;
        if (write(writer->xpad_fd, &stop, sizeof(stop)) < 0) {
            return;
        }
    }
}

/*
 * The upload alone rearms a playing effect for ff-memless's timer, and the play
 * event sends the packet at once. When the timer fires in between, the word
 * goes out twice, and the dongle takes the second as a copy.
 */
static bool write_word(sc2_link_writer_t *writer, uint16_t word) {
    struct ff_effect effect;
    memset(&effect, 0, sizeof(effect));
    effect.type = FF_RUMBLE;
    effect.id = (int16_t) writer->effect_id;
    effect.replay.length = EFFECT_LENGTH_MS;
    effect.u.rumble.strong_magnitude = sc2_link_strong_magnitude(word);
    effect.u.rumble.weak_magnitude = sc2_link_weak_magnitude(word);
    if (ioctl(writer->xpad_fd, EVIOCSFF, &effect) < 0) {
        note_node_failure(writer, "upload a rumble effect to", errno);
        return false;
    }
    writer->effect_id = effect.id;
    return write_event(writer, EV_FF, (uint16_t) effect.id, 1);
}

static bool write_event(sc2_link_writer_t *writer, uint16_t type, uint16_t code, int32_t value) {
    struct input_event event;
    memset(&event, 0, sizeof(event));
    event.type = type;
    event.code = code;
    event.value = value;
    if (write(writer->xpad_fd, &event, sizeof(event)) != (ssize_t) sizeof(event)) {
        note_node_failure(writer, "write to", errno);
        return false;
    }
    return true;
}

/* As the kernel holds them now, which can be ahead of the frames the reader passed on. */
static bool read_acknowledgement(sc2_link_writer_t *writer, uint16_t *echo, uint16_t *state) {
    uint16_t state_after = 0;
    for (int attempt = 0; attempt < ACKNOWLEDGEMENT_READ_ATTEMPTS; attempt++) {
        if (!read_axis(writer, ABS_BRAKE, state) || !read_axis(writer, ABS_GAS, echo) ||
            !read_axis(writer, ABS_BRAKE, &state_after)) {
            return false;
        }
        if (state_after == *state) {
            break;
        }
    }
    *state = state_after;
    return true;
}

static bool read_axis(sc2_link_writer_t *writer, unsigned int code, uint16_t *value) {
    struct input_absinfo info;
    if (ioctl(writer->pads_fd, EVIOCGABS(code), &info) < 0) {
        note_node_failure(writer, "read the acknowledgements for", errno);
        return false;
    }
    *value = (uint16_t) info.value;
    return true;
}

static uint32_t acknowledgement_changes(sc2_link_writer_t *writer) {
    SDL_LockMutex(writer->lock);
    uint32_t changes = writer->acknowledgement_changes;
    SDL_UnlockMutex(writer->lock);
    return changes;
}

/**
 * Waits up to SC2_LINK_RESEND_AFTER_MS for the reader to pass on a count past
 * the one in state_before.
 */
static bool wait_for_advance(sc2_link_writer_t *writer, uint32_t changes_seen,
                             sc2_link_acknowledgement_t *acknowledgement) {
    uint64_t deadline_ms = monotonic_time_ms() + SC2_LINK_RESEND_AFTER_MS;
    bool advanced = false;
    SDL_LockMutex(writer->lock);
    for (;;) {
        if (writer->acknowledgement_changes != changes_seen) {
            changes_seen = writer->acknowledgement_changes;
            if (sc2_link_state_advanced(acknowledgement->state_before, writer->acknowledged_state)) {
                acknowledgement->echo = writer->acknowledged_echo;
                acknowledgement->state = writer->acknowledged_state;
                advanced = true;
                break;
            }
        }
        uint64_t now_ms = monotonic_time_ms();
        if (now_ms >= deadline_ms) {
            break;
        }
        SDL_CondWaitTimeout(writer->changed, writer->lock, (Uint32) (deadline_ms - now_ms));
    }
    SDL_UnlockMutex(writer->lock);
    return advanced;
}

/* Each failure ends the output, so this logs once per start. */
static void note_node_failure(sc2_link_writer_t *writer, const char *action, int error) {
    commons_log_warn("SC2Link", "Can't %s %s, so the controller gets no rumble or haptics: %s", action,
                     writer->xpad_path, strerror(error));
    writer->node_failed = true;
}

static uint64_t monotonic_time_ms(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t) now.tv_sec * 1000u + (uint64_t) now.tv_nsec / 1000000u;
}

/*
 * Stop-and-wait, as docs/PROTOCOL.md describes it and sc2link_test.py's
 * Link.send() does: one word in flight, since xpad keeps at most one packet
 * pending (Linux 6.12) or fails a write during a transfer (4.4).
 */
static sc2_link_word_result_t transport_send_word(void *context, uint16_t word, bool in_session,
                                                  sc2_link_acknowledgement_t *acknowledgement) {
    sc2_link_writer_t *writer = context;
    if (!read_acknowledgement(writer, &acknowledgement->echo, &acknowledgement->state)) {
        return SC2_LINK_WORD_FAILED;
    }
    acknowledgement->state_before = acknowledgement->state;
    for (int resend = 0; resend <= SC2_LINK_MAX_RESENDS; resend++) {
        uint32_t changes_seen = acknowledgement_changes(writer);
        if (!write_word(writer, word)) {
            return SC2_LINK_WORD_FAILED;
        }
        if (wait_for_advance(writer, changes_seen, acknowledgement)) {
            return SC2_LINK_WORD_ACKNOWLEDGED;
        }
        if (!read_acknowledgement(writer, &acknowledgement->echo, &acknowledgement->state)) {
            return SC2_LINK_WORD_FAILED;
        }
        if (sc2_link_state_advanced(acknowledgement->state_before, acknowledgement->state)) {
            return SC2_LINK_WORD_ACKNOWLEDGED;
        }
        // Outside a session a resend would be rumble.
        if (in_session && !sc2_link_session_active(acknowledgement->state)) {
            return SC2_LINK_WORD_UNACKNOWLEDGED;
        }
    }
    return SC2_LINK_WORD_UNACKNOWLEDGED;
}

static bool transport_read_state(void *context, uint16_t *state) {
    uint16_t echo;
    return read_acknowledgement(context, &echo, state);
}

static bool transport_wait_silence(void *context, uint32_t milliseconds) {
    sc2_link_writer_t *writer = context;
    uint64_t deadline_ms = monotonic_time_ms() + milliseconds;
    SDL_LockMutex(writer->lock);
    for (;;) {
        uint64_t now_ms = monotonic_time_ms();
        if (must_close(writer) || now_ms >= deadline_ms) {
            break;
        }
        SDL_CondWaitTimeout(writer->changed, writer->lock, (Uint32) (deadline_ms - now_ms));
    }
    bool waited = !must_close(writer);
    SDL_UnlockMutex(writer->lock);
    return waited;
}

static bool transport_stopping(void *context) {
    sc2_link_writer_t *writer = context;
    SDL_LockMutex(writer->lock);
    bool stopping = must_close(writer);
    SDL_UnlockMutex(writer->lock);
    return stopping;
}
