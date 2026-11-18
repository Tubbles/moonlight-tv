target_sources(moonlight-lib PRIVATE
        session_input.c
        session_keyboard.c
        session_gamepad.c
        session_mouse.c
        session_touch.c
        session_virt_mouse.c)
if (FEATURE_INPUT_EVMOUSE)
    target_sources(moonlight-lib PRIVATE session_evmouse.c)
endif ()
# Pure functions only, built on every Linux target so the desktop build can test them.
if (OS_LINUX)
    target_sources(moonlight-lib PRIVATE sc2_link_protocol.c sc2_link_words.c)
endif ()
if (FEATURE_INPUT_SC2_LINK)
    target_sources(moonlight-lib PRIVATE session_sc2_link.c session_sc2_link_writer.c)
endif ()