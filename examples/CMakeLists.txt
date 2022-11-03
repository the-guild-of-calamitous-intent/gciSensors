LIST(APPEND apps
    linux/simple_press.cpp
    linux/simple_accel.cpp
    linux/simple_mag.cpp
)

foreach(MAINS ${apps})
    get_filename_component(APP ${MAINS} NAME_WE)
    message(STATUS ">> ${MAINS} as ${APP}")
    add_executable(${APP} ${MAINS})

    if (APPLE)
    target_include_directories(${APP}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/mock
    )
    else()
    target_include_directories(${APP}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/linux
    )
    endif()

    target_link_libraries(${APP}
        PRIVATE
            ${PROJECT_NAME}
            # wirelib
    )
endforeach()