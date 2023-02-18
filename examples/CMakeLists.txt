LIST(APPEND apps
    linux/simple_press.cpp
    linux/simple_accel.cpp
    linux/simple_mag.cpp
)

foreach(MAINS ${apps})
    get_filename_component(APP ${MAINS} NAME_WE)
    # message(STATUS ">> ${MAINS} as ${APP}")
    add_executable(${APP} ${MAINS})

    target_include_directories(${APP}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
            ${gcimock_SOURCE_DIR}/include
            ${squaternion_SOURCE_DIR}/src
    )

    target_link_libraries(${APP}
        PRIVATE
            ${PROJECT_NAME}
            # gcimock
    )
endforeach()