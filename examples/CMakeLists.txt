LIST(APPEND apps
    simple_press_alt/simple_press.cpp
    simple_accel_alt/simple_accel.cpp
)

# message(STATUS ">> ${CMAKE_BINARY_DIR}/_deps/marko-src/include")

foreach(MAINS ${apps})
    #set_source_files_properties(${MAINS}.ino PROPERTIES LANGUAGE CXX)
    get_filename_component(APP ${MAINS} NAME_WE)
    message(STATUS ">> ${MAINS} as ${APP}")
    add_executable(${APP} ${MAINS})
    # add_library(${MAINS} ${MAINS}.ino)
    # add_dependencies(${MAIN} gecko)
    target_include_directories(${APP}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/linux
    )
    # target_link_directories(${MAIN}
    #     PRIVATE
    #         ${CMAKE_BINARY_DIR}/_deps/marko-build
    #         ${CMAKE_BINARY_DIR}/_deps/serialcomm-build
    # )
    target_link_libraries(${APP}
        PRIVATE
            ${PROJECT_NAME}
            # wirelib
    )
endforeach()