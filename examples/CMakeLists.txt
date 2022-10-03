LIST(APPEND apps
    simple_accel/simple_accel
)

# message(STATUS ">> ${CMAKE_BINARY_DIR}/_deps/marko-src/include")

foreach(MAINS ${apps})
    set_source_files_properties(${MAINS}.ino PROPERTIES LANGUAGE CXX)
    get_filename_component(FILE ${MAINS} NAME_WE)
    message(STATUS ">> ${MAINS}.ino as ${FILE}")
    add_library(${FILE} ${MAINS}.ino)
    # add_library(${MAINS} ${MAINS}.ino)
    # add_dependencies(${MAIN} gecko)
    # target_include_directories(${MAIN}
    #     PRIVATE
    #         ${CMAKE_BINARY_DIR}/_deps/marko-src/include
    #         ${CMAKE_BINARY_DIR}/_deps/serialcomm-src/include
    # )
    # target_link_directories(${MAIN}
    #     PRIVATE
    #         ${CMAKE_BINARY_DIR}/_deps/marko-build
    #         ${CMAKE_BINARY_DIR}/_deps/serialcomm-build
    # )
    target_link_libraries(${FILE}
        PRIVATE
            ${PROJECT_NAME}
    )
endforeach()