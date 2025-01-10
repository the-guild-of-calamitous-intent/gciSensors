list(APPEND examples
    accel
    mag
    press_bmp390
    dummy
    scan_bus
    # gps
    press_dps310
)

foreach(app ${examples})
    add_executable(${app} ${app}.cpp)
    # target_link_libraries(${app} PRIVATE ${PROJECT_NAME} i2c)
    target_link_libraries(${app} PRIVATE ${PROJECT_NAME})
endforeach()