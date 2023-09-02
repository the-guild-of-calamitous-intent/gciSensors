LIST(APPEND apps
    simple_press
    simple_accel
    simple_mag
)

foreach(app ${apps})
    add_executable(${app} ${app}.cpp)
    target_link_libraries(${app} PRIVATE ${PROJECT_NAME})
endforeach()
