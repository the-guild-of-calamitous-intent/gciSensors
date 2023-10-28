
enable_testing()

list(APPEND gtests
    main-gtest.cpp
)

add_executable(test-all ${gtests})
target_link_libraries(test-all PRIVATE GTest::gtest_main ${PROJECT_NAME})
add_test(NAME test-all COMMAND test-all)
include(GoogleTest)
gtest_discover_tests(test-all)