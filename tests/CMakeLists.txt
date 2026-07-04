find_package(GTest REQUIRED)

add_executable(robstride_driver_tests
  test_at_serial_framing.cpp
  test_position_unwrapper.cpp
  test_protocol.cpp
  test_robstride_motor.cpp
)
target_link_libraries(robstride_driver_tests
  robstride_driver::robstride_driver
  GTest::gtest
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(robstride_driver_tests)
