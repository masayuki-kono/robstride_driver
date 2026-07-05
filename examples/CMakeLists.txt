add_executable(velocity_control velocity_control.cpp)
target_link_libraries(velocity_control robstride_driver::robstride_driver)

# Command-tracking captures for docs/test_results.md, one per control mode.
foreach(mode velocity position pp current operation)
  add_executable(tracking_capture_${mode} tracking_capture_${mode}.cpp)
  target_link_libraries(tracking_capture_${mode}
                        robstride_driver::robstride_driver)
endforeach()
