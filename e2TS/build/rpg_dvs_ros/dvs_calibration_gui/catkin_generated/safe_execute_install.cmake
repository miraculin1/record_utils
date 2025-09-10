execute_process(COMMAND "/workspace/record_utils/e2TS/build/rpg_dvs_ros/dvs_calibration_gui/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/workspace/record_utils/e2TS/build/rpg_dvs_ros/dvs_calibration_gui/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
