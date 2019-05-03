execute_process(COMMAND "/home/Code/point_robot_2d_processing/build/ros_processing_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/Code/point_robot_2d_processing/build/ros_processing_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
