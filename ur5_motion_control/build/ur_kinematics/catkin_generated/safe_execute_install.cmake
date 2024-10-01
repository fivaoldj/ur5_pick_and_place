execute_process(COMMAND "/home/andrey/projects/ur5_pick_and_place/ur5_motion_control/build/ur_kinematics/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/andrey/projects/ur5_pick_and_place/ur5_motion_control/build/ur_kinematics/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
