search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ar4.srdf
robot_name_in_srdf=ar4
moveit_config_pkg=ar4_moveit_config
robot_name=ar4
planning_group_name=ar_manipulator
ikfast_plugin_pkg=ar4_ar_manipulator_ikfast_plugin
base_link_name=ar4_base_link
eef_link_name=ar4_ee_link
ikfast_output_path=/home/mariamelsebaey/gp_ws/src/ar4_ar_manipulator_ikfast_plugin/src/ar4_ar_manipulator_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
