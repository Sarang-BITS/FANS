#include <pci/drone.h>

Drone::Drone(ros::NodeHandle nh, ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private)
{
  ros::Timer timer = nh_.createTimer(ros::Duration(0.05), &Drone::timer_cb, this);
  initialize_variables();
  initialize_pub_sub();
  initialize_drone();
  ros::spin();
}

void Drone::initialize_pub_sub()
{
  state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::state_cb, this);
  odom_sub = nh_.subscribe("mavros/local_position/pose", 10, &Drone::odom_cb, this);
  local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  local_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("global_pose", 10);
  arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  sp_pos_sub = nh_.subscribe("sp_pos", 10, &Drone::sp_pos_cb, this);
  sp_vel_sub = nh_.subscribe("sp_vel", 10, &Drone::sp_vel_cb, this);
}

void Drone::initialize_variables()
{
  sp_mode = SP_mode::kPos;
  pci_ready = false;
  takeoff_pose = Eigen::Vector3d(0.0,0.0,5.0);

  sp_vel.linear.x = 0.0;
  sp_vel.linear.y = 0.0;
  sp_vel.linear.z = 0.0;
  sp_vel.angular.x = 0.0;
  sp_vel.angular.y = 0.0;
  sp_vel.angular.z = 0.0;

  sp_pose.position.x = takeoff_pose(0);
  sp_pose.position.y = takeoff_pose(1);
  sp_pose.position.z = takeoff_pose(2);
  sp_pose.orientation.x = 0.0;
  sp_pose.orientation.y = 0.0;
  sp_pose.orientation.z = 0.0;
  sp_pose.orientation.w = 1.0;

  /* Set params */
  std::vector<double> param_val;
  std::string param_name;
  ns = ros::this_node::getNamespace();
  // Offset
  param_name = ns + "/offset";
  // std::cout << param_name << std::endl;
  while(!ros::param::get(param_name, param_val)) 
  {
    // std::cout << "Getting offset" << std::endl; 
    ros::spinOnce();
    // std::cout << "Getting offset" << std::endl; 
  }
  offset << param_val[0], param_val[1], param_val[2];
  // Initial position
  param_val.clear();
  param_name = ns + "/initial_pos";
  // std::cout << param_name << std::endl;
  while(!ros::param::get(param_name, param_val))
  {
    // std::cout << "Getting initial position" << std::endl; 
    ros::spinOnce();
  }
  takeoff_pose << param_val[0], param_val[1], param_val[2];
}


void Drone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Drone::sp_pos_cb(const geometry_msgs::Pose& pos_sp)
{
  sp_mode = SP_mode::kPos;
  sp_pose = pos_sp;
}

void Drone::sp_vel_cb(const geometry_msgs::Twist& vel_sp)
{
  sp_mode = SP_mode::kVel;
  sp_vel = vel_sp;
}

void Drone::odom_cb(const geometry_msgs::PoseStamped& odom)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  current_pose.position.x = odom.pose.position.x + offset(0);
  current_pose.position.y = odom.pose.position.y + offset(1);
  current_pose.position.z = odom.pose.position.z + offset(2);
  current_pose.orientation = odom.pose.orientation;
  current_pose_vec(0) = odom.pose.position.x;
  current_pose_vec(1) = odom.pose.position.y;
  current_pose_vec(2) = odom.pose.position.z;
  current_pose_vec = current_pose_vec + offset;

  geometry_msgs::PoseStamped global_pose;
  global_pose.header.frame_id = "map";
  global_pose.pose = current_pose;
  global_pose_pub_.publish(global_pose);

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = ns + "/base_link";
  transformStamped.transform.translation.x = global_pose.pose.position.x;
  transformStamped.transform.translation.y = global_pose.pose.position.y;
  transformStamped.transform.translation.z = global_pose.pose.position.z;
  transformStamped.transform.rotation.x = global_pose.pose.orientation.x;
  transformStamped.transform.rotation.y = global_pose.pose.orientation.y;
  transformStamped.transform.rotation.z = global_pose.pose.orientation.z;
  transformStamped.transform.rotation.w = global_pose.pose.orientation.w;
  br.sendTransform(transformStamped);
}

void Drone::timer_cb(const ros::TimerEvent& e)
{
  controller();
}

bool Drone::initialize_drone()
{
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected)
  {
      ros::spinOnce();
      rate.sleep();
  }

  // Dummy setpoint
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i)
  {
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }
  if(set_offboard())
  {
    if(arm_drone())
    {
      takeoff();
      sp_pose = current_pose;
      pci_ready = true;
      return true;
    }
    else
    {
      ROS_ERROR("UNABLE TO ARM");
      return false;
    }
  }
  else
  {
    ROS_ERROR("UNABLE TO SET INTO OFFBOARD");
    return false;
  }
}

bool Drone::set_offboard()
{
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)    
  {
    ROS_INFO("Offboard enabled");
    return true;
  }
  else return false;
}

bool Drone::arm_drone()
{
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  if( arming_client.call(arm_cmd) && arm_cmd.response.success)
  {
    ROS_INFO("Vehicle armed");
    return true;
  }
  else return false;
}

void Drone::takeoff()
{
  ROS_INFO("TAKING OFF");
  ros::Rate loop_rate(20.0);
  Eigen::Vector3d setpoint;
  setpoint = takeoff_pose - offset;
  geometry_msgs::PoseStamped pose_to_publish;
  pose_to_publish.pose.position.x = setpoint(0);
  pose_to_publish.pose.position.y = setpoint(1);
  pose_to_publish.pose.position.z = setpoint(2);
  pose_to_publish.pose.orientation.x = 0.0;
  pose_to_publish.pose.orientation.y = 0.0;
  pose_to_publish.pose.orientation.z = 0.0;
  pose_to_publish.pose.orientation.w = 1.0;
  
  while(!target_reached(0.2, takeoff_pose))
  {
    local_pos_pub.publish(pose_to_publish);
    ros::spinOnce();
    loop_rate.sleep();
    // std::cout << "takeof manuver" << std::endl;
  }
  // pci_ready = true;
  ROS_INFO("TAKEOFF DONE");
}

void Drone::controller()  // Sends commands to the drone
{
  if(pci_ready)
  {
    if(sp_mode == SP_mode::kPos)
    {
      geometry_msgs::PoseStamped pose_to_pub;
      pose_to_pub.pose.position.x = sp_pose.position.x - offset(0);
      pose_to_pub.pose.position.y = sp_pose.position.y - offset(1);
      pose_to_pub.pose.position.z = sp_pose.position.z - offset(2);
      pose_to_pub.pose.orientation.x = 0.0;
      pose_to_pub.pose.orientation.y = 0.0;
      pose_to_pub.pose.orientation.z = 0.0;
      pose_to_pub.pose.orientation.w = 1.0;
      local_pos_pub.publish(pose_to_pub);
    }
    else if(sp_mode == SP_mode::kVel)
    {
      geometry_msgs::TwistStamped vel_to_pub;
      vel_to_pub.twist = sp_vel;
      local_vel_pub.publish(vel_to_pub);
    }
  }
}

bool Drone::target_reached(double tol, Eigen::Vector3d target)  // Target is in global frame
{
  if((current_pose_vec-target).norm() < tol) return true;
  else return false;
}
