#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

class GummiInverseJacobian
{
public:
  GummiInverseJacobian();

private:
  void initializeJointPublishers();
  void advertiseJointPublishers();
  void desiredCallback(const geometry_msgs::Twist::ConstPtr& desired);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_in);
  geometry_msgs::Twist scaleDesired(geometry_msgs::Twist desired);
  void calculateDesiredJointVelocity(geometry_msgs::Twist desired);
  void publishJointVelocities();
  bool checkIfConnectedToRobot();
  double limitJointVelocity(double vel, double max);
  void findAndSetParameters();
  void doUpdate();

  ros::NodeHandle nh_;

  int num_joints_;
  bool debug_mode_;
  ros::Publisher joint_cmd_pub_;
  ros::Subscriber desired_sub_, joint_state_sub_;
  std::vector<std::string> joint_names_;
  std::vector<double> current_joint_positions_;
  bool received_joint_positions_;
  std::vector<double> joint_stiffnesses_;

  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  ros::Time timeOfLastJointState_;
  double control_gain_;
  double control_gain_pose_;
  std::vector<double> desired_joint_velocities_;
  geometry_msgs::Twist zero_vel_;
  geometry_msgs::Twist desired_vel_;
  double max_joint_vel_;
  double min_joint_vel_;
  double scale_translation_;
  double scale_rotation_;

  KDL::ChainFkSolverPos_recursive* fk_solver_;
  KDL::ChainIkSolverVel_pinv* ik_solver_;

};

GummiInverseJacobian::GummiInverseJacobian()
{

  KDL::Tree kdl_tree;
  KDL::Chain chain;
  std::string robot_desc_string;
  
  received_joint_positions_ = false;
  zero_vel_.linear.x = 0.0;
  zero_vel_.linear.y = 0.0;
  zero_vel_.linear.z = 0.0;
  zero_vel_.angular.x = 0.0;
  zero_vel_.angular.y = 0.0;
  zero_vel_.angular.z = 0.0;
  desired_vel_ = zero_vel_;

  findAndSetParameters();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
  kinematic_model_ = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());  

  nh_.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }

  kdl_tree.getChain("base_link", "tool", chain);

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);
  ik_solver_ = new KDL::ChainIkSolverVel_pinv(chain);

  const std::vector<std::string>&  j_n = kinematic_model_->getJointModelNames();
  const std::vector< moveit::core::JointModel * > & joint_models = kinematic_model_->getActiveJointModels();
  for(std::vector< moveit::core::JointModel * >::const_iterator joint = joint_models.begin(); joint != joint_models.end(); joint++) {
    if (!(*joint)->isPassive()) {
      std::string name = (*joint)->getName();
      joint_names_.push_back(name);
      printf("Active joint found: %s.\n", name.c_str()); 
    }
  }
  assert(joint_names_.size() == num_joints_);

  for(int i = 0; i < num_joints_; i++) {
    joint_stiffnesses_.push_back(-0.1);
    desired_joint_velocities_.push_back(0.0);
    current_joint_positions_.push_back(0.0);
  }
  joint_stiffnesses_.at(0) = -0.25;
  joint_stiffnesses_.at(6) = -0.75;

  joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("inverse_jacobian/joint_commands", 1);

  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("inverse_jacobian/joint_states", 10, &GummiInverseJacobian::jointStateCallback, this);
  desired_sub_ = nh_.subscribe<geometry_msgs::Twist>("inverse_jacobian/cmd_vel", 10, &GummiInverseJacobian::desiredCallback, this);
  
}

void GummiInverseJacobian::findAndSetParameters()
{
  nh_.param("inverse_jacobian/num_joints", num_joints_, 7);
  nh_.param("inverse_jacobian/debug_mode", debug_mode_, false);
  nh_.param("inverse_jacobian/control_gain", control_gain_ , 0.01);
  nh_.param("inverse_jacobian/control_gain_pose", control_gain_pose_ , 0.001);
  nh_.param("inverse_jacobian/max_joint_vel", max_joint_vel_, 0.04);
  nh_.param("inverse_jacobian/scale_translation", scale_translation_, 0.5);
  nh_.param("inverse_jacobian/scale_rotation", scale_rotation_, 1.0);
}

void GummiInverseJacobian::doUpdate()
{

  bool connected = checkIfConnectedToRobot();
  if(connected) {

    if(!received_joint_positions_) {

      ros::Duration(2.0).sleep();

      printf("Received initial pose OK, starting robot control!\n");
      received_joint_positions_ = true;

    }
    else {
      
      geometry_msgs::Twist vel = desired_vel_;

      calculateDesiredJointVelocity(vel);
      publishJointVelocities();

    }
    
  }
  else {
    printf("Warning: Not connected to robot, not sending command.\n");
    received_joint_positions_ = false;
  }

  ros::Duration(0.01).sleep();
  
}

void GummiInverseJacobian::desiredCallback(const geometry_msgs::Twist::ConstPtr& desired)
{

  desired_vel_ = scaleDesired(*desired);

}

void GummiInverseJacobian::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_in)
{

  sensor_msgs::JointState in = *joint_state_in;

  for(int i = 0; i < num_joints_; i++) {
    std::string name = joint_names_.at(i);
    double position = 0.0;

    for(int j = 0; j < in.name.size(); j++) {

      if(name.compare(in.name[j]) == 0)  {
	position = in.position[j];
	current_joint_positions_.at(i) = position; // TODO: mstoelen, break if cannot find
	break;
      }

    }

  }

  timeOfLastJointState_ = ros::Time::now();

  doUpdate();

}

geometry_msgs::Twist GummiInverseJacobian::scaleDesired(geometry_msgs::Twist desired) 
{
  geometry_msgs::Twist out;

  out.linear.x = desired.linear.x * scale_translation_;
  out.linear.y = desired.linear.y * scale_translation_;
  out.linear.z = desired.linear.z * scale_translation_;
  
  out.angular.x = desired.angular.x * scale_rotation_;
  out.angular.y = desired.angular.y * scale_rotation_;
  out.angular.z = desired.angular.z * scale_rotation_;

  return out;
}

void GummiInverseJacobian::calculateDesiredJointVelocity(geometry_msgs::Twist desired)
{

  KDL::JntArray q_current(num_joints_);
  KDL::JntArray qdot(num_joints_);

  for(int i = 0; i < num_joints_; i++) {
    q_current(i) = current_joint_positions_.at(i);
  }

  KDL::Vector pos_vel(desired.linear.x,
		      desired.linear.y,
		      desired.linear.z);
  KDL::Vector rot_vel(desired.angular.x,
		      desired.angular.y,
		      desired.angular.z);
  KDL::Twist T_desired = KDL::Twist(pos_vel, rot_vel);

  KDL::Frame F_current;
  int ret_fk = fk_solver_->JntToCart(q_current,F_current);
  if(ret_fk < 0) {
    printf("FK not solved!\n");
    assert(false);
  }
 
  KDL::Frame F_at_hand = F_current;
  F_at_hand.M = KDL::Rotation::Identity();
  
  T_desired = F_at_hand * T_desired; 

  int ret_ik = ik_solver_->CartToJnt(q_current,T_desired, qdot);
  if(ret_ik < 0) {
    printf("Diff IK not solved!\n");
    assert(false);
  }

  double max_current_joint_vel = 0.0;
  for(unsigned int i=0; i<num_joints_; i++) {
    if(std::abs(qdot(i)) > max_joint_vel_) {
      max_current_joint_vel = std::abs(qdot(i));
    }
  }

  if(max_current_joint_vel > max_joint_vel_) {
    printf("Warning: Limiting joint velocities.\n");
    for(unsigned int i=0; i<num_joints_; i++) {
      desired_joint_velocities_.at(i) = limitJointVelocity(qdot(i), max_current_joint_vel); 
    }
  }
  else {
    for(unsigned int i=0; i<num_joints_; i++) {
      desired_joint_velocities_.at(i) = qdot(i);
    }
  }
  
}

void GummiInverseJacobian::publishJointVelocities()
{

  sensor_msgs::JointState message;
  std::vector<double> positions;

  message.name = joint_names_;
  message.effort = joint_stiffnesses_;
  message.velocity = desired_joint_velocities_;

  joint_cmd_pub_.publish(message);

}

bool GummiInverseJacobian::checkIfConnectedToRobot()
{

  assert(timeOfLastJointState_.isValid());

  ros::Time joint = timeOfLastJointState_;
  ros::Time now = ros::Time::now();
  ros::Duration difference = now-joint;
  double timePassedMs = difference.toSec()*1000;

  if(timePassedMs > 200) {
    return false;
  }
  else {
    return true;
  }

}

double GummiInverseJacobian::limitJointVelocity(double vel, double max_current_vel) {
  return vel * max_joint_vel_/max_current_vel;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gummi_inverse_jacobian");
  GummiInverseJacobian gummi_inverse_jacobian;

  ros::spin();

}

