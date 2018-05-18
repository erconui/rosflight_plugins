 /*
 * Copyright 2017 James Jackson Brigham Young University MAGICC LAB
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rosflight_plugins/imu_plugin.h"

namespace rosflight_plugins
{

ImuPlugin::ImuPlugin() : gazebo::ModelPlugin() { }

ImuPlugin::~ImuPlugin()
{
  updateConnection_.reset();
  // gazebo::event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  nh_.shutdown();
}


void ImuPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load IMU plugin");
    return;
  }
  ROS_INFO("Loaded the IMU plugin");

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  last_time_ = world_->SimTime();

  namespace_.clear();


  gravity_ = world_->Gravity();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    ROS_ERROR("[imu_plugin] Please specify a namespace.");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    ROS_ERROR("[imu_plugin] Please specify a linkName.");

  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  // Get mass
  mass_ = link_->GetInertial()->Mass();

  //
  // ROS Node Setup
  //

  nh_ = ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/imu");

  // load params from rosparam server
  noise_on_ = nh_private_.param<bool>("noise_on", true);
  pub_rate_ = nh_private_.param<double>("rate", 1000.0);
  imu_topic_ = nh_private_.param<std::string>("topic", "imu/data");
  acc_bias_topic_ = nh_private_.param<std::string>("acc_bias_topic", "imu/acc_bias");
  gyro_bias_topic_ = nh_private_.param<std::string>("gyro_bias_topic", "imu/gyro_bias");

  gyro_stdev_ = nh_private_.param<double>("gyro_stdev", 0.02);
  gyro_bias_range_ = nh_private_.param<double>("gyro_bias_range", 0.001);
  gyro_bias_walk_stdev_ = nh_private_.param<double>("gyro_bias_walk_stdev", 1e-6);

  acc_stdev_ = nh_private_.param<double>("acc_stdev", 1.15);
  acc_bias_range_ = nh_private_.param<double>("acc_bias_range", 0.015);
  acc_bias_walk_stdev_ = nh_private_.param<double>("acc_bias_walk_stdev", 1e-6);

  // ROS Publishers
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
  gyro_bias_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(gyro_bias_topic_, 10);
  acc_bias_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(acc_bias_topic_, 10);

  // Calculate sample time from sensor update rate
  sample_time_ = 1.0/pub_rate_;

  // Configure Noise
  random_generator_.seed(std::time(nullptr));
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  // Turn off noise and bias if noise_on_ disabled
  if (!noise_on_)
  {
    gyro_stdev_ = 0;
    gyro_bias_range_ = 0;
    gyro_bias_walk_stdev_ = 0;
    gyro_bias_.X(0);
    gyro_bias_.Y(0);
    gyro_bias_.Z(0);
    acc_stdev_ = 0;
    gyro_bias_range_ = 0;
    acc_bias_walk_stdev_ = 0;
    acc_bias_.X(0);
    acc_bias_.Y(0);
    acc_bias_.Z(0);
  }

  // Initialize the Bias
  gyro_bias_.X(gyro_bias_range_*uniform_distribution_(random_generator_));
  gyro_bias_.Y(gyro_bias_range_*uniform_distribution_(random_generator_));
  gyro_bias_.Z(gyro_bias_range_*uniform_distribution_(random_generator_));
  acc_bias_.X(acc_bias_range_*uniform_distribution_(random_generator_));
  acc_bias_.Y(acc_bias_range_*uniform_distribution_(random_generator_));
  acc_bias_.Z(acc_bias_range_*uniform_distribution_(random_generator_));

  // Set up static members of IMU message
  imu_message_.header.frame_id = link_name_;
  imu_message_.angular_velocity_covariance[0] = gyro_stdev_*gyro_stdev_;
  imu_message_.angular_velocity_covariance[4] = gyro_stdev_*gyro_stdev_;
  imu_message_.angular_velocity_covariance[8] = gyro_stdev_*gyro_stdev_;
  imu_message_.linear_acceleration_covariance[0] = acc_stdev_*acc_stdev_;
  imu_message_.linear_acceleration_covariance[4] = acc_stdev_*acc_stdev_;
  imu_message_.linear_acceleration_covariance[8] = acc_stdev_*acc_stdev_;

  // Connect update callback
  this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ImuPlugin::OnUpdate, this, std::placeholders::_1));
}

void ImuPlugin::Reset()
{
  last_time_ = world_->SimTime();
}

// This gets called by the world update start event.
void ImuPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  // check if time to publish
  gazebo::common::Time current_time = world_->SimTime();
  if ((current_time - last_time_).Double() >= sample_time_)
  {
    ignition::math::Quaterniond q_I_NWU = link_->WorldPose().Rot();
    ignition::math::Vector3d omega_B_NWU = link_->RelativeAngularVel();
    ignition::math::Vector3d uvw_B_NWU = link_->RelativeLinearVel();

    // y_acc = F/m - R*g
    ignition::math::Vector3d y_acc = link_->RelativeForce()/mass_ - q_I_NWU.RotateVectorReverse(gravity_);
    ignition::math::Vector3d y_gyro = link_->RelativeAngularVel();

    // Apply normal noise
    y_acc.X(y_acc.X() + acc_stdev_*normal_distribution_(random_generator_));
    y_acc.Y(y_acc.Y() + acc_stdev_*normal_distribution_(random_generator_));
    y_acc.Z(y_acc.Z() + acc_stdev_*normal_distribution_(random_generator_));
    y_gyro.X(y_gyro.X() + gyro_stdev_*normal_distribution_(random_generator_));
    y_gyro.Y(y_gyro.Y() + gyro_stdev_*normal_distribution_(random_generator_));
    y_gyro.Z(y_gyro.Z() + gyro_stdev_*normal_distribution_(random_generator_));

    // Perform Random Walk for biases
    acc_bias_.X(acc_bias_.X() + acc_bias_walk_stdev_*normal_distribution_(random_generator_));
    acc_bias_.Y(acc_bias_.Y() + acc_bias_walk_stdev_*normal_distribution_(random_generator_));
    acc_bias_.Z(acc_bias_.Z() + acc_bias_walk_stdev_*normal_distribution_(random_generator_));
    gyro_bias_.X(gyro_bias_.X() + gyro_bias_walk_stdev_*normal_distribution_(random_generator_));
    gyro_bias_.Y(gyro_bias_.Y() + gyro_bias_walk_stdev_*normal_distribution_(random_generator_));
    gyro_bias_.Z(gyro_bias_.Z() + gyro_bias_walk_stdev_*normal_distribution_(random_generator_));

    // Add constant Bias to measurement
    y_acc.X(y_acc.X() + acc_bias_.X());
    y_acc.Y(y_acc.Y() + acc_bias_.Y());
    y_acc.Z(y_acc.Z() + acc_bias_.Z());
    y_gyro.X(y_gyro.X() + gyro_bias_.X());
    y_gyro.Y(y_gyro.Y() + gyro_bias_.Y());
    y_gyro.Z(y_gyro.Z() + gyro_bias_.Z());

    // Set Time stamp
    imu_message_.header.stamp.sec = current_time.sec;
    imu_message_.header.stamp.nsec = current_time.nsec;

    // Convert to NED for publishing
    imu_message_.orientation.w = 1;
    imu_message_.orientation.x = 0;
    imu_message_.orientation.y = 0;
    imu_message_.orientation.z = 0;
    imu_message_.orientation.w = q_I_NWU.W();
    imu_message_.orientation.x = q_I_NWU.X();
    imu_message_.orientation.y = -q_I_NWU.Y();
    imu_message_.orientation.z = -q_I_NWU.Z();

    imu_message_.linear_acceleration.x = y_acc.X();
    imu_message_.linear_acceleration.y = -y_acc.Y();
    imu_message_.linear_acceleration.z = -y_acc.Z();
    imu_message_.angular_velocity.x = y_gyro.X();
    imu_message_.angular_velocity.y = -y_gyro.Y();
    imu_message_.angular_velocity.z = -y_gyro.Z();

    imu_pub_.publish(imu_message_);

    last_time_ = current_time;

    // Publish the current value of the biases
    geometry_msgs::Vector3Stamped acc_bias_msg, gyro_bias_msg;
    acc_bias_msg.header = imu_message_.header;
    gyro_bias_msg.header = imu_message_.header;

    // Convert Bias to NED
    acc_bias_msg.vector.x = acc_bias_.X();
    acc_bias_msg.vector.y = -acc_bias_.Y();
    acc_bias_msg.vector.z = -acc_bias_.Z();
    acc_bias_pub_.publish(acc_bias_msg);

    // Convert Bias to NED
    gyro_bias_msg.vector.x = gyro_bias_.X();
    gyro_bias_msg.vector.y = -gyro_bias_.Y();
    gyro_bias_msg.vector.z = -gyro_bias_.Z();
    gyro_bias_pub_.publish(gyro_bias_msg);
  }
}


GZ_REGISTER_MODEL_PLUGIN(ImuPlugin);
}
