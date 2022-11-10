//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

namespace rc_telep
{

class Teleop
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_raw_subscriber_;
  ros::Publisher joy_publisher_;

  struct Axis
  {
    Axis()
      : axis(0), factor(0.0), offset(0.0)
    {}

    int axis;
    double factor;
    double offset;
  };

  struct Button
  {
    Button()
      : button(0)
    {}

    int button;
  };

  struct
  {
    Axis roll;
    Axis pitch;
    Axis x;
    Axis y;
    Axis z;
    Axis thrust;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
    Button go;
    Button stop;
    Button interrupt;
  } buttons_;

  double slow_factor_;

public:
  Teleop()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("roll_axis", axes_.x.axis, 10);
    private_nh.param<int>("pitch_axis", axes_.x.axis, 10);
    private_nh.param<int>("x_axis", axes_.x.axis, 5);
    private_nh.param<int>("y_axis", axes_.y.axis, 4);
    private_nh.param<int>("z_axis", axes_.z.axis, 2);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, 3);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);

    private_nh.param<double>("yaw_velocity_max", axes_.yaw.factor, 1.0);

    private_nh.param<int>("slow_button", buttons_.slow.button, 4);
    private_nh.param<int>("go_button", buttons_.go.button, 7);
    private_nh.param<int>("stop_button", buttons_.stop.button, 2);
    private_nh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
    private_nh.param<double>("slow_factor", slow_factor_, 0.2);

    private_nh.param<double>("pitch_max", axes_.x.factor, 1.0);
    private_nh.param<double>("roll_max", axes_.y.factor, 1.0);
    private_nh.param<double>("thrust_max", axes_.thrust.factor, 1.0);
    private_nh.param<double>("thrust_offset", axes_.thrust.offset, 0.0);

    joy_raw_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy_raw", 1,
                                                               boost::bind(&Teleop::joyRemapRCCallback, this, _1));

    joy_publisher_ = node_handle_.advertise<sensor_msgs::Joy>("joy", 1);
  }

  ~Teleop()
  {
    stop();
  }

  void joyRemapRCCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    sensor_msgs::Joy joy_;
    std_msgs::Float32 axes_ [6] ;
    std_msgs::Int32 buttons_ [12] ;

    joy_.header = joy->header;
    joy_.axes = joy->axes;

    joy_.axes[0] = joy->axes[3];
    joy_.axes[1] = -joy->axes[1];
    joy_.axes[2] = joy->axes[0];
    joy_.axes[3] = joy->axes[2];
    joy_.axes[4] = joy->axes[4];
    joy_.axes[5] = joy->axes[5];

    joy_.buttons = joy->buttons;
    joy_.buttons[6] = joy->buttons[3];
    joy_.buttons[3] = joy->buttons[6];

    joy_publisher_.publish(joy_);
    // double rudder, sail;

    // rudder_cmd.data = -getAxis(joy, axes_.y) * M_PI/180.0;
    // sail_cmd.data = getAxis(joy, axes_.thrust) * M_PI/180.0;
    // motor_cmd.data = getAxis(joy, axes_.x);

    // rudder_joint_publisher_.publish(rudder_cmd);
    // sail_joint_publisher_.publish(sail_cmd);
    // motor_joint_publisher_.publish(motor_cmd);
  }

double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
      return 0.0;
    }

    double output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;

    return output;
  }

  bool getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button)
  {
    if (button.button <= 0 || button.button > joy->buttons.size())
    {
      ROS_ERROR_STREAM("Button " << button.button << " out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }

    return joy->buttons[button.button - 1] > 0;
  }

  void stop()
  {
    if (joy_publisher_.getNumSubscribers() > 0) 
    {
      joy_publisher_.publish(sensor_msgs::Joy());
    }
  }
};

} // namespace uav

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rc_teleop");

  rc_telep::Teleop teleop;
  ros::spin();

  return 0;
}