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
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

namespace rc_teleop_txs
{

class TeleopTxs
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
  TeleopTxs()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("roll_axis", axes_.roll.axis, 4);
    private_nh.param<int>("pitch_axis", axes_.pitch.axis, 2);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, 3);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);

    private_nh.param<double>("yaw_max", axes_.yaw.factor, 1.0);
    private_nh.param<double>("yaw_offset", axes_.yaw.offset, 0.0);

    private_nh.param<double>("roll_max", axes_.roll.factor, 1.0);
    private_nh.param<double>("roll_offset", axes_.roll.offset, 0.0);

    private_nh.param<double>("pitch_max", axes_.pitch.factor, 1.0);
    private_nh.param<double>("pitch_offset", axes_.pitch.offset, 0.0);

    private_nh.param<double>("thrust_max", axes_.thrust.factor, 1.0);
    private_nh.param<double>("thrust_offset", axes_.thrust.offset, 0.0);

    private_nh.param<int>("slow_button", buttons_.slow.button, 4);
    private_nh.param<int>("go_button", buttons_.go.button, 7);
    private_nh.param<int>("stop_button", buttons_.stop.button, 2);
    private_nh.param<int>("interrupt_button", buttons_.interrupt.button, 3);
    private_nh.param<double>("slow_factor", slow_factor_, 0.2);

    joy_raw_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy_raw", 1,
                                                               boost::bind(&TeleopTxs::joyRemapRCCallback, this, _1));

    joy_publisher_ = node_handle_.advertise<mavros_msgs::RCIn>("mavros/rc/in", 1);
  }

  ~TeleopTxs()
  {
    stop();
  }

  void joyRemapRCCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    sensor_msgs::Joy joy_;
    mavros_msgs::RCIn rcin_;

    rcin_.header = joy->header;
    joy_.axes = joy->axes;

    rcin_.channels = {0, 0, 0, 0, 0, 0, 0, 0};

    rcin_.channels[0] = getAxis(joy, axes_.yaw);
    rcin_.channels[1] = getAxis(joy, axes_.pitch);
    rcin_.channels[2] = getAxis(joy, axes_.thrust);
    rcin_.channels[3] = getAxis(joy, axes_.roll);

    // rcin_.buttons = joy->buttons;
    // rcin_.buttons[6] = getButton(joy, buttons_.go);
    // rcin_.buttons[buttons_.go.button-1] = joy->buttons[6];

    joy_publisher_.publish(rcin_);
  }

uint16_t getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
      return 0.0;
    }

    float output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.factor + axis.offset;

    return (uint16_t) output;
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
  ros::init(argc, argv, "rc_teleop_txs");

  rc_teleop_txs::TeleopTxs teleop;
  ros::spin();

  return 0;
}