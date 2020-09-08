/** 
* Copyright ( c ) 2019, KNR Selfie 
* This code is licensed under BSD license (see LICENSE for details) 
**/
#include <mutex> //NOLINT

#include <ignition/math/Color.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include "gazebo_selfie_sim/IndicatorPlugin.hh"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <functional>
#include <string>

namespace gazebo
{
class IndicatorPluginPrivate
{
public:
  rendering::VisualPtr visual;

public:
  event::ConnectionPtr updateConnection;

public:
  ignition::math::Color onColor;

public:
  ignition::math::Color offColor;

public:
  double period;

public:
  std::string topic;

public:
  ros::NodeHandle nh;

public:
  ros::Subscriber sub;

public:
  ros::Timer tim;

public:
  std::mutex mutex;

public:
  bool on;

public:
  bool onLight;
};
}  // namespace gazebo

using namespace gazebo; //NOLINT


GZ_REGISTER_VISUAL_PLUGIN(IndicatorPlugin)

IndicatorPlugin::IndicatorPlugin() : dataPtr(new IndicatorPluginPrivate)
{
}

IndicatorPlugin::~IndicatorPlugin()
{
}

void IndicatorPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  this->dataPtr->onColor.Set(1, 0.6, 0, 1);
  if (_sdf->HasElement("on_color"))
    this->dataPtr->onColor = _sdf->Get<ignition::math::Color>("on_color");

  this->dataPtr->onLight = false;
  this->changeColor(this->dataPtr->offColor);

  this->dataPtr->offColor.Set(0, 0, 0, 1);
  if (_sdf->HasElement("off_color"))
    this->dataPtr->offColor = _sdf->Get<ignition::math::Color>("off_color");

  this->dataPtr->period = 1;
  if (_sdf->HasElement("period"))
    this->dataPtr->period = _sdf->Get<double>("period");

  if (_sdf->HasElement("topic"))
    this->dataPtr->topic = _sdf->Get<std::string>("topic");

  if (this->dataPtr->period <= 0)
  {
    gzerr << "Period can't be lower than zero." << std::endl;
    return;
  }

  this->dataPtr->nh = ros::NodeHandle();
  this->dataPtr->sub =
      this->dataPtr->nh.subscribe(this->dataPtr->topic, 1000, &IndicatorPlugin::indicatorCallback, this);
  this->dataPtr->tim = this->dataPtr->nh.createTimer(ros::Duration(this->dataPtr->period / 2),
                                                     &IndicatorPlugin::timerCallback, this, false, false);
}

void IndicatorPlugin::changeColor(const ignition::math::Color& color)
{
  this->dataPtr->visual->SetDiffuse(color);
  this->dataPtr->visual->SetAmbient(color);
  this->dataPtr->visual->SetTransparency(1 - color.A());
}

void IndicatorPlugin::indicatorCallback(const std_msgs::Bool& msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (msg.data)
  {
    if (!this->dataPtr->on)
    {
      this->changeColor(this->dataPtr->onColor);
      this->dataPtr->tim.start();
      this->dataPtr->on = true;
      this->dataPtr->onLight = true;
    }
  }
  else
  {
    this->dataPtr->on = false;
  }
}

void IndicatorPlugin::timerCallback(const ros::TimerEvent& time)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->onLight)
  {
    this->changeColor(this->dataPtr->offColor);
    this->dataPtr->onLight = false;
  }
  else if (this->dataPtr->on)
  {
    this->changeColor(this->dataPtr->onColor);
    this->dataPtr->onLight = true;
  }

  if (!this->dataPtr->on)
  {
    this->dataPtr->tim.stop();
  }
}
