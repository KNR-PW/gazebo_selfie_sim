/** 
* Copyright ( c ) 2019, KNR Selfie 
* This code is licensed under BSD license (see LICENSE for details) 
**/

#ifndef GAZEBO_PLUGINS_BLINKVISUALPLUGIN_HH_
#define GAZEBO_PLUGINS_BLINKVISUALPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace gazebo
{
class IndicatorPluginPrivate;
class GZ_PLUGIN_VISIBLE IndicatorPlugin : public VisualPlugin
{
public:
  IndicatorPlugin();

public:
  ~IndicatorPlugin();

public:
  virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

private:
  void Update();

private:
  void OnInfo(ConstPosesStampedPtr& _msg);

private:
  std::unique_ptr<IndicatorPluginPrivate> dataPtr;

private:
  void changeColor(const ignition::math::Color& color);

private:
  void indicatorCallback(const std_msgs::Bool& msg);

private:
  void timerCallback(const ros::TimerEvent& msg);
};
}  // namespace gazebo
#endif
