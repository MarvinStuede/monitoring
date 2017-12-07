/*
 * max.cpp
 *
 *  Created on: Dec 6, 2017
 *      Author: matthias
 */

#include "max.h"

Max::Max(float value, std::string errormsg, float errorLevel, ros::Publisher& publisher)
  :maxValue(value)
  ,msg(errormsg)
  ,errorlevel(errorLevel)
  ,ConfigInterface(publisher)
{
}

Max::~Max() {}


void Max::check(ros_monitoring::KeyValue newMsg) {
  ROS_INFO("Checking CPU TEMP %s", newMsg.value);

  std::string::size_type sz;
  float value = std::stof (newMsg.value,&sz);
  if(maxValue< value) {
    ROS_ERROR("ERROR: %s is higher then expected, Errorlevel to %f", newMsg.key, errorlevel);
    ros_monitoring::Error errormsg;
    errormsg.key = msg;
    errormsg.value = newMsg.value;
    errormsg.level = errorlevel;
    publishError(errormsg);
  }

}