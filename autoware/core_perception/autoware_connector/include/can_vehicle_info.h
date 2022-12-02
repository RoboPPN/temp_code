/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAN_VEHICLE_INFO_H
#define CAN_VEHICLE_INFO_H

namespace autoware_connector
{
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60); //km/h --> m/s
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

/**
 * @brief 角度转化为弧度
 * 
 * @param deg 
 * @return double 
 */
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

/**
 * @brief 弧度转化为角度
 * 
 * @param rad 
 * @return double 
 */
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}
/**
 * @brief 结构体：下位机传送上来的车辆信息
 * 
 */
struct VehicleInfo
{
  bool is_stored; //是否要进行数据处理
  double wheel_base;  
  double minimum_turning_radius;  //最小转弯半径
  double maximum_steering_wheel_angle_deg;   //最大转向角

  VehicleInfo()
  {
    is_stored = false;
    wheel_base = 0.0;
    minimum_turning_radius = 0.0;
    maximum_steering_wheel_angle_deg = 0.0;
  }
  /**
   * @brief 转向角转换成角速度
   * 
   * @param cur_vel_mps 
   * @param cur_angle_rad 
   * @return double 
   */
  double convertSteeringAngleToAngularVelocity(const double cur_vel_mps, const double cur_angle_rad)  // rad/s
  {
    return is_stored ? tan(cur_angle_rad) * cur_vel_mps / wheel_base : 0;
  }
  /**
   * @brief 获取当前转向角
   * 
   * @param steering_wheel_angle_rad 
   * @return double 
   */
  double getCurrentSteeringAngle(const double steering_wheel_angle_rad)  // steering wheel [rad] -> steering [rad]
  {
    return is_stored ?
               steering_wheel_angle_rad * getMaximumSteeringWheelAngle() / deg2rad(maximum_steering_wheel_angle_deg) :
               0;
  }
  /**
   * @brief 获取最大车轮转向角度
   * 
   * @return double 
   */
  double getMaximumSteeringWheelAngle()  // radian
  {
    return is_stored ? asin(wheel_base / minimum_turning_radius) : 0;
  }
};
}
#endif  // CAN_VEHICLE_INFO_H
