// XTDrone's fixed-wing formation law fed from PX4's uXRCE-DDS messages, as pack_fw_states and task_main fed it from
// MAVROS. PX4 publishes NED / FRD, the frames the law works in, so none of MAVROS's ENU / FLU swaps remain.
#pragma once

#include <algorithm>

#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

// After every ROS header: the law's mathlib.hpp defines PI as a macro and opens namespace std.
#include "formation_controller/abs_formation_controller.hpp"

namespace fixed_wing_formation
{

// Port change (c): FW_THR_MAX of our airframe 22003_pterosim_standard_vtol; PX4 does not clamp an offboard FW throttle.
constexpr float FW_THR_MAX = 0.6f;
// Nor an offboard pitch: FW_P_LIM_MIN of airframe 22003, FW_P_LIM_MAX's default (fw_mode_manager_params.c:152).
constexpr float FW_P_LIM_MIN_DEG = -15.0f;
constexpr float FW_P_LIM_MAX_DEG = 30.0f;

using FwStates = FORMATION_CONTROLLER::_s_fw_states;
using LeaderStates = FORMATION_CONTROLLER::_s_leader_states;

inline void configure_law(ABS_FORMATION_CONTROLLER &law, int follower_index)
{
  FORMATION_CONTROLLER::_s_fw_model_params params;
  params.throttle_max = FW_THR_MAX;
  params.pitch_min_rad = deg_2_rad(FW_P_LIM_MIN_DEG);
  params.pitch_max_rad = deg_2_rad(FW_P_LIM_MAX_DEG);
  law.set_fw_model_params(params);
  law.set_fw_planeID(follower_index);
}

// One vehicle, field for field as task_main's control_formation() filled the follower.
inline FwStates fw_states_from_px4(const px4_msgs::msg::VehicleOdometry &odom,
                                   const px4_msgs::msg::VehicleLocalPosition &lpos,
                                   const px4_msgs::msg::VehicleGlobalPosition &gpos,
                                   const px4_msgs::msg::AirspeedValidated &airspeed)
{
  FwStates s;
  s.latitude = gpos.lat;
  s.longitude = gpos.lon;
  // AMSL: one reference for all vehicles. task_main fed MAVROS's rel_alt, AMSL over home (GLOBAL_POSITION_INT.hpp:77).
  s.altitude = gpos.alt;
  // Read only by identify_led_fol_states (> 3 m: flying), which the fixed-wing gating already implies.
  s.relative_alt = -lpos.z;
  s.global_vel_x = s.ned_vel_x = lpos.vx;
  s.global_vel_y = s.ned_vel_y = lpos.vy;
  s.global_vel_z = s.ned_vel_z = lpos.vz;
  s.air_speed = airspeed.calibrated_airspeed_m_s;  // what PX4 sends as VFR_HUD airspeed, MAVROS's source
  std::copy(odom.q.begin(), odom.q.end(), s.att_quat);
  quat_2_rotmax(s.att_quat, s.rotmat);
  float euler[3];
  quaternion_2_euler(s.att_quat, euler);
  s.roll_angle = euler[0];
  s.pitch_angle = euler[1];
  s.yaw_angle = euler[2] < 0 ? euler[2] + 2 * PI : euler[2];  // pack_fw_states kept yaw in [0, 2 pi)
  s.yaw_rate = odom.angular_velocity[2];
  // TECS wants body x specific force (it adds rotMat[2][0] * g back): f = R^T (a - g) from PX4's NED acceleration.
  const float down_minus_g = lpos.az - CONSTANTS_ONE_G;
  s.body_acc[0] = s.rotmat[0][0] * lpos.ax + s.rotmat[1][0] * lpos.ay + s.rotmat[2][0] * down_minus_g;
  s.body_acc[1] = s.rotmat[0][1] * lpos.ax + s.rotmat[1][1] * lpos.ay + s.rotmat[2][1] * down_minus_g;
  // The law passes body_acc[2] to TECS as az, read as minus the height acceleration: NED down, gravity removed.
  s.body_acc[2] = lpos.az;
  s.altitude_lock = true;  // task_main set both "to keep TECS running"
  s.in_air = true;
  return s;
}

// The leader from the same fields, as vir_sim_leader copied them over.
inline LeaderStates leader_states_from(const FwStates &fw)
{
  LeaderStates l;
  l.air_speed = fw.air_speed;
  l.altitude = fw.altitude;
  l.relative_alt = fw.relative_alt;
  l.latitude = fw.latitude;
  l.longitude = fw.longitude;
  l.global_vel_x = fw.global_vel_x;
  l.global_vel_y = fw.global_vel_y;
  l.global_vel_z = fw.global_vel_z;
  l.ned_vel_x = fw.ned_vel_x;
  l.ned_vel_y = fw.ned_vel_y;
  l.ned_vel_z = fw.ned_vel_z;
  l.roll_angle = fw.roll_angle;
  l.pitch_angle = fw.pitch_angle;
  l.yaw_angle = fw.yaw_angle;
  l.yaw_rate = fw.yaw_rate;
  return l;
}

inline bool fixed_wing_flight(const px4_msgs::msg::VehicleStatus &s)
{
  return s.vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING && !s.in_transition_mode;
}

// Why the follower must not hold OFFBOARD, nullptr when it may (a stale status is nullptr): the law flies aeroplanes only.
inline const char *heartbeat_blocked(const px4_msgs::msg::VehicleStatus *self, const px4_msgs::msg::VehicleStatus *leader)
{
  if (!self) return "no fresh vehicle_status of its own";
  if (!leader) return "no fresh vehicle_status of the leader";
  if (!fixed_wing_flight(*self)) return "not in fixed-wing flight";
  if (!fixed_wing_flight(*leader)) return "leader not in fixed-wing flight";
  return nullptr;
}

// PX4's FW attitude control takes roll and pitch from q_d and ignores its yaw; thrust_body[0] is the FW throttle.
// The timestamp stays 0: PX4 stamps a message on arrival.
inline px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_from(const FORMATION_CONTROLLER::_s_4cmd &cmd, float yaw)
{
  px4_msgs::msg::VehicleAttitudeSetpoint sp;
  float euler[3] = {cmd.roll, cmd.pitch, yaw};
  float q[4];
  euler_2_quaternion(euler, q);
  std::copy(q, q + 4, sp.q_d.begin());
  // Again here: TECS's underspeed branch writes a throttle of 1 past any throttle_max.
  sp.thrust_body[0] = std::min(cmd.thrust, FW_THR_MAX);
  return sp;
}

}  // namespace fixed_wing_formation
