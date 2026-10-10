// The law as the follower node flies it: PX4 messages in, attitude setpoint out, for a leader flying straight east.
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

#include "fixed_wing_formation/px4_law_io.hpp"

namespace
{

using namespace fixed_wing_formation;
using px4_msgs::msg::VehicleStatus;

constexpr double HOME_LAT = 47.397742;  // PX4 SITL's default home (PX4_HOME_LAT)
constexpr double HOME_LON = 8.545594;  // PX4_HOME_LON
constexpr float HOME_AMSL = 488.0f;  // PX4_HOME_ALT
constexpr float SPEED = 18.0f;  // m/s, inside the law's 12..22 m/s airspeed setpoint band
constexpr float HEIGHT = 80.0f;  // m over home, chosen apart from the original's fixed 50 m so change (b) shows
constexpr float EAST_YAW = PI / 2;
constexpr float OFF_SLOT = 40.0f;  // m, beside the slot
constexpr float COLUMN_SLOT_AHEAD = 30.0f;  // m, follower 1's slot in formation type 1 (formation_controller.cpp:36)
constexpr float LINE_ABREAST_SLOT_RIGHT = 30.0f;  // m, follower 1's slot in formation type 3 (formation_controller.cpp:68)
constexpr int COLUMN = 1;
constexpr int LINE_ABREAST = 3;
constexpr int FOLLOWER = 1;
constexpr float ANGLE_LIMIT = PI / 4;  // the law's roll_max
constexpr float THROTTLE_MIN = 0.1f;  // the law's throttle_min
constexpr float TECS_UNDERSPEED_THROTTLE = 1.0f;  // tecs.cpp _update_throttle_setpoint()
constexpr int SETTLE_STEPS = 100;  // 2 s of the node's 50 Hz: the law low-passes the leader's velocity by 0.1 a step
constexpr double SLOT_TOLERANCE_M = 0.01;  // chosen
constexpr float PITCH_TOLERANCE_RAD = 1e-3f;  // chosen
constexpr float STALLING = 4.0f;  // m/s, chosen under FW_AIRSPD_STALL's 7 (performance_model_params.c:180)
constexpr float FAST = 40.0f;  // m/s, chosen past the law's 22 m/s maximum airspeed setpoint
constexpr float CLIMB_BELOW = 50.0f;  // m under the leader, chosen past the law's 10 m climb-out threshold (abs_formation_controller.cpp:645)
constexpr float NOSE_UP_DEG = 40.0f;  // chosen past FW_P_LIM_MAX

// Latitude and longitude north / east metres from home, by the law's own conversion.
void fix_at(float north, float east, double fix[3])
{
  double home[3] = {HOME_LAT, HOME_LON, 0.0};
  cov_m_2_lat_long_alt(home, north, east, 0.0f, fix);
}

// A wings-level vehicle flying east at `airspeed` and `height` over home, north / east metres from home.
FwStates east_flyer(float north, float east, float height = HEIGHT, float airspeed = SPEED, float pitch = 0.0f)
{
  px4_msgs::msg::VehicleOdometry odom;
  float euler[3] = {0.0f, pitch, EAST_YAW};
  float q[4];
  euler_2_quaternion(euler, q);
  std::copy(q, q + 4, odom.q.begin());
  px4_msgs::msg::VehicleLocalPosition lpos;
  lpos.vy = airspeed;
  lpos.z = -height;
  double fix[3];
  fix_at(north, east, fix);
  px4_msgs::msg::VehicleGlobalPosition gpos;
  gpos.lat = fix[0];
  gpos.lon = fix[1];
  gpos.alt = HOME_AMSL + height;
  px4_msgs::msg::AirspeedValidated airspeed_msg;
  airspeed_msg.calibrated_airspeed_m_s = airspeed;
  return fw_states_from_px4(odom, lpos, gpos, airspeed_msg);
}

// Follower 1 of formation `type`, after SETTLE_STEPS of the node's ticks.
void settle(ABS_FORMATION_CONTROLLER &law, int type, const FwStates &leader, const FwStates &follower)
{
  std::cout.setstate(std::ios_base::failbit);  // the law prints some 25 lines a step
  configure_law(law, FOLLOWER);
  law.set_formation_type(type);
  const LeaderStates l = leader_states_from(leader);
  for (int i = 0; i < SETTLE_STEPS; ++i) {
    law.update_led_fol_states(&l, &follower);
    ASSERT_TRUE(law.identify_led_fol_states());
    law.control_law();
  }
}

// The law's slot, north / east metres from home.
void slot_from_home(ABS_FORMATION_CONTROLLER &law, double off[2])
{
  FORMATION_CONTROLLER::_s_fw_sp slot;
  law.get_formation_sp(slot);
  double home[2] = {HOME_LAT, HOME_LON};
  double at[2] = {slot.latitude, slot.longitude};
  cov_lat_long_2_m(home, at, off);
}

float pitch_of(ABS_FORMATION_CONTROLLER &law)
{
  FORMATION_CONTROLLER::_s_4cmd cmd;
  law.get_formation_4cmd(cmd);
  return cmd.pitch;
}

TEST(ControlLaw, TurnsTowardTheSlotWithinLimits)
{
  // Flying east, north is left: a follower north of its slot must bank right (positive roll), one south of it left.
  for (const float north_of_slot : {OFF_SLOT, -OFF_SLOT}) {
    ABS_FORMATION_CONTROLLER law;
    const FwStates follower = east_flyer(north_of_slot, COLUMN_SLOT_AHEAD);
    settle(law, COLUMN, east_flyer(0.0f, 0.0f), follower);
    FORMATION_CONTROLLER::_s_4cmd cmd;
    law.get_formation_4cmd(cmd);
    const auto sp = attitude_setpoint_from(cmd, follower.yaw_angle);
    float q[4];
    std::copy(sp.q_d.begin(), sp.q_d.end(), q);
    float euler[3];
    quaternion_2_euler(q, euler);
    const float throttle = sp.thrust_body[0];
    ASSERT_TRUE(std::isfinite(euler[0]) && std::isfinite(euler[1]) && std::isfinite(throttle));
    EXPECT_LE(std::abs(euler[0]), ANGLE_LIMIT);
    EXPECT_GE(throttle, THROTTLE_MIN);
    EXPECT_LE(throttle, FW_THR_MAX);
    EXPECT_GT(north_of_slot * euler[0], 0.0f) << "roll " << euler[0] << " rad, follower " << north_of_slot
                                              << " m north of its slot";

    // Change (a): the column's slot lies ahead along the leader's course, east here; the original put it north.
    double off[2];
    slot_from_home(law, off);
    EXPECT_NEAR(off[0], 0.0, SLOT_TOLERANCE_M);
    EXPECT_NEAR(off[1], COLUMN_SLOT_AHEAD, SLOT_TOLERANCE_M);
    // Change (b): the height to hold is the leader's, AMSL like the follower's own.
    FORMATION_CONTROLLER::_s_fw_sp slot;
    law.get_formation_sp(slot);
    EXPECT_DOUBLE_EQ(slot.altitude, HOME_AMSL + HEIGHT);
  }
}

// Formation type 3 puts follower 1 at yb > 0, the leader's right: south of a leader flying east.
TEST(ControlLaw, LineAbreastPutsFollowerOneOnTheRight)
{
  ABS_FORMATION_CONTROLLER law;
  settle(law, LINE_ABREAST, east_flyer(0.0f, 0.0f), east_flyer(-LINE_ABREAST_SLOT_RIGHT, 0.0f));
  double off[2];
  slot_from_home(law, off);
  EXPECT_NEAR(off[0], -LINE_ABREAST_SLOT_RIGHT, SLOT_TOLERANCE_M);
  EXPECT_NEAR(off[1], 0.0, SLOT_TOLERANCE_M);
}

// PX4 flies an offboard pitch as given, so the law's own limits must be the airframe's.
TEST(ControlLaw, PitchStaysWithinTheAirframesLimits)
{
  ABS_FORMATION_CONTROLLER diving;  // stalling in the slot: nose down for speed
  settle(diving, COLUMN, east_flyer(0.0f, 0.0f), east_flyer(0.0f, COLUMN_SLOT_AHEAD, HEIGHT, STALLING));
  EXPECT_NEAR(pitch_of(diving), deg_2_rad(FW_P_LIM_MIN_DEG), PITCH_TOLERANCE_RAD);
  // Well below and too fast: climb out, nose up. Nose-high already, as TECS slews from the vehicle's pitch at ~1 ms steps.
  ABS_FORMATION_CONTROLLER climbing;
  settle(climbing, COLUMN, east_flyer(0.0f, 0.0f),
         east_flyer(0.0f, COLUMN_SLOT_AHEAD, HEIGHT - CLIMB_BELOW, FAST, deg_2_rad(NOSE_UP_DEG)));
  EXPECT_NEAR(pitch_of(climbing), deg_2_rad(FW_P_LIM_MAX_DEG), PITCH_TOLERANCE_RAD);
}

// Change (c), where TECS itself ignores its throttle_max.
TEST(AttitudeSetpoint, ClampsTheUnderspeedThrottle)
{
  FORMATION_CONTROLLER::_s_4cmd cmd;
  cmd.thrust = TECS_UNDERSPEED_THROTTLE;
  EXPECT_FLOAT_EQ(attitude_setpoint_from(cmd, 0.0f).thrust_body[0], FW_THR_MAX);
}

VehicleStatus status_of(uint8_t vehicle_type, bool in_transition)
{
  VehicleStatus s;
  s.vehicle_type = vehicle_type;
  s.in_transition_mode = in_transition;
  return s;
}

TEST(HeartbeatGating, OnlyWhileBothFlyAsAeroplanes)
{
  const VehicleStatus fw = status_of(VehicleStatus::VEHICLE_TYPE_FIXED_WING, false);
  const VehicleStatus mc = status_of(VehicleStatus::VEHICLE_TYPE_ROTARY_WING, false);
  // Commander reports a transition as rotary wing (Commander.cpp:2231); the flag must block on its own all the same.
  const VehicleStatus transition = status_of(VehicleStatus::VEHICLE_TYPE_FIXED_WING, true);
  EXPECT_EQ(heartbeat_blocked(&fw, &fw), nullptr);
  EXPECT_NE(heartbeat_blocked(&mc, &fw), nullptr);
  EXPECT_NE(heartbeat_blocked(&transition, &fw), nullptr);
  EXPECT_NE(heartbeat_blocked(&fw, &mc), nullptr);
  EXPECT_NE(heartbeat_blocked(&fw, &transition), nullptr);
  EXPECT_NE(heartbeat_blocked(nullptr, &fw), nullptr);
  EXPECT_NE(heartbeat_blocked(&fw, nullptr), nullptr);
}

}  // namespace
