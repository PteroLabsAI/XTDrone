// One follower of XTDrone's fixed-wing VTOL formation on PX4's own ROS 2 interface: the law's roll, pitch and throttle
// as an OFFBOARD attitude setpoint. The heartbeat goes out while the law can fly (fresh state, finite outputs, both
// vehicles in fixed-wing flight), so PX4 leaves OFFBOARD on its own when it cannot; the setpoint only in OFFBOARD,
// since in AUTO vtol_att_control writes the same uORB topic.
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "fixed_wing_formation/px4_law_io.hpp"

namespace fixed_wing_formation
{

using px4_msgs::msg::AirspeedValidated;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleControlMode;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

constexpr double CONTROL_HZ = 50.0;  // task_main's loop; tecs.hpp wants its state update at 50 Hz or more
// Twice commander's slowest vehicle_status and vehicle_control_mode period (500 ms, Commander.cpp:1990).
constexpr double STATE_TIMEOUT_S = 1.0;
constexpr int FOLLOWERS = 2;  // set_formation_type() has a slot for plane 1 and one for the other
constexpr int FORMATION_TYPES = 3;  // set_formation_type(): 1 column, 2 triangle, 3 line abreast
constexpr int WARN_PERIOD_MS = 2000;  // chosen
constexpr int PROGRESS_PERIOD_MS = 2000;  // chosen
constexpr size_t TYPE_QUEUE_DEPTH = 1;  // only the newest figure matters
constexpr char TYPE_TOPIC[] = "/fw_formation/type";

// PX4's client publishes best effort (uxrce_dds_client/utilities.hpp), as px4.py notes; the ROS 2 port reads and writes it so.
const rclcpp::QoS PX4_QOS = rclcpp::SensorDataQoS();

template <class T, class = void>
struct MessageVersion : std::integral_constant<uint32_t, 0> {};
template <class T>
struct MessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>> : std::integral_constant<uint32_t, T::MESSAGE_VERSION> {};

// /<ns>/fmu/<in|out>/<name>, with _v<N> for a versioned message: the name PX4's client gives it, as in px4.py.
template <class T>
std::string px4_topic(const std::string &ns, const std::string &direction, const std::string &name)
{
  const uint32_t version = MessageVersion<T>::value;
  return "/" + ns + "/fmu/" + direction + "/" + name + (version ? "_v" + std::to_string(version) : "");
}

template <class T>
struct Latest
{
  typename T::ConstSharedPtr msg;
  rclcpp::Time at;
  bool fresh(const rclcpp::Time &now) const { return msg && (now - at).seconds() < STATE_TIMEOUT_S; }
};

template <class T>
rclcpp::SubscriptionBase::SharedPtr listen(rclcpp::Node &node, const std::string &ns, const std::string &name, Latest<T> &latest)
{
  return node.create_subscription<T>(px4_topic<T>(ns, "out", name), PX4_QOS, [&node, &latest](typename T::ConstSharedPtr m) {
    latest.msg = m;
    latest.at = node.now();
  });
}

// Everything the law and its gating read of one vehicle.
class Px4Vehicle
{
public:
  Px4Vehicle(rclcpp::Node &node, const std::string &ns)
  : ns(ns),
    subscriptions_{listen(node, ns, "vehicle_odometry", odom_), listen(node, ns, "vehicle_local_position", lpos_),
                   listen(node, ns, "vehicle_global_position", gpos_), listen(node, ns, "airspeed_validated", airspeed_),
                   listen(node, ns, "vehicle_status", status_)}
  {
  }

  // The first stream of the law's that is stale or invalid, nullptr when the law can use this vehicle.
  const char *missing(const rclcpp::Time &now) const
  {
    if (!odom_.fresh(now) || !std::isfinite(odom_.msg->q[0])) return "vehicle_odometry";
    if (!lpos_.fresh(now) || !lpos_.msg->z_valid || !lpos_.msg->v_xy_valid) return "vehicle_local_position";
    if (!gpos_.fresh(now) || !gpos_.msg->lat_lon_valid || !gpos_.msg->alt_valid) return "vehicle_global_position";
    // TECS would fly a NaN airspeed as the middle of its range; better not to fly at all.
    if (!airspeed_.fresh(now) || !std::isfinite(airspeed_.msg->calibrated_airspeed_m_s)) return "airspeed_validated";
    return nullptr;
  }

  FwStates states() const { return fw_states_from_px4(*odom_.msg, *lpos_.msg, *gpos_.msg, *airspeed_.msg); }

  const VehicleStatus *status(const rclcpp::Time &now) const { return status_.fresh(now) ? status_.msg.get() : nullptr; }

  const std::string ns;

private:
  Latest<VehicleOdometry> odom_;
  Latest<VehicleLocalPosition> lpos_;
  Latest<VehicleGlobalPosition> gpos_;
  Latest<AirspeedValidated> airspeed_;
  Latest<VehicleStatus> status_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

class FwFormationFollower : public rclcpp::Node
{
public:
  FwFormationFollower()
  : Node("fw_formation_follower"),
    self_(*this, declare_parameter<std::string>("vehicle_ns")),
    leader_(*this, declare_parameter<std::string>("leader_ns")),
    index_(declare_parameter<int>("follower_index"))
  {
    if (index_ < 1 || index_ > FOLLOWERS) {
      throw std::invalid_argument("follower_index " + std::to_string(index_) + ": the law has slots 1 to " + std::to_string(FOLLOWERS));
    }
    configure_law(law_, index_);
    const int type = declare_parameter<int>("formation_type");
    if (!set_formation_type(type)) {
      throw std::invalid_argument("formation_type " + std::to_string(type) + ": the law knows 1 to " + std::to_string(FORMATION_TYPES));
    }
    heartbeat_.attitude = true;
    // 50 Hz over the bridge, vehicle_status 5 (dds_topics.yaml:69, :92): the setpoint stops a tick after OFFBOARD does.
    control_mode_sub_ = listen(*this, self_.ns, "vehicle_control_mode", control_mode_);
    // Transient local: a figure published before this side matched the publisher still arrives.
    const auto type_qos = rclcpp::QoS(TYPE_QUEUE_DEPTH).reliable().transient_local();
    type_sub_ = create_subscription<std_msgs::msg::UInt8>(TYPE_TOPIC, type_qos, [this](std_msgs::msg::UInt8::ConstSharedPtr m) {
      if (set_formation_type(m->data)) {
        RCLCPP_INFO(get_logger(), "formation type %d", type_);
      } else {
        RCLCPP_WARN(get_logger(), "formation type %d refused: the law knows 1 to %d", m->data, FORMATION_TYPES);
      }
    });
    mode_pub_ = create_publisher<OffboardControlMode>(px4_topic<OffboardControlMode>(self_.ns, "in", "offboard_control_mode"), PX4_QOS);
    setpoint_pub_ = create_publisher<VehicleAttitudeSetpoint>(
      px4_topic<VehicleAttitudeSetpoint>(self_.ns, "in", "vehicle_attitude_setpoint"), PX4_QOS);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / CONTROL_HZ), [this] { tick(); });
  }

private:
  bool set_formation_type(int type)
  {
    if (type < 1 || type > FORMATION_TYPES) return false;
    type_ = type;
    law_.set_formation_type(type);
    return true;
  }

  void tick()
  {
    const rclcpp::Time now = this->now();
    for (const Px4Vehicle *v : {&self_, &leader_}) {
      if (const char *what = v->missing(now)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_PERIOD_MS, "no fresh, valid %s from %s: no offboard heartbeat",
                             what, v->ns.c_str());
        return;
      }
    }
    const FwStates fw = self_.states();
    const LeaderStates leader = leader_states_from(leader_.states());
    law_.update_led_fol_states(&leader, &fw);
    // control_law() returns without new outputs in this case, so its old ones must not go out.
    if (!law_.identify_led_fol_states()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_PERIOD_MS,
                           "%s or %s not flying (ground speed and height both 3 m or less): no offboard heartbeat",
                           leader_.ns.c_str(), self_.ns.c_str());
      return;
    }
    const bool offboard = control_mode_.fresh(now) && control_mode_.msg->flag_control_offboard_enabled;
    // task_main reset the law on every mode change; entering OFFBOARD is the one whose outputs fly.
    if (offboard != was_offboard_) law_.reset_formation_controller();
    was_offboard_ = offboard;
    law_.control_law();
    FORMATION_CONTROLLER::_s_4cmd cmd;
    law_.get_formation_4cmd(cmd);
    if (!std::isfinite(cmd.roll) || !std::isfinite(cmd.pitch) || !std::isfinite(cmd.thrust)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_PERIOD_MS,
                           "law output not finite (roll %f, pitch %f, throttle %f): no offboard heartbeat", cmd.roll,
                           cmd.pitch, cmd.thrust);
      return;
    }
    if (const char *why = heartbeat_blocked(self_.status(now), leader_.status(now))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_PERIOD_MS, "%s: no offboard heartbeat", why);
      return;
    }
    mode_pub_->publish(heartbeat_);
    if (offboard) setpoint_pub_->publish(attitude_setpoint_from(cmd, fw.yaw_angle));
    FORMATION_CONTROLLER::_s_fw_error error;
    law_.get_formation_error(error);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), PROGRESS_PERIOD_MS,
                         "type %d: %.1f m ahead, %.1f m right of the slot; roll %.1f, pitch %.1f deg, throttle %.2f%s", type_,
                         error.PXb, error.PYb, rad_2_deg(cmd.roll), rad_2_deg(cmd.pitch), cmd.thrust, offboard ? "" : " (not flown)");
  }

  ABS_FORMATION_CONTROLLER law_;
  Px4Vehicle self_;
  Px4Vehicle leader_;
  const int index_;
  int type_{0};
  bool was_offboard_{false};
  Latest<VehicleControlMode> control_mode_;
  OffboardControlMode heartbeat_;
  rclcpp::SubscriptionBase::SharedPtr control_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr type_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fixed_wing_formation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // The law prints some 25 lines a step, 50 steps a second; the node logs what matters itself.
  std::cout.setstate(std::ios_base::failbit);
  rclcpp::spin(std::make_shared<fixed_wing_formation::FwFormationFollower>());
  rclcpp::shutdown();
  return 0;
}
