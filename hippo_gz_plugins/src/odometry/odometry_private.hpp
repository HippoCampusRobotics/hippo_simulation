#pragma once
#include <gz/msgs/odometry.pb.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace odometry {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(gz::sim::EntityComponentManager &_ecm,
                 gz::sim::Entity _entity);
  void Publish(const gz::sim::EntityComponentManager &_ecm,
               const gz::msgs::Time &stamp);
  void Advertise();

  //! Publishes linear and angular acceleration as combined twist message in
  //! body frame
  void PublishAcceleration(
      const gz::sim::EntityComponentManager &_ecm,
      const std::chrono::steady_clock::duration &_sim_time);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration angular_velocity_update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};
  std::chrono::steady_clock::duration last_angular_velocity_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    double angular_velocity_update_rate{250.0};
    std::string base_topic{"odometry"};
  } sdf_params_;

  void InitHeader();
  void InitComponents(gz::sim::EntityComponentManager &_ecm);
  std::string OdometryTopicName();
  std::string WorldLinearAccelerationTopicName();
  std::string AccelerationsTopicName();

  //! Publishes linear acceleration in inertial coordinate system
  void PublishWorldLinearAcceleration(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::msgs::Time &_stamp);

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  gz::transport::Node::Publisher odometry_pub_;
  gz::transport::Node::Publisher
      world_linear_acceleration_pub_;  //!< publishes linear acceleration in
                                       //!< inertial COS
  gz::transport::Node::Publisher
      accelerations_pub_;  //!< publishes linear and angular local acceleration
  gz::msgs::Odometry msg_;
};
}  // namespace odometry
