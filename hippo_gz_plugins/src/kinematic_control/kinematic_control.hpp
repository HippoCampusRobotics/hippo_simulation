#ifndef HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP
#define HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"

namespace kinematic_control {
class KinematicControlPrivate {
  /// \brief Callback for twist msg subscription
 public:
  void OnVelCmd(const gz::msgs::Twist &_msg);

  /// \brief Ignition communication node.
 public:
  gz::transport::Node node;

 public:
  gz::sim::Link link;
  /// \brief Link Entity
 public:
  gz::sim::Entity linkEntity;

  /// \brief Commanded linear velocity
 public:
  gz::math::Vector3d linearVelCmd;

 public:
  gz::math::Vector3d lastLinearVelCmd;

 public:
  gz::math::Vector3d offsetsLinearVelCmd;

  /// \brief Commanded angular velocity
 public:
  gz::math::Vector3d angularVelCmd;

 public:
  gz::math::Vector3d lastAngularVelCmd;

 public:
  gz::math::Vector3d offsetsAngularVelCmd;

 public:
  bool first_update;

  /// \brief Smoothing factor for calculating offsets based on difference
  /// between current state and last command
 public:
  double smoothing_fac;
  /// \brief mutex to protect linearVelCmd
 public:
  std::mutex linearVelCmdMutex;

  /// \brief mutex to protect angularVelCmd
 public:
  std::mutex angularVelCmdMutex;

  /// \brief Model interface
 public:
  gz::sim::Model model{gz::sim::kNullEntity};
};

class KinematicControl : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate {
 public:
  KinematicControl();

 public:
  ~KinematicControl() override = default;

 public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

 public:
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<KinematicControlPrivate> dataPtr;
};
}  // namespace kinematic_control

#endif  // HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP
