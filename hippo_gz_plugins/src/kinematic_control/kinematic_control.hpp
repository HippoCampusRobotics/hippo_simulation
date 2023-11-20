#ifndef HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP
#define HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP

#include <ignition/msgs/vector3d.pb.h>

#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"

namespace kinematic_control {
class KinematicControlPrivate {
  /// \brief Callback for twist msg subscription
 public:
  void OnVelCmd(const ignition::msgs::Twist &_msg);

  /// \brief Ignition communication node.
 public:
  ignition::transport::Node node;

 public:
  ignition::gazebo::Link link;
  /// \brief Link Entity
 public:
  ignition::gazebo::Entity linkEntity;

  /// \brief Commanded linear velocity
 public:
  ignition::math::Vector3d linearVelCmd;

 public:
  ignition::math::Vector3d lastLinearVelCmd;

 public:
  ignition::math::Vector3d offsetsLinearVelCmd;

  /// \brief Commanded angular velocity
 public:
  ignition::math::Vector3d angularVelCmd;

 public:
  ignition::math::Vector3d lastAngularVelCmd;

 public:
  ignition::math::Vector3d offsetsAngularVelCmd;

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
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
};

class KinematicControl : public ignition::gazebo::System,
                         public ignition::gazebo::ISystemConfigure,
                         public ignition::gazebo::ISystemPreUpdate {
 public:
  KinematicControl();

 public:
  ~KinematicControl() override = default;

 public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

 public:
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<KinematicControlPrivate> dataPtr;
};
}  // namespace kinematic_control

#endif  // HIPPO_GZ_PLUGINS_KINEMATIC_CONTROL_HPP
