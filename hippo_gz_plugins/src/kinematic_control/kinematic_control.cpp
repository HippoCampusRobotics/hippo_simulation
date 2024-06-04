#include "kinematic_control.hpp"

#include <gz/msgs.hh>

#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

GZ_ADD_PLUGIN(kinematic_control::KinematicControl, gz::sim::System,
              kinematic_control::KinematicControl::ISystemConfigure,
              kinematic_control::KinematicControl::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(kinematic_control::KinematicControl,
                    "hippo_gz_plugins::kinematic_control")

using namespace kinematic_control;

//////////////////////////////////////////////////
KinematicControl::KinematicControl()
    : dataPtr(std::make_unique<KinematicControlPrivate>()) {}

//////////////////////////////////////////////////
void KinematicControl::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) {
  this->dataPtr->model = gz::sim::Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "KinematicControl plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto linkName = _sdf->Get<std::string>("link_name");
  if (linkName.empty()) {
    ignerr << "KinematicControl found an empty link_name parameter. "
           << "Failed to initialize.";
    return;
  }

  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
  if (this->dataPtr->linkEntity == gz::sim::kNullEntity) {
    ignerr << "Link with name[" << linkName << "] not found. "
           << "The KinematicControl may not control this joint.\n";
    return;
  }

  this->dataPtr->link = gz::sim::Link(this->dataPtr->linkEntity);
  this->dataPtr->link.EnableVelocityChecks(_ecm, true);
  std::string topic;
  // Subscribe to commands

  topic = gz::transport::TopicUtils::AsValidTopic(
      "/" + this->dataPtr->model.Name(_ecm) + "/vel_cmds");
  if (topic.empty()) {
    ignerr << "Failed to create topic vel_cmds for link [" << linkName << "]"
           << std::endl;
    return;
  }

  this->dataPtr->node.Subscribe(topic, &KinematicControlPrivate::OnVelCmd,
                                this->dataPtr.get());

  ignmsg << "KinematicControl subscribing to Twist messages on [" << topic
         << "]" << std::endl;

  this->dataPtr->offsetsLinearVelCmd = gz::math::Vector3d(0.0, 0.0, 0.0);
  this->dataPtr->offsetsAngularVelCmd = gz::math::Vector3d(0.0, 0.0, 0.0);
  this->dataPtr->first_update = false;
  this->dataPtr->smoothing_fac = 0.1;
}

//////////////////////////////////////////////////
void KinematicControl::PreUpdate(const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm) {
  // IGN_PROFILE("KinematicControl::PreUpdate");

  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->linkEntity == gz::sim::kNullEntity) {
    ignwarn << "Link could be identified yet, skip update step" << std::endl;
    return;
  }

  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn
        << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) return;

  if (this->dataPtr->first_update) {
    auto linear_vel_comp = _ecm.Component<gz::sim::components::LinearVelocity>(
        this->dataPtr->linkEntity);
    if (linear_vel_comp == nullptr) {
      _ecm.CreateComponent(this->dataPtr->linkEntity,
                           gz::sim::components::LinearVelocity());
    }

    auto angular_vel_comp =
        _ecm.Component<gz::sim::components::AngularVelocity>(
            this->dataPtr->linkEntity);
    if (angular_vel_comp == nullptr) {
      _ecm.CreateComponent(this->dataPtr->linkEntity,
                           gz::sim::components::AngularVelocity());
    }
    auto pose = this->dataPtr->link.WorldPose(_ecm);

    auto world_linear_velocity = this->dataPtr->link.WorldLinearVelocity(_ecm);
    auto world_angular_velocity =
        this->dataPtr->link.WorldAngularVelocity(_ecm);
    gz::math::Vector3d linear_velocity =
        pose->Rot().Inverse().RotateVector(world_linear_velocity.value());
    gz::math::Vector3d angular_velocity =
        pose->Rot().Inverse().RotateVector(world_angular_velocity.value());
    this->dataPtr->offsetsLinearVelCmd =
        this->dataPtr->smoothing_fac *
            (this->dataPtr->lastLinearVelCmd - linear_velocity) +
        (1 - this->dataPtr->smoothing_fac) * this->dataPtr->offsetsLinearVelCmd;
    this->dataPtr->offsetsAngularVelCmd =
        this->dataPtr->smoothing_fac *
            (this->dataPtr->lastAngularVelCmd - angular_velocity) +
        (1 - this->dataPtr->smoothing_fac) *
            this->dataPtr->offsetsAngularVelCmd;
  }

  // Set linear and angular velocity in local link (!) frame
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->linearVelCmdMutex);
    this->dataPtr->link.SetLinearVelocity(_ecm, this->dataPtr->linearVelCmd);
  }
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->angularVelCmdMutex);
    this->dataPtr->link.SetAngularVelocity(_ecm, this->dataPtr->angularVelCmd);
  }

  this->dataPtr->lastLinearVelCmd = this->dataPtr->linearVelCmd;
  this->dataPtr->lastAngularVelCmd = this->dataPtr->angularVelCmd;

  if (!this->dataPtr->first_update) {
    this->dataPtr->first_update = true;
  }
}

//////////////////////////////////////////////////

void KinematicControlPrivate::OnVelCmd(const gz::msgs::Twist &_msg) {
  {
    std::lock_guard<std::mutex> lock(this->linearVelCmdMutex);
    this->linearVelCmd = gz::msgs::Convert(_msg.linear());
    this->linearVelCmd += this->offsetsLinearVelCmd;
  }
  {
    std::lock_guard<std::mutex> lock(this->angularVelCmdMutex);
    this->angularVelCmd = gz::msgs::Convert(_msg.angular());
    this->angularVelCmd += this->offsetsAngularVelCmd;
  }
}
