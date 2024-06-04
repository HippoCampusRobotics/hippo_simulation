#include "thruster.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>

GZ_ADD_PLUGIN(thruster::Plugin, gz::sim::System,
              thruster::Plugin::ISystemConfigure,
              thruster::Plugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(thruster::Plugin, "hippo_gz_plugins::thruster")

namespace thruster {
Plugin::Plugin() : System(), private_(std::make_unique<PluginPrivate>()) {}

void Plugin::Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       [[maybe_unused]] gz::sim::EventManager &_eventMgr) {
  private_->ParseSdf(_sdf);
  if (!private_->InitModel(_ecm, _entity)) {
    ignerr << "Plugin needs to be attached to model entity." << std::endl;
    return;
  }
  private_->AdvertiseRpm();
  private_->AdvertiseThrust();
  private_->SubscribeThrottleCmd();
}
void Plugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }
  if (private_->throttle_cmd_updated_) {
    private_->last_command_time_ = _info.simTime;
    private_->throttle_cmd_updated_ = false;
  }
  if (_info.simTime - private_->last_command_time_ >
      std::chrono::milliseconds(500)) {
    private_->ThrottleCmdTimedOut();
  }
  // Apply forces/moments in each step
  private_->UpdateRotorVelocity(
      _ecm, std::chrono::duration<double>(_info.dt).count());
  private_->ApplyWrench(_ecm);

  // publish messages with specified update rate
  auto dt = _info.simTime - private_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < private_->update_period_)) {
    return;
  }
  private_->last_pub_time_ = _info.simTime;
  private_->PublishRpm(_ecm);
  private_->PublishThrust();
}
}  // namespace thruster
