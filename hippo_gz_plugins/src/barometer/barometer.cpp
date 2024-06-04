#include "barometer.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>

GZ_ADD_PLUGIN(barometer::Plugin, gz::sim::System,
              barometer::Plugin::ISystemConfigure,
              barometer::Plugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(barometer::Plugin, "hippo_gz_plugins::barometer")

namespace barometer {
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
  private_->Advertise();
}
void Plugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  auto dt = _info.simTime - private_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < private_->update_period_)) {
    return;
  }

  private_->last_pub_time_ = _info.simTime;
  private_->Publish(_ecm, gz::sim::convert<gz::msgs::Time>(_info.simTime));
}
}  // namespace barometer
