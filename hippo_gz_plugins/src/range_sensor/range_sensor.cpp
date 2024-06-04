#include "range_sensor.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>

#include "range_sensor_private.hpp"

GZ_ADD_PLUGIN(range_sensor::Plugin, gz::sim::System,
              range_sensor::Plugin::ISystemConfigure,
              range_sensor::Plugin::ISystemUpdate,
              range_sensor::Plugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(range_sensor::Plugin, "hippo_gz_plugins::range_sensor")

namespace range_sensor {
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
  private_->AdvertiseRanges();
}

void Plugin::Update([[maybe_unused]] const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) {
  private_->UpdateTargetComponents(_ecm);
  private_->InitComponents(_ecm);
}

void Plugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }
  private_->PublishRanges(_ecm, _info.simTime);
}
}  // namespace range_sensor
