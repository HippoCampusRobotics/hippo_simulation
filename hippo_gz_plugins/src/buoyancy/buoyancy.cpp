#include "buoyancy.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>

GZ_ADD_PLUGIN(buoyancy::Plugin, gz::sim::System,
              buoyancy::Plugin::ISystemConfigure,
              buoyancy::Plugin::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(buoyancy::Plugin, "hippo_gz_plugins::buoyancy")

namespace buoyancy {
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
}
void Plugin::Update(const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  private_->ApplyBuoyancy(_ecm);
}
}  // namespace buoyancy
