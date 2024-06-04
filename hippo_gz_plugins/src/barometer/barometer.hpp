#pragma once

#include <gz/sim/System.hh>

#include "barometer_private.hpp"

namespace barometer {
class Plugin : public gz::sim::System,
               public gz::sim::ISystemConfigure,
               public gz::sim::ISystemPostUpdate {
 public:
  Plugin();
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace barometer
