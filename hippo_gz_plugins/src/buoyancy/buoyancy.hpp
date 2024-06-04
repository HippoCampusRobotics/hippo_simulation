#pragma once

#include <gz/sim/System.hh>

#include "buoyancy_private.hpp"

namespace buoyancy {
class Plugin : public gz::sim::System,
               public gz::sim::ISystemConfigure,
               public gz::sim::ISystemUpdate {
 public:
  Plugin();
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace buoyancy
