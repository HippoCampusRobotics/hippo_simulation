#pragma once
#include <gz/sim/System.hh>

namespace pose {
class PosePluginPrivate;
class PosePlugin : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate {
 public:
  PosePlugin();
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PosePluginPrivate> private_;
};
}  // namespace pose
