#pragma once

#include <gz/math.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace hydrodynamics {
class HydrodynamicsPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemUpdate {
 public:
  HydrodynamicsPlugin();
  ~HydrodynamicsPlugin() override;
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

 private:
  gz::math::Vector3d added_mass_linear_{0.0, 0.0, 0.0};
  gz::math::Vector3d added_mass_angular_{0.0, 0.0, 0.0};
  gz::math::Matrix3d damping_linear_;
  gz::math::Matrix3d damping_angular_;

  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};

  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                const gz::sim::EntityComponentManager &_ecm);
  void ParseHydrodynamics(const sdf::ElementPtr _element,
                          const gz::sim::EntityComponentManager &_ecm);
  void UpdateForcesAndMoments(gz::sim::EntityComponentManager &_ecm);

  void CreateComponents(const gz::sim::Entity &_entity,
                        gz::sim::EntityComponentManager &_ecm);
};
}  // namespace hydrodynamics
