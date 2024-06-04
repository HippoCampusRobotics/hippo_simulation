#pragma once
#include <gz/msgs/fluid_pressure.pb.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <sdf/Element.hh>

namespace buoyancy {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(gz::sim::EntityComponentManager &_ecm,
                 gz::sim::Entity _entity);

  void ApplyBuoyancy(gz::sim::EntityComponentManager &_ecm);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double additional_buoyancy_force{0.0};
    double relative_compensation{1.0};
    // center of gravity relative to the origin of the link
    gz::math::Vector3d origin{0.0, 0.0, 0.0};
    // distance over which buoyancy gets scaled down to zero at the water
    // surface.
    double height_scale_limit{0.1};
  } sdf_params_;

  void InitComponents(gz::sim::EntityComponentManager &_ecm);

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  gz::sim::Link link_{gz::sim::kNullEntity};
};
}  // namespace buoyancy
