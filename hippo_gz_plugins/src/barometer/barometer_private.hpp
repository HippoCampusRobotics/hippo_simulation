#pragma once
#include <gz/msgs/fluid_pressure.pb.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace barometer {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(gz::sim::EntityComponentManager &_ecm,
                 gz::sim::Entity _entity);
  void Publish(const gz::sim::EntityComponentManager &_ecm,
               const gz::msgs::Time &stamp);
  void Advertise();

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    std::string base_topic{"pressure"};
    double atmospheric_pressure{101325.0};
    double water_surface_offset{0.0};
    gz::math::Vector3d position{-1.0, 0.0, 0.0};
  } sdf_params_;

  void InitHeader();
  void InitComponents(gz::sim::EntityComponentManager &_ecm);
  std::string TopicName();

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  gz::msgs::FluidPressure msg_;
};
}  // namespace barometer
