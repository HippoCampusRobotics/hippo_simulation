#pragma once

#include <gz/hippo_gz_plugins/range_measurement_array.pb.h>

#include <chrono>
#include <gz/math/Pose3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <random>
#include <sdformat.hh>

namespace range_sensor {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);
  bool InitModel(gz::sim::EntityComponentManager &_ecm,
                 gz::sim::Entity _entity);
  void PublishRanges(const gz::sim::EntityComponentManager &_ecm,
                     const std::chrono::steady_clock::duration &_sim_time);
  void AdvertiseRanges() {
    ranges_publisher_ =
        node_.Advertise<gz::hippo_gz_plugins::RangeMeasurementArray>(
            RangesTopicName());
  }

  void UpdateTargetComponents(gz::sim::EntityComponentManager &_ecm);
  void InitComponents(gz::sim::EntityComponentManager &_ecm);

 private:
  struct TargetModel {
    int id;
    std::string name{""};
    std::string link{"base_link"};
    bool active{true};
  };

  struct SdfParams {
    std::string link{"base_link"};
    std::string ranges_base_topic{"ranges"};
    double update_rate{10.0};
    double range_noise_stddev{0.0};
    double fov_angle{90.0};
    double max_viewing_angle{140.0};
    double drop_probability{0.05};
    double max_detection_distance{5.0};
    double drop_probability_exp{2.0};
    std::vector<TargetModel> target_models;
    gz::math::Vector3d translation;
    gz::math::Vector3d rotation;
  } sdf_params_;

  std::optional<double> GetRange(TargetModel &_target,
                                 const gz::sim::EntityComponentManager &_ecm);

  std::string TopicPrefix() { return "/" + model_name_; }

  std::string RangesTopicName() {
    return TopicPrefix() + "/" + sdf_params_.ranges_base_topic;
  }

  std::optional<gz::math::Pose3d> GetPose(
      const gz::sim::EntityComponentManager &_ecm);

  std::optional<gz::math::Pose3d> GetTargetPose(
      const gz::sim::EntityComponentManager &_ecm, const TargetModel &_target);

  bool DropMeasurement(const gz::sim::EntityComponentManager &_ecm,
                       const TargetModel &_target);

  double DistanceDropProbability(double _distance);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::string model_name_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher ranges_publisher_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;
};
}  // namespace range_sensor
