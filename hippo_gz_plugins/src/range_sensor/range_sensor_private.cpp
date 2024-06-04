#include "range_sensor_private.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "common.hpp"

namespace range_sensor {
void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  auto ptr = std::const_pointer_cast<sdf::Element>(_sdf);
  if (!ptr->HasElement("model") || !ptr->GetElement("model")) {
    ignerr << "No model element specified." << std::endl;
    return;
  }
  for (auto model_element = ptr->GetElement("model"); model_element;
       model_element = model_element->GetNextElement("model")) {
    auto param = model_element->GetAttribute("name");
    if (!param) {
      ignerr << "Model is missing required attribute 'name'" << std::endl;
      continue;
    }
    std::string name{"no-name"};
    std::string link{"no-name"};
    int id = -1;
    param->Get(name);
    if (!AssignSdfParam(model_element, "link", link)) {
      ignerr << "No link specified. Ignoring model [" << name << "]"
             << std::endl;
      continue;
    }
    if (!AssignSdfParam(model_element, "id", id)) {
      ignerr << "No id specified. Ignoring model [" << name << "]" << std::endl;
      continue;
    }
    TargetModel target_model;
    target_model.name = name;
    target_model.link = link;
    target_model.id = id;
    sdf_params_.target_models.push_back(target_model);
  }

  AssignSdfParam(_sdf, "update_rate", sdf_params_.update_rate);
  if (sdf_params_.update_rate > 0.0) {
    std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
    update_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }
  AssignSdfParam(_sdf, "range_noise_stddev", sdf_params_.range_noise_stddev);
  AssignSdfParam(_sdf, "fov_angle", sdf_params_.fov_angle);
  AssignSdfParam(_sdf, "max_viewing_angle", sdf_params_.max_viewing_angle);
  AssignSdfParam(_sdf, "drop_probability", sdf_params_.drop_probability);
  AssignSdfParam(_sdf, "max_detection_distance",
                 sdf_params_.max_detection_distance);
  AssignSdfParam(_sdf, "drop_probability_exp",
                 sdf_params_.drop_probability_exp);
  AssignSdfParam(_sdf, "translation", sdf_params_.translation);
  AssignSdfParam(_sdf, "rotation", sdf_params_.rotation);
}

bool PluginPrivate::InitModel(gz::sim::EntityComponentManager &_ecm,
                              gz::sim::Entity _entity) {
  model_ = gz::sim::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  return true;
}

std::optional<double> PluginPrivate::GetRange(
    TargetModel &_target, const gz::sim::EntityComponentManager &_ecm) {
  if (!_target.active) {
    return std::nullopt;
  }
  auto pose = GetPose(_ecm);
  auto target_pose = GetTargetPose(_ecm, _target);
  if (!pose) {
    ignerr << "Cannot get own model position" << std::endl;
    return std::nullopt;
  }
  if (!target_pose) {
    ignwarn << "Cannot get target model postion. Deactivating it." << std::endl;
    _target.active = false;
    return std::nullopt;
  }
  gz::math::Vector3<double> diff = (*pose).Pos() - (*target_pose).Pos();
  return diff.Length();
}

std::optional<gz::math::Pose3d> PluginPrivate::GetPose(
    const gz::sim::EntityComponentManager &_ecm) {
  auto pose = link_.WorldPose(_ecm);
  if (pose) {
    pose->Pos() += pose->Rot().RotateVector(sdf_params_.translation);
    gz::math::Quaterniond q{sdf_params_.rotation};
    pose->Rot() = q * pose->Rot();
  }
  return link_.WorldPose(_ecm);
}

std::optional<gz::math::Pose3d> PluginPrivate::GetTargetPose(
    const gz::sim::EntityComponentManager &_ecm, const TargetModel &_target) {
  gz::sim::Entity model = _ecm.EntityByComponents(
      gz::sim::components::Name(_target.name), gz::sim::components::Model());
  if (model == gz::sim::kNullEntity) {
    return std::nullopt;
  }
  gz::math::Pose3d pose = gz::sim::worldPose(model, _ecm);
  return pose;
}

bool PluginPrivate::DropMeasurement(const gz::sim::EntityComponentManager &_ecm,
                                    const TargetModel &_target) {
  auto target_pose = GetTargetPose(_ecm, _target);
  auto pose = GetPose(_ecm);

  if (!(target_pose && pose)) {
    return true;
  }
  gz::math::Vector3d target_vec =
      target_pose.value().Pos() - pose.value().Pos();

  gz::math::Vector3d target_normal_vec =
      target_pose.value().Rot().RotateVector(-1.0 * gz::math::Vector3d::UnitZ);

  gz::math::Vector3d normal_vec =
      pose.value().Rot().RotateVector(gz::math::Vector3d::UnitX);

  double fov_angle = acos(target_vec.Dot(normal_vec) /
                          (target_vec.Length() * normal_vec.Length()));
  double viewing_angle =
      acos(target_normal_vec.Dot(normal_vec) /
           (target_normal_vec.Length() * normal_vec.Length()));
  bool is_visible = (fov_angle < sdf_params_.fov_angle / 2.0) &&
                    (viewing_angle < sdf_params_.max_viewing_angle);
  if (!is_visible) {
    return true;
  }
  double p = uniform_distribution_(random_generator_);
  double p_dist = uniform_distribution_(random_generator_);
  double drop_prob_dist = DistanceDropProbability(target_vec.Length());

  bool is_dropped = p < sdf_params_.drop_probability || p_dist < drop_prob_dist;
  return is_dropped;
}

double PluginPrivate::DistanceDropProbability(double _distance) {
  double p = pow(_distance / sdf_params_.max_detection_distance,
                 sdf_params_.drop_probability_exp);
  return std::clamp(p, 0.0, 1.0);
}

void PluginPrivate::UpdateTargetComponents(
    gz::sim::EntityComponentManager &_ecm) {
  for (auto &target : sdf_params_.target_models) {
    gz::sim::Entity model = _ecm.EntityByComponents(
        gz::sim::components::Name(target.name), gz::sim::components::Model());
    if (model == gz::sim::kNullEntity) {
      target.active = false;
      return;
    }
    target.active = true;
  }
}

void PluginPrivate::InitComponents(gz::sim::EntityComponentManager &_ecm) {
  link_ = gz::sim::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!link_.Valid(_ecm)) {
    ignerr << "Link not available: " << sdf_params_.link << std::endl;
    return;
  }
  if (!_ecm.Component<gz::sim::components::WorldPose>(link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(), gz::sim::components::WorldPose());
  }
}

void PluginPrivate::PublishRanges(
    const gz::sim::EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_sim_time) {
  auto dt = _sim_time - last_pub_time_;
  if (dt > std::chrono::steady_clock::duration::zero() && dt < update_period_) {
    return;
  }
  last_pub_time_ = _sim_time;
  gz::hippo_gz_plugins::RangeMeasurementArray range_array;
  for (auto &target : sdf_params_.target_models) {
    auto range = GetRange(target, _ecm);
    if (!range) {
      continue;
    }
    if (DropMeasurement(_ecm, target)) {
      continue;
    }
    gz::hippo_gz_plugins::RangeMeasurement *meas =
        range_array.add_measurements();
    meas->set_id(target.id);
    meas->set_range(range.value());
  }
  ranges_publisher_.Publish(range_array);
}
}  // namespace range_sensor
