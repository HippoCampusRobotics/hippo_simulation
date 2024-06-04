#include "buoyancy_private.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>

namespace buoyancy {

void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  for (sdf::ElementPtr element = _sdf->GetFirstElement(); element != nullptr;
       element = element->GetNextElement()) {
    if (element->GetName() != "buoyancy") {
      continue;
    }
    ignmsg << "Found buoyancy element." << std::endl;
    if (element->HasElement("link_name")) {
      sdf_params_.link =
          element->Get<std::string>("link_name", sdf_params_.link).first;
    } else {
      ignerr << "Missing field 'link' in buoyancy plugin" << std::endl;
    }
    if (element->HasElement("force_added")) {
      sdf_params_.additional_buoyancy_force =
          element
              ->Get<double>("force_added",
                            sdf_params_.additional_buoyancy_force)
              .first;
    } else {
      ignerr << "Missing field 'force_added' in buoyancy plugin" << std::endl;
    }
    if (element->HasElement("compensation")) {
      sdf_params_.relative_compensation =
          element
              ->Get<double>("compensation", sdf_params_.relative_compensation)
              .first;
    } else {
      ignerr << "Missing field 'compensation' in buoyancy plugin" << std::endl;
    }

    if (element->HasElement("height_scale_limit")) {
      sdf_params_.height_scale_limit =
          element
              ->Get<double>("height_scale_limit",
                            sdf_params_.height_scale_limit)
              .first;
    } else {
      ignerr << "Missing field 'height_scale_limit' in buoyancy plugin"
             << std::endl;
    }

    if (element->HasElement("origin")) {
      sdf_params_.origin =
          element->Get<gz::math::Vector3d>("origin", sdf_params_.origin).first;
    } else {
      ignerr << "Missing field 'origin' in buoyancy plugin" << std::endl;
    }
    break;
  }
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

void PluginPrivate::ApplyBuoyancy(gz::sim::EntityComponentManager &_ecm) {
  if (!_ecm.HasEntity(link_.Entity())) {
    return;
  }
  if (!_ecm.EntityHasComponentType(link_.Entity(),
                                   gz::sim::components::Inertial().TypeId())) {
    return;
  }
  auto inertial = _ecm.Component<gz::sim::components::Inertial>(link_.Entity());
  double mass = inertial->Data().MassMatrix().Mass();
  double buoyancy_mass = mass * sdf_params_.relative_compensation;
  gz::math::Vector3d force = sdf_params_.additional_buoyancy_force +
                             gz::math::Vector3d(0.0, 0.0, 9.81 * buoyancy_mass);

  gz::math::Pose3d pose_link = gz::sim::worldPose(link_.Entity(), _ecm);
  gz::math::Vector3d buoyancy_offset =
      pose_link.Rot().RotateVector(sdf_params_.origin);
  gz::math::Vector3d center_of_gravity = pose_link.Pos() + buoyancy_offset;

  double scale =
      std::abs((center_of_gravity.Z() - sdf_params_.height_scale_limit) /
               (2 * sdf_params_.height_scale_limit));
  if (center_of_gravity.Z() > sdf_params_.height_scale_limit) {
    scale = 0.0;
  }
  scale = gz::math::clamp(scale, 0.0, 1.0);
  force *= scale;
  gz::math::Vector3d moment = buoyancy_offset.Cross(force);
  link_.AddWorldWrench(_ecm, force, moment);
}

void PluginPrivate::InitComponents(gz::sim::EntityComponentManager &_ecm) {
  link_ = gz::sim::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!_ecm.Component<gz::sim::components::WorldPose>(link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(), gz::sim::components::WorldPose());
  }
}
}  // namespace buoyancy
