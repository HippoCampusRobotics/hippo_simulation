#include "pose.hpp"

#include <gz/msgs/pose.pb.h>

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

GZ_ADD_PLUGIN(pose::PosePlugin, gz::sim::System,
              pose::PosePlugin::ISystemConfigure,
              pose::PosePlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(pose::PosePlugin, "hippo_gz_plugins::pose")

using namespace pose;
using namespace gz;

class pose::PosePluginPrivate {
 public:
  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
    sdf_params_.link = _sdf->Get<std::string>("link", sdf_params_.link).first;
    sdf_params_.base_topic =
        _sdf->Get<std::string>("base_topic", sdf_params_.base_topic).first;
    sdf_params_.update_rate =
        _sdf->Get<double>("update_rate", sdf_params_.update_rate).first;
    if (sdf_params_.update_rate > 0.0) {
      std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
      update_period_ =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              period);
    }
  }

  bool InitModel(gz::sim::EntityComponentManager &_ecm,
                 gz::sim::Entity _entity) {
    model_ = gz::sim::Model(_entity);
    if (!model_.Valid(_ecm)) {
      return false;
    }
    SetModelName(model_.Name(_ecm));
    InitEntities(_ecm);
    InitPoseHeader();
    return true;
  }

  void PublishPose(const gz::sim::EntityComponentManager &_ecm,
                   const gz::msgs::Time &stamp) {
    auto pose = link_.WorldPose(_ecm);
    auto header = pose_msg_.mutable_header();
    header->mutable_stamp()->CopyFrom(stamp);
    gz::msgs::Set(&pose_msg_, *pose);
    pose_pub_.Publish(pose_msg_);
  }

  void Advertise() { pose_pub_ = node_.Advertise<msgs::Pose>(TopicName()); }

  const std::string TopicName() {
    return "/" + model_name_ + "/" + sdf_params_.base_topic;
  }

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    std::string base_topic{"pose"};
  } sdf_params_;

  void InitPoseHeader() {
    auto header = pose_msg_.mutable_header();
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value("map");
  }

  void SetModelName(const std::string _name) { model_name_ = _name; }

  std::string ModelName() { return model_name_; }

  std::string LinkName() { return sdf_params_.link; }

  void InitEntities(gz::sim::EntityComponentManager &_ecm) {
    link_ = gz::sim::Link(model_.LinkByName(_ecm, sdf_params_.link));
    if (!_ecm.Component<gz::sim::components::WorldPose>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), gz::sim::components::WorldPose());
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  gz::sim::Link link_{gz::sim::kNullEntity};
  transport::Node node_;
  transport::Node::Publisher pose_pub_;
  msgs::Pose pose_msg_;
};

PosePlugin::PosePlugin()
    : System(), private_(std::make_unique<PosePluginPrivate>()) {}

void PosePlugin::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           [[maybe_unused]] gz::sim::EventManager &_eventMgr) {
  private_->ParseSdf(_sdf);
  if (!private_->InitModel(_ecm, _entity)) {
    ignerr << "PosePlugin needs to be attached to model entity." << std::endl;
    return;
  }
  private_->Advertise();
}
void PosePlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  auto dt = _info.simTime - private_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < private_->update_period_)) {
    return;
  }

  private_->last_pub_time_ = _info.simTime;
  private_->PublishPose(_ecm, gz::sim::convert<msgs::Time>(_info.simTime));
}
