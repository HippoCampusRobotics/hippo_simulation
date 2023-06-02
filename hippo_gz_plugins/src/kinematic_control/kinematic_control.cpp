#include "kinematic_control.hpp"

#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

IGNITION_ADD_PLUGIN(kinematic_control::KinematicControl,
                    ignition::gazebo::System,
                    kinematic_control::KinematicControl::ISystemConfigure,
                    kinematic_control::KinematicControl::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(kinematic_control::KinematicControl, "hippo_gz_plugins::kinematic_control")

using namespace kinematic_control;

//////////////////////////////////////////////////
KinematicControl::KinematicControl()
        : dataPtr(std::make_unique<KinematicControlPrivate>())
{
}

//////////////////////////////////////////////////
void KinematicControl::Configure(const ignition::gazebo::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             ignition::gazebo::EntityComponentManager &_ecm,
                             ignition::gazebo::EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = ignition::gazebo::Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "KinematicControl plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }

    // Get params from SDF
    auto linkName = _sdf->Get<std::string>("link_name");
    if (linkName.empty())
    {
        ignerr << "KinematicControl found an empty link_name parameter. "
               << "Failed to initialize.";
        return;
    }

    this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm,
                                                                  linkName);
    if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
    {
        ignerr << "Link with name[" << linkName << "] not found. "
               << "The KinematicControl may not control this joint.\n";
        return;
    }

    this->dataPtr->link = ignition::gazebo::Link(this->dataPtr->linkEntity);
    this->dataPtr->link.EnableVelocityChecks(_ecm, true);
    std::string topic;
    // Subscribe to commands

    topic = ignition::transport::TopicUtils::AsValidTopic("/" +
                                                          this->dataPtr->model.Name(_ecm) + "/vel_cmds");
    if (topic.empty())
    {
        ignerr << "Failed to create topic vel_cmds for link [" << linkName
               << "]" << std::endl;
        return;
    }

    this->dataPtr->node.Subscribe(topic, &KinematicControlPrivate::OnVelCmd,
                                  this->dataPtr.get());

    ignmsg << "KinematicControl subscribing to Twist messages on [" << topic
           << "]" << std::endl;

    /*
    topic = ignition::transport::TopicUtils::AsValidTopic("/" +
                                                          this->dataPtr->model.Name(_ecm) + "/linear_vel_cmds");
    if (topic.empty())
    {
        ignerr << "Failed to create topic_linear for link [" << linkName
               << "]" << std::endl;
        return;
    }

    this->dataPtr->node.Subscribe(topic, &KinematicControlPrivate::OnLinearVelCmd,
                                      this->dataPtr.get());

    ignmsg << "KinematicControl subscribing to Vector3d messages on [" << topic
           << "]" << std::endl;

    topic = ignition::transport::TopicUtils::AsValidTopic("/" +
                                                          this->dataPtr->model.Name(_ecm) + "/angular_vel_cmds");
    if (topic.empty())
    {
        ignerr << "Failed to create topic_linear for link [" << linkName
               << "]" << std::endl;
        return;
    }

    this->dataPtr->node.Subscribe(topic, &KinematicControlPrivate::OnAngularVelCmd,
                                      this->dataPtr.get());


    ignmsg << "KinematicControl subscribing to Vector3d messages on [" << topic
           << "]" << std::endl;
    */

 }

//////////////////////////////////////////////////
void KinematicControl::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
    //IGN_PROFILE("KinematicControl::PreUpdate");


    // If the joint hasn't been identified yet, the plugin is disabled
    if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity) {
        ignwarn << "Link could be identified yet, skip update step" << std::endl;
        return;
    }

    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
                << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                << "s]. System may not work properly." << std::endl;
    }

    // Nothing left to do if paused.
    if (_info.paused)
        return;


    /*
    // update linear velocity of model
    auto modelLinearVel =
            _ecm.Component<ignition::gazebo::components::LinearVelocityCmd>(
                    this->dataPtr->model.Entity());

    if (modelLinearVel == nullptr)
    {
        _ecm.CreateComponent(
                this->dataPtr->model.Entity(),
                ignition::gazebo::components::LinearVelocityCmd({this->dataPtr->linearVelCmd}));
    }
    else
    {
        *modelLinearVel =
                ignition::gazebo::components::LinearVelocityCmd({this->dataPtr->linearVelCmd});
    }

    auto modelAngularVel =
            _ecm.Component<ignition::gazebo::components::AngularVelocityCmd>(
                    this->dataPtr->model.Entity());

    if (modelAngularVel == nullptr)
    {
        _ecm.CreateComponent(
                this->dataPtr->model.Entity(),
                ignition::gazebo::components::AngularVelocityCmd({this->dataPtr->angularVelCmd}));
    }
    else
    {
        *modelAngularVel =
                ignition::gazebo::components::AngularVelocityCmd({this->dataPtr->angularVelCmd});
    }

     */




    /*
    auto linearVelComp = _ecm.Component<ignition::gazebo::components::LinearVelocityCmd>(this->dataPtr->linkEntity);

    {
        std::lock_guard<std::mutex> lock(this->dataPtr->linearVelCmdMutex);
        if (linearVelComp == nullptr) {
            _ecm.CreateComponent(this->dataPtr->linkEntity,
                                 ignition::gazebo::components::LinearVelocityCmd({this->dataPtr->linearVelCmd}));
        } else {
            *linearVelComp = ignition::gazebo::components::LinearVelocityCmd(this->dataPtr->linearVelCmd);
        }
    }
    */


    /*
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->angularVelCmdMutex);
        auto angularVelComp = _ecm.Component<ignition::gazebo::components::AngularVelocityCmd>(
                this->dataPtr->linkEntity);

        if (angularVelComp == nullptr) {
            _ecm.CreateComponent(this->dataPtr->linkEntity,
                                 ignition::gazebo::components::AngularVelocityCmd({this->dataPtr->angularVelCmd}));
        } else {
            *angularVelComp = ignition::gazebo::components::AngularVelocityCmd(this->dataPtr->angularVelCmd);
        }
    }
     */

    {
        std::lock_guard<std::mutex> lock(this->dataPtr->linearVelCmdMutex);
        this->dataPtr->link.SetLinearVelocity(_ecm, this->dataPtr->linearVelCmd);
    }
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->angularVelCmdMutex);
        this->dataPtr->link.SetAngularVelocity(_ecm, this->dataPtr->angularVelCmd);
    }
    /*

    auto velcomp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(this->dataPtr->linkEntity);
    if (velcomp == nullptr) {
        _ecm.CreateComponent(this->dataPtr->linkEntity, ignition::gazebo::components::LinearVelocity());
    }

    auto pose = this->dataPtr->link.WorldPose(_ecm);

    auto world_linear_velocity = this->dataPtr->link.WorldLinearVelocity(_ecm);
    auto world_angular_velocity = this->dataPtr->link.WorldAngularVelocity(_ecm);
    ignition::math::Vector3d linear_velocity = pose->Rot().Inverse().RotateVector(world_linear_velocity.value());
    ignition::math::Vector3d angular_velocity = pose->Rot().Inverse().RotateVector(world_angular_velocity.value());

    ignwarn << "Linear velocity command: " << this->dataPtr->linearVelCmd.X() << ", " <<
                this->dataPtr->linearVelCmd.Y() << ", "<< this->dataPtr->linearVelCmd.Z() << std::endl;
    ignwarn << "Linear velocity: " << linear_velocity.X() << ", " <<
                                   linear_velocity.Y() << ", "<< linear_velocity.Z() << std::endl;

    ignwarn << "Angular velocity command: " << this->dataPtr->angularVelCmd.X() << ", " <<
                this->dataPtr->angularVelCmd.Y() << ", "<< this->dataPtr->angularVelCmd.Z() << std::endl;
    ignwarn << "Angular velocity: " << angular_velocity.X() << ", " <<
                                   angular_velocity.Y() << ", "<< angular_velocity.Z() << std::endl;

    */

}

//////////////////////////////////////////////////
/*
void KinematicControlPrivate::OnLinearVelCmd(const ignition::msgs::Vector3d &_msg) {
    std::lock_guard<std::mutex> lock(this->linearVelCmdMutex);
    this->linearVelCmd = ignition::msgs::Convert(_msg);
}

void KinematicControlPrivate::OnAngularVelCmd(const ignition::msgs::Vector3d &_msg) {
    std::lock_guard<std::mutex> lock(this->angularVelCmdMutex);
    this->angularVelCmd = ignition::msgs::Convert(_msg);
}
 */

void KinematicControlPrivate::OnVelCmd(const ignition::msgs::Twist &_msg){
    {
        std::lock_guard<std::mutex> lock(this->linearVelCmdMutex);
        this->linearVelCmd = ignition::msgs::Convert(_msg.linear());
    }
    {
        std::lock_guard<std::mutex> lock(this->angularVelCmdMutex);
        this->angularVelCmd = ignition::msgs::Convert(_msg.angular());
    }
}
