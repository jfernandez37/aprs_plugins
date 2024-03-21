#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <aprs_plugins/conveyor_plugin.hpp>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    aprs_plugins::ConveyorPlugin,
    gz::sim::System,
    aprs_plugins::ConveyorPlugin::ISystemPreUpdate,
    aprs_plugins::ConveyorPlugin::ISystemConfigure)

using namespace aprs_plugins;
 
ConveyorPlugin::ConveyorPlugin()
{
}
 
ConveyorPlugin::~ConveyorPlugin()
{
}

void ConveyorPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &)
{
  _model = gz::sim::Model(_entity);
  _belt_joint = gz::sim::Joint(_model.JointByName(_ecm, "belt_joint"));
  
  gzmsg << _belt_joint.Name(_ecm).value() << std::endl;
}

void ConveyorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm)
{ 
  if(_belt_position >= _conveyor_limit){
    _belt_joint.ResetPosition(_ecm, _reset_positions);
  }

  gz::sim::components::JointVelocityCmd* jvc_comp = nullptr;

  auto _joint = _model.JointByName(_ecm, "belt_joint");

  jvc_comp = _ecm.Component<gz::sim::components::JointVelocityCmd>(_joint);

  if (jvc_comp == nullptr) {

    jvc_comp = _ecm.CreateComponent(_joint, gz::sim::components::JointVelocityCmd({0.0}));

  } else {
    jvc_comp->Data() = {_belt_velocity};
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(_joint, jvc_comp->Data());
  }

  gz::sim::components::JointPosition* jp_comp = nullptr;
  jp_comp = _ecm.Component<gz::sim::components::JointPosition>(_joint);

  if (jp_comp == nullptr) {
    jp_comp = _ecm.CreateComponent(_joint, gz::sim::components::JointPosition());
  } else {
    _belt_position = jp_comp->Data()[0];
  }
}




