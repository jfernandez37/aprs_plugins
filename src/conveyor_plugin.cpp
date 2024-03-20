#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
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

  


  // _max_velocity = _sdf->GetElementImpl("max_velocity")->Get<double>();
  
  gzmsg << _belt_joint.Name(_ecm).value() << std::endl;
}

void ConveyorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm)
{
  
  // auto belt_position = _belt_joint.Position(_ecm);

  // if (belt_position.has_value()){
  //   _belt_position = belt_position.value()[0];
  // } else{
  //   _belt_position = 0.0;
  // }  
  
  // if(_belt_position >= _conveyor_limit){
  //   _belt_joint.ResetPosition(_ecm, _reset_positions);
  // }

  // const std::vector<double>velocities{_belt_velocity};

  gz::sim::components::JointVelocityCmd* jvc_comp = nullptr;

  auto _joint = _model.JointByName(_ecm, "belt_joint");

  jvc_comp = _ecm.Component<gz::sim::components::JointVelocityCmd>(_joint);

  if (jvc_comp == nullptr) {

    jvc_comp = _ecm.CreateComponent(_joint, gz::sim::components::JointVelocityCmd({0.0}));

    gzmsg << "NULLPTR" << std::endl;
  } else {
    jvc_comp->Data() = {0.1};
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(_joint, jvc_comp->Data());
  }

  

  // _belt_joint.ResetVelocity(_ecm, velocities);
  // if(_belt_joint.Velocity(_ecm).has_value()){
  //   gzmsg << _belt_joint.Velocity(_ecm).value()[0] << std::endl;
  // }else{
  //   gzmsg << "NO VALUE" << std::endl;
  // }
}




