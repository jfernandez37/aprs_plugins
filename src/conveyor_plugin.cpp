#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <aprs_plugins/conveyor_plugin.hpp>
#include <time.h>

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
  _belt_joint.EnablePositionCheck(_ecm, true);
  gzmsg << _belt_joint.Name(_ecm).value() << std::endl;
 // _ros_node = std::make_shared<rclcpp::Node>("conveyor_plugin");
  
  std::vector<std::string> arguments = {"--ros-args","-p","use_sim_time:=true"};
  auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());
  std::string ns = "/";
  
  if (sdfPtr->HasElement("ros")) {
    sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

    // Set namespace if tag is present
    if (sdfRos->HasElement("namespace")) {
      ns = sdfRos->GetElement("namespace")->Get<std::string>();
      // prevent exception: namespace must be absolute, it must lead with a '/'
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }
    }

    // Get list of remapping rules from SDF
    if (sdfRos->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("remapping");
      }
    }
  }
  
  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  // Create a default context, if not already
  if(!rclcpp::ok()) {
    rclcpp::init(
      static_cast<int>(argv.size()), argv.data(), rclcpp::InitOptions(),
      rclcpp::SignalHandlerOptions::None);

  }

  std::string node_name = "conveyor_node";

  _ros_node = rclcpp::Node::make_shared(node_name, ns);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(_ros_node);
  stop_ = false;
  auto spin = [this]()
    {
      while (rclcpp::ok() && !stop_) {
        executor_->spin_once();
      }
    };
  thread_executor_spin_ = std::thread(spin);
}

void ConveyorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm)
{ 
  // gzmsg << std::to_string(_ros_node->get_clock()->now().nanoseconds()) << std::endl;


  _belt_joint.SetVelocity(_ecm, {_direction * _belt_velocity});

  double position = 0.0;
  std::optional<std::vector<double>> position_vector = _belt_joint.Position(_ecm);
  if(position_vector.has_value()){
    if(position_vector.value().size() > 0){
      _belt_position = position_vector.value()[0];
    }else{
      _belt_position = 0.0;
    }
  }
  if(abs(_belt_position) >= _conveyor_limit){
    _belt_joint.ResetPosition(_ecm, _reset_positions);
    _direction = _direction * -1;
  }
}




