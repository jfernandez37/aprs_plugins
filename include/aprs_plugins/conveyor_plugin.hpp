#ifndef APRS_PLUGINS__CONVEYOR_PLUGIN_HPP_
#define APRS_PLUGINS__CONVEYOR_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
 
namespace aprs_plugins
{
  class ConveyorPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: ConveyorPlugin();
 
    public: ~ConveyorPlugin() override;

    void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

    private: 
      gz::sim::Model _model;
      
      gz::sim::Joint _belt_joint;
      
      double _belt_velocity = 0.1;
      double _max_velocity = 0.1;
      double _power = 0.1;
      double _conveyor_limit = 0.2;
      double _belt_position = 0.0;

      std::vector<double>_reset_positions{0};
  };
}

#endif

