#include <chrono>
#include <map>
#include <string>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace hx5_simulation
{
class ContactSystem : public gz::sim::System,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate
{
  struct Sensor
  {
    gz::sim::Entity collision;
    gz::transport::Node::Publisher publisher;
  };

  gz::transport::Node transport;
  std::map<gz::sim::Entity, Sensor> sensors;
  std::chrono::steady_clock::duration previous{};

public:
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &entities) override
  {
    entities.Each<gz::sim::components::ContactSensor, gz::sim::components::ParentEntity,
                  gz::sim::components::Name>(
      [&](const gz::sim::Entity &entity, const gz::sim::components::ContactSensor *sensor,
          const gz::sim::components::ParentEntity *parent, const gz::sim::components::Name *name)
      {
        if (name->Data().rfind("tactile_", 0) != 0 || sensors.count(entity)) return true;
        const auto collisionName = sensor->Data()->GetElement("contact")->Get<std::string>("collision");
        auto collisions = entities.ChildrenByComponents(parent->Data(),
          gz::sim::components::Collision(), gz::sim::components::Name(collisionName));
        if (collisions.size() != 1) return true;
        const auto collision = collisions.front();
        if (!entities.Component<gz::sim::components::ContactSensorData>(collision))
          entities.CreateComponent(collision, gz::sim::components::ContactSensorData());
        const auto suffix = name->Data().substr(8);
        const auto topic = "/hx5/contact/" + suffix.substr(0, 1) + "/finger" + suffix.substr(2);
        sensors.emplace(entity, Sensor{collision, transport.Advertise<gz::msgs::Contacts>(topic)});
        return true;
      });
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &entities) override
  {
    if (info.paused) return;
    if (info.simTime < previous) previous = {};
    if (info.simTime - previous < std::chrono::milliseconds(20)) return;
    previous = info.simTime;
    for (auto &[entity, sensor] : sensors)
    {
      const auto data = entities.Component<gz::sim::components::ContactSensorData>(sensor.collision);
      if (!data || !entities.HasEntity(entity)) continue;
      auto message = data->Data();
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
      const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - seconds);
      message.mutable_header()->mutable_stamp()->set_sec(seconds.count());
      message.mutable_header()->mutable_stamp()->set_nsec(nanoseconds.count());
      sensor.publisher.Publish(message);
    }
  }
};
}

GZ_ADD_PLUGIN(hx5_simulation::ContactSystem, gz::sim::System,
              gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(hx5_simulation::ContactSystem, "hx5_simulation::ContactSystem")
