#ifndef ROBOTACTIONNODES_H
#define ROBOTACTIONNODES_H


#include <iostream>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "flecs.h"
#include "Components.h"
#include "SteeringSystem.h"
#include "PathSystem.h"
#include "SteeringSystem.h"
#include "Systems.h"


static const char *const targetPlant = "targetPlant";

class EntityAwareNode {
public:
    flecs::id_t entity_id;
    flecs::world *world;

    void initialize(flecs::entity e, flecs::world *w) {
        entity_id = e.raw_id();
        world = w;
    }
};

class FindPlant : public BT::SyncActionNode, public EntityAwareNode {
public:
    FindPlant(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        auto entity = world->entity(entity_id);
        auto pa = entity.get_mut<CForge::PathComponent>();
        auto pc = entity.get_mut<CForge::PositionComponent>();

        if (pa->path.empty()) {
            std::vector<std::tuple<Eigen::Vector3f, float>> obstacles;
            world->filter<CForge::PositionComponent, CForge::PlantComponent>()
                    .each([&obstacles, pc](const CForge::PositionComponent &t, CForge::PlantComponent &p) {
                        if (abs(t.translation().y() - pc->translation().y()) < 5) {
                            obstacles.emplace_back(t.translation(), p.waterLevel);
                        }

                    });
            std::sort(obstacles.begin(), obstacles.end(),
                      [](auto v1, auto v2) {
                          return std::get<1>(v1) < std::get<1>(v2);
                      });
            if (!obstacles.empty()) {
                entity.add<CForge::PathRequestComponent>();
                auto pathComponent = entity.get_mut<CForge::PathRequestComponent>();
                pathComponent->start = pc->translation();
                pathComponent->destination = std::get<0>(obstacles.front());
                entity.get_mut<CForge::SteeringComponent>()->originalTarget = Eigen::Vector3f(
                        pathComponent->destination);
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<Eigen::Vector3f>(targetPlant)};
    }
};


class FindWay : public BT::StatefulActionNode, public EntityAwareNode {
public:
    FindWay(const std::string &name) : BT::StatefulActionNode(name, {}) {}


    BT::NodeStatus onStart() override {
        return calculateState();
    }

    BT::NodeStatus onRunning() override {
        return calculateState();
    }

    void onHalted() override {}

    BT::NodeStatus calculateState() {
        auto entity = world->entity(entity_id);
        bool hasComponent = entity.has<CForge::PathComponent>();
        return hasComponent ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
};


class DriveToPlant : public BT::StatefulActionNode, public EntityAwareNode {
public:
    DriveToPlant(const std::string &name) : BT::StatefulActionNode(name, {}) {}


    BT::NodeStatus onStart() override {
        //std::cout << "Start drive to plant " << std::endl;
        return calculateState();
    }

    BT::NodeStatus onRunning() override {
        return calculateState();
    }

    void onHalted() override {}

    BT::NodeStatus calculateState() {
        auto entity = world->entity(entity_id);
        entity.get_mut<CForge::SteeringComponent>()->mode = CForge::SteeringComponent::PathFollowing;
        if (entity.has<CForge::PathComponent>() && entity.get<CForge::PathComponent>()->path.empty()) {
            //std::cout << "Successful drive to plant " << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        //std::cout << "DriveToPlant" << std::endl;
        return BT::NodeStatus::RUNNING;
    }
};

class TurnToPlant : public BT::StatefulActionNode, public EntityAwareNode {
public:
    TurnToPlant(const std::string &name, const BT::NodeConfig &config) : BT::StatefulActionNode(name, config) {}


    BT::NodeStatus onStart() override {
       // std::cout << "Start Turn to plant " << std::endl;
        return calculateState();
    }

    BT::NodeStatus onRunning() override {
        return calculateState();
    }

    void onHalted() override {}

    BT::NodeStatus calculateState() {
        auto entity = world->entity(entity_id);
        auto pc = entity.get_mut<CForge::PositionComponent>();
        pc->translationDelta(Eigen::Vector3f(0, 0, 0));
        auto sc = entity.get_mut<CForge::SteeringComponent>();

        Eigen::Vector3f toPlant = sc->originalTarget - pc->m_Translation;
        Eigen::Vector3f rotationTarget = Eigen::AngleAxisf(-3.1415 / 2, Eigen::Vector3f::UnitY()) * toPlant;
        Eigen::Vector3f robotForward = pc->rotation() * Eigen::Vector3f::UnitX();
        sc->mode = CForge::SteeringComponent::drivingMode::TurnTo;
        sc->targetRotation = rotationTarget;

        if (std::abs(rotationTarget.normalized().dot(robotForward)) > 0.9) {
           // std::cout << "Successful turned to plant " << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<Eigen::Vector3f>(targetPlant)};
    }
};

class Watering : public BT::StatefulActionNode, public EntityAwareNode {
public:
    Watering(const std::string &name) : BT::StatefulActionNode(name, {}) {}

    BT::NodeStatus onStart() override {
        return calculateState();
    }

    BT::NodeStatus onRunning() override {
        return calculateState();
    }

    void onHalted() override {}

    BT::NodeStatus calculateState() {
        //std::cout << "Watering start: " << std::endl;
        auto entity = world->entity(entity_id);
        auto position = entity.get_mut<CForge::PositionComponent>();
        auto robotRadius = entity.get<CForge::SteeringComponent>()->securityDistance +
                           entity.get<CForge::GeometryComponent>()->actor->boundingVolume().boundingSphere().radius();
        bool wateredPlant = false;
        float waterIncreaseRate = 1.0;
        float dt = world->delta_time();
        world->filter<CForge::PositionComponent, CForge::PlantComponent, CForge::GeometryComponent>("WateringQuery")
                .iter([position, &wateredPlant, dt, waterIncreaseRate, robotRadius](flecs::iter it,
                                                                                    CForge::PositionComponent *p,
                                                                                    CForge::PlantComponent *pl,
                                                                                    CForge::GeometryComponent *gc) {

                    for (int i: it) {
                        float plantRadius = gc[i].actor->boundingVolume().boundingSphere().radius() * p[i].scale().x();
                        float wateringRadius = (plantRadius + robotRadius) * 1.1f;
                        if ((position->translation() - p[i].translation()).norm() < wateringRadius &&
                            pl[i].waterLevel <= pl[i].maxWaterLevel * 0.95f) {
                            //increaseWaterLevel
                            if (pl[i].waterLevel + waterIncreaseRate * dt < pl[i].maxWaterLevel) {
                                pl[i].waterLevel += waterIncreaseRate * dt;
                                //std::cout << "increased by" << (waterIncreaseRate * dt) << std::endl;
                            } else {
                                pl[i].waterLevel = pl[i].maxWaterLevel;
                            }
                            wateredPlant = true;
                            //std::cout << "Watering: " << pl[i].waterLevel << std::endl;
                        }
                    }
                });
        //std::cout << "returning: " << wateredPlant << std::endl;
        return wateredPlant ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }
};


#endif // ROBOTACTIONNODES_H