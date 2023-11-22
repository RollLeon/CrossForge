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
    FindPlant(const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        auto entity = world->entity(entity_id);
        auto pa = entity.get_mut<CForge::PathComponent>();

        if (pa->path.empty()) {
            std::vector<std::tuple<Eigen::Vector3f, float>> obstacles;
            world->filter<CForge::PositionComponent, CForge::PlantComponent>()
                    .each([&obstacles](const CForge::PositionComponent &t, CForge::PlantComponent &p) {
                        obstacles.emplace_back(t.translation(), p.waterLevel);
                    });
            std::sort(obstacles.begin(), obstacles.end(),
                      [](auto v1, auto v2) {
                          return std::get<1>(v1) < std::get<1>(v2);
                      });
            if (!obstacles.empty()) {
                entity.add<CForge::PathRequestComponent>();
                auto pathComponent = entity.get_mut<CForge::PathRequestComponent>();
                pathComponent->destination = std::get<0>(obstacles.front());
            }
        }
        std::cout << "FindPlant" << std::endl;
        return BT::NodeStatus::SUCCESS;
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
        std::cout << "Start drive to plant " << std::endl;
        return calculateState();
    }

    BT::NodeStatus onRunning() override {
        return calculateState();
    }

    void onHalted() override {}

    BT::NodeStatus calculateState() {
        auto entity = world->entity(entity_id);
        if (entity.has<CForge::PathComponent>() && entity.get<CForge::PathComponent>()->path.empty()) {
            std::cout << "Successful drive to plant " << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout << "DriveToPlant" << std::endl;
        return BT::NodeStatus::RUNNING;
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
        std::cout << "Watering start: " << std::endl;
        auto entity = world->entity(entity_id);
        auto position = entity.get_mut<CForge::PositionComponent>();
        bool wateredPlant = false;
        float dt = world->delta_time();
        world->filter<CForge::PositionComponent, CForge::PlantComponent>("WateringQuery")
                .iter([position, &wateredPlant, dt](flecs::iter it, CForge::PositionComponent *p,
                                                    CForge::PlantComponent *pl) {

                    for (int i: it) {
                        if ((position->translation() - p[i].translation()).norm() < 6 &&
                            pl[i].waterLevel <= pl[i].maxWaterLevel * 0.95f) {
                            //increaseWaterLevel
                            if (p.waterLevel + waterIncreaseRate * dt < p.maxWaterLevel) {
                                p.waterLevel += waterIncreaseRate * dt;
                                std::cout << "increased by" << (waterIncreaseRate * dt) << std::endl;
                            }
                            else {
                                p.waterLevel = p.maxWaterLevel;
                            }
                            wateredPlant = true;
                            std::cout << "Watering: " << pl[i].waterLevel << std::endl;
                        }
                    }
                });
        std::cout << "returning: " << wateredPlant << std::endl;
        return wateredPlant ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }
};


#endif // ROBOTACTIONNODES_H