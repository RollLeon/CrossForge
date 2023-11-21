#ifndef ROBOTACTIONNODES_H
#define ROBOTACTIONNODES_H


#include <iostream>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "flecs.h"
#include "PathComponent.h"
#include "PositionComponent.h"
#include "PlantComponent.h"
#include "PathRequestComponent.h"
#include "SteeringSystem.h"
#include "PathSystem.h"
#include "SteeringSystem.h"
#include "PlantSystem.h"



class EntityAwareNode {
public:
	flecs::id_t entity_id;
	flecs::world *world; 

	void initialize(flecs::entity e, flecs::world *w)
	{
		entity_id = e.raw_id();
		world = w;
	}
};

class FindPlant : public BT::SyncActionNode, public EntityAwareNode {
public:
	FindPlant(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {
        auto entity = world->entity(entity_id);
		auto pa = entity.get_mut<CForge::PathComponent>();

		if (pa->path.empty()) {
			std::vector<std::tuple<Eigen::Vector3f, float>> obstacles;
			world->filter<CForge::PositionComponent, CForge::PlantComponent>()
				.each([&obstacles](const CForge::PositionComponent& t, CForge::PlantComponent& p) {
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


class FindWay : public BT::SyncActionNode, public EntityAwareNode {
public:
	FindWay(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {
		
		auto entity = world->entity(entity_id);
		std::cout << "FindWay1" << std::endl;
		if (entity.has<CForge::PathComponent>())
		{
			return BT::NodeStatus::SUCCESS;
		}
		std::cout << "FindWay" << std::endl;
		return BT::NodeStatus::RUNNING;
	}
};


class DriveToPlant : public BT::SyncActionNode, public EntityAwareNode {
public:
	DriveToPlant(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {
		auto entity = world->entity(entity_id);
		auto path = entity.get<CForge::PathComponent>();
		if (path->path.empty())
		{
			return BT::NodeStatus::SUCCESS;
		}
		std::cout << "DriveToPlant" << std::endl;
		return BT::NodeStatus::RUNNING;
	}
};


class Watering : public BT::SyncActionNode, public EntityAwareNode {
public:
	Watering(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {
		auto entity = world->entity(entity_id);
		auto position = entity.get_mut<CForge::PositionComponent>();
		bool wateredPlant = false;
		world->query<CForge::PositionComponent, CForge::PlantComponent>("WateringQuery")
			.iter([position, &wateredPlant](flecs::iter it, CForge::PositionComponent* p, CForge::PlantComponent* pl) {
			
			for (int i : it) {
				if ((position->translation() - p[i].translation()).norm() < 2.1 && pl[i].waterLevel < pl[i].maxWaterLevel)
				{
					CForge::PlantSystem::increaseWaterLevel(pl[i]);
					wateredPlant = true;
				}
			}
			});

		std::cout << "Watering: "  << std::endl;
		return wateredPlant? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
	}
};


#endif // ROBOTACTIONNODES_H