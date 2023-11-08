#ifndef BEHAVIORTREE_H
#define BEHAVIORTREE_H


#include <iostream>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"


class FindPlant : public BT::SyncActionNode {
public:
	FindPlant(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {

		return BT::NodeStatus::SUCCESS;
	}
};


class FindWay : public BT::SyncActionNode {
public:
	FindWay(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {

		return BT::NodeStatus::SUCCESS;
	}
};


class DriveToPlant : public BT::SyncActionNode {
public:
	DriveToPlant(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {

		return BT::NodeStatus::SUCCESS;
	}
};


class Watering : public BT::SyncActionNode {
public:
	Watering(const std::string& name) : BT::SyncActionNode(name, {})
	{}
	BT::NodeStatus tick() override {
		std::cout << "Watering: "  << std::endl;
		return BT::NodeStatus::SUCCESS;
	}
};


#endif // BEHAVIORTREE_H