#include "behaviortree_cpp/bt_factory.h"

// file that contains the custom nodes definitions
#include "BehaviorTree.h"

using namespace BehaviorTree;

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<FindPlant>("FindPlant");
    factory.registerNodeType<FindWay>("FindWay");
    factory.registerNodeType<DriveToPlant>("DriveToPlant");
    factory.registerNodeType<Watering>("Watering");

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("./WateringBehaviorTree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();

    return 0;
}

/* Expected output:
*
  [ Battery: OK ]
  GripperInterface::open
  ApproachObject: approach_object
  GripperInterface::close
*/