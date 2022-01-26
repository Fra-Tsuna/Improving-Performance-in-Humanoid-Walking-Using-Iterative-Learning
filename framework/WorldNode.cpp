#include "WorldNode.hpp"
#include "dart/external/imgui/imgui.h"

WorldNode::WorldNode(const dart::simulation::WorldPtr world, const dart::dynamics::SkeletonPtr hrp4)
    : dart::gui::osg::WorldNode(world), mExternalForce(Eigen::Vector3d::Zero()), mForceDuration(0.0)
{
    assert(world);
    assert(hrp4);

    mWorld = world;
    mRobot = hrp4;

    mController.reset(new Controller(hrp4, world));
    //mController->setInitialConfiguration();
}

void WorldNode::customPreStep() {
    mController->update();
}

