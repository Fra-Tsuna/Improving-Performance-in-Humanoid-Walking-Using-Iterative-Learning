#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/gui/gui.hpp>
#include "WorldNode.hpp"

int main(int argc, char* argv[])
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and HRP4 robot and add them to the world
  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton(realpath("../urdf/ground.urdf", NULL));
  auto hrp4 = urdfLoader.parseSkeleton(realpath("../urdf/hrp4.urdf", NULL));
  world->addSkeleton(ground);
  world->addSkeleton(hrp4);

  // set joint actuator type and force limits
  double forceLimit = 100;

  for (int i = 0; i < hrp4->getNumJoints(); i++) {
	  size_t  dim   = hrp4->getJoint(i)->getNumDofs();
	  if(dim==6) {
		  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
	  }
	  if(dim==1) {
		  //hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::SERVO);
                  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::ACCELERATION);
		  hrp4->getJoint(i)->setForceUpperLimit(0,  forceLimit);
		  hrp4->getJoint(i)->setForceLowerLimit(0, -forceLimit);
		  hrp4->getJoint(i)->setPositionLimitEnforced(true);
	  }
  }

  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1.0/100.0);

  // Wrap a WorldNode around it
  osg::ref_ptr<WorldNode> node = new WorldNode(world, hrp4);
  node->setNumStepsPerCycle(10);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Set recording
  //viewer.record("../Sim_Vx0.0_Vy0.23_Robust","frame"); 

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("HRP4 MPC");

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 0.8,  -8.2*3.28*0.2, 3.3*0.155)*1.0,  
        ::osg::Vec3d( -0.10,  2.5, 0.35),  
        ::osg::Vec3d( 0.00,  0.2, 2.1));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
