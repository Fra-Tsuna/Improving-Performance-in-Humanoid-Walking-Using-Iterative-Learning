#pragma once

#include <Eigen/Core>
#include "dart/dart.hpp"
#include  <fstream>
#include "utils.cpp"
#include "types.hpp"
#include "FootstepPlan.hpp"
#include <string>
#include "computeCandidateMotion.hpp"

class Controller
{
public:
  Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world);
  virtual ~Controller();

  Eigen::Vector3d getZmpFromExternalForces();

  void update();

  Eigen::MatrixXd getTorsoAndSwfJacobian();
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getJacobianDeriv();

  void setInitialConfiguration();
  void VRPPlan();
  void DCMPlan();

  Eigen::VectorXd getJointAccelerations(State desired, State current, WalkState walkState);

  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*, dart::dynamics::BodyNode*);
  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*);

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode* mTorso;

  dart::simulation::WorldPtr mWorld;

  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;

  State desired;
  State current;
  State initial;
  WalkState walkState;
  PlanningConstants constants;

  FootstepPlan* footstepPlan;

  std::vector<Logger*> logList;

  CandidateMotion cm;
  Eigen::Vector3d currentVRP;
  std::vector<Eigen::Vector3d> VRPTrajectory;
  Eigen::Vector3d currentDCM;
  std::vector<Eigen::Vector3d> DCMTrajectory;

  Eigen::VectorXd initialConfiguration;

public:

};
