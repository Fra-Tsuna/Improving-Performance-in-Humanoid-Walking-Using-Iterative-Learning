#pragma once

#include <Eigen/Core>
#include "dart/dart.hpp"
#include <fstream>
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
  void CoMPlan();
  CandidateMotion computeCMForFootstep(FootstepPlan* plan, State current, int startTime);

  Eigen::VectorXd getJointAccelerations(State desired, State current, WalkState walkState);

  Eigen::Vector3d getRPY(dart::dynamics::BodyNode *, dart::dynamics::BodyNode *);
  Eigen::Vector3d getRPY(dart::dynamics::BodyNode *);

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode *mTorso;

  dart::simulation::WorldPtr mWorld;

  dart::dynamics::BodyNode *mLeftFoot;
  dart::dynamics::BodyNode *mRightFoot;
  dart::dynamics::BodyNode *mSupportFoot;
  dart::dynamics::BodyNode *mSwingFoot;
  dart::dynamics::BodyNode *mBase;

  State desired;
  State current;
  State initial;
  WalkState walkState;

  FootstepPlan *footstepPlan;

  std::vector<Logger *> logList;

  CandidateMotion cm;
  std::vector<Eigen::Vector3d> VRPTrajectory;
  std::vector<Eigen::Vector3d> DCMTrajectory;
  Eigen::Vector3d VRPCommanded, DCMCommanded, DCMCommandedDot, futureVRP;
  Eigen::Vector3d kl = Eigen::Vector3d(kl_x,kl_y,kl_z);
  double alfa;
  Eigen::Matrix3d RDelta;
  std::vector<Eigen::Vector3d> VRPWindow;
  int learningIterations, learningIter, ILCIter, learningIterSamples, k, stepIter, endOfLearning, nextIter = 0;
  double globalTime, ILCTime, learningIterDuration, time;
  void logVars(){
      std::cout << "tg: " << walkState.iter << ":" << globalTime << 
        " i: " << learningIter << 
        " Titer: " << learningIterSamples << ":" << learningIterDuration <<
        " t: " << stepIter << ":" << time << 
        " k: " << k << 
        " t_in_w: " << ILCIter << ":" << ILCTime << 
        " alfa: " << alfa <<
        " RDelta: " << RDelta.row(0) << " " << RDelta.row(1) << " " << RDelta.row(2) <<
        // " VRPWindow last: " << learningIterSamples/stepPerILC-1 << 
        // " VRPWindow size: " << VRPWindow.size() <<
        // " Total iterations: " << learningIterations <<
        // " VRPl: " << desired.vrpPos.x() << ", " << desired.vrpPos.y() << ", " << desired.vrpPos.z() <<
        // " DCMl: " << desired.dcmPos.x() << ", " << desired.dcmPos.y() << ", " << desired.dcmPos.z() <<
        // " VRPl1+1: " << futureVRP.x() << ", " << futureVRP.y() << ", " << futureVRP.z() <<
        std::endl;
  }

  Eigen::VectorXd initialConfiguration;

public:
};
