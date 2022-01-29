#include "Controller.hpp"

Controller::Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world)
	: mRobot(_robot), mWorld(_world)
{
    setInitialConfiguration();

    // some useful pointers to robot limbs
    mLeftFoot = mRobot->getBodyNode("l_sole");
    mRightFoot = mRobot->getBodyNode("r_sole");
    mBase = mRobot->getBodyNode("base_link");
    mTorso = mRobot->getBodyNode("body");

    // Initialize walk state
    walkState.iter = 0;
    walkState.footstepCounter = 0; // useless
    walkState.supportFoot = false; // useless

    
    std::vector<Vref> vrefSequence;

    for (int i = 0; i < 5000; i++) {
        if (i < 100) vrefSequence.push_back(Vref(0.0, 0.0, 0.00));
        else vrefSequence.push_back(Vref(0.2, 0.0, 0.00));
    }

    bool firstSupportFootIsLeft = false;
    Eigen::VectorXd leftFootPose(6), rightFootPose(6);
    leftFootPose << getRPY(mLeftFoot), mLeftFoot->getCOM();
    rightFootPose << getRPY(mRightFoot), mRightFoot->getCOM();
    
    footstepPlan = new FootstepPlan;
    footstepPlan->plan(vrefSequence, leftFootPose, rightFootPose, firstSupportFootIsLeft);

    // Retrieve current state

    current.com.pos = mRobot->getCOM();
    current.com.vel = mRobot->getCOMLinearVelocity();
    current.com.acc = mRobot->getCOMLinearAcceleration();
    current.com.ang_pos = getRPY(mBase);
    current.com.ang_vel = mBase->getCOMSpatialVelocity().head(3);
    current.com.ang_acc = mBase->getCOMSpatialAcceleration().head(3);

    current.zmpPos = current.com.pos - current.com.acc / (omega*omega);
    current.dcmPos = current.com.pos + b*current.com.vel;

    current.leftFoot.pos = mLeftFoot->getCOM();
    current.leftFoot.vel = mLeftFoot->getCOMLinearVelocity();
    current.leftFoot.acc = mLeftFoot->getCOMLinearAcceleration();
    current.leftFoot.ang_pos = getRPY(mLeftFoot);
    current.leftFoot.ang_vel = mLeftFoot->getCOMSpatialVelocity().head(3);
    current.leftFoot.ang_acc = mLeftFoot->getCOMSpatialAcceleration().head(3);

    current.rightFoot.pos = mRightFoot->getCOM();
    current.rightFoot.vel = mRightFoot->getCOMLinearVelocity();
    current.rightFoot.acc = mRightFoot->getCOMLinearAcceleration();
    current.rightFoot.ang_pos = getRPY(mRightFoot);
    current.rightFoot.ang_vel = mRightFoot->getCOMSpatialVelocity().head(3);
    current.rightFoot.ang_acc = mRightFoot->getCOMSpatialAcceleration().head(3);

    // Initialize desired state with reasonable values

    desired.com.pos = Eigen::Vector3d(current.com.pos(0), current.com.pos(1), comTargetHeight);
    desired.com.vel = Eigen::Vector3d::Zero();
    desired.com.acc = Eigen::Vector3d::Zero();
    desired.com.ang_pos = Eigen::Vector3d::Zero();
    desired.com.ang_vel = Eigen::Vector3d::Zero();
    desired.com.ang_acc = Eigen::Vector3d::Zero();

    desired.zmpPos = Eigen::Vector3d(current.com.pos(0), current.com.pos(1),0.0);

    desired.leftFoot.pos = Eigen::Vector3d(current.com.pos(0), 0.08, 0.0);
    desired.leftFoot.vel = Eigen::Vector3d::Zero();
    desired.leftFoot.acc = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_pos = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_vel = Eigen::Vector3d::Zero();
    desired.leftFoot.ang_acc = Eigen::Vector3d::Zero();

    desired.rightFoot.pos = Eigen::Vector3d(current.com.pos(0), -0.08, 0.0);
    desired.rightFoot.vel = Eigen::Vector3d::Zero();
    desired.rightFoot.acc = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_pos = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_vel = Eigen::Vector3d::Zero();
    desired.rightFoot.ang_acc = Eigen::Vector3d::Zero();

    initial = desired;

    // Create file loggers

    logList.push_back(new Logger("desired.comPos", &desired.com.pos));
    logList.push_back(new Logger("desired.comVel", &desired.com.vel));
    logList.push_back(new Logger("desired.zmpPos", &desired.zmpPos));

    logList.push_back(new Logger("current.comPos", &current.com.pos));
    logList.push_back(new Logger("current.comVel", &current.com.vel));
    logList.push_back(new Logger("current.zmpPos", &current.zmpPos));

    //test
    cm = computeCandidateMotion(footstepPlan, desired, 0, 3000, 4000);
    VRPPlan();
    DCMPlan();
    desired.dcmPos = DCMTrajectory.at(0);
    desired.vrpPos = VRPTrajectory.at(0);
    desired.dcmVel = eta*(DCMTrajectory.at(0) - VRPTrajectory.at(0));
    DCMCommanded = desired.dcmPos;
    for (int i = 0; i < footstepPlan->getFootstepEndTiming(1); i+=round(timeILC/timeStep)) { // 4
        VRPWindow.push_back(VRPTrajectory.at(i));
    }
}

Controller::~Controller() {}

void Controller::update() {
    if (walkState.iter >= 3000) return;
    // std::cout << walkState.iter << std::endl;
    // This adds a push to the robot
    //if (mWorld->getSimFrames()>=510 && mWorld->getSimFrames()<=520) mBase->addExtForce(Eigen::Vector3d(0,300,0));

    //if (mWorld->getSimFrames()==500) system("gnuplot ../plotters/plot");
    walkState.simulationTime = mWorld->getSimFrames();
    int index = footstepPlan->getFootstepIndexAtTime(walkState.iter);
    learningIter = index/2; // 1
    learningIterSamples = footstepPlan->getFootstepDuration(learningIter*2) + footstepPlan->getFootstepDuration(learningIter*2+1);
    learningIterDuration = (double) learningIterSamples * timeStep;
    globalTime = (double) walkState.iter*timeStep; // 2

    VRPCommanded = desired.vrpPos + zeta * (current.dcmPos - desired.dcmPos);
    

    // Retrieve current state in the world frame

    current.com.pos = mRobot->getCOM();
    current.com.vel = mRobot->getCOMLinearVelocity();
    current.com.acc = mRobot->getCOMLinearAcceleration();
    current.com.ang_pos = getRPY(mBase);
    current.com.ang_vel = mBase->getCOMSpatialVelocity().head(3);
    current.com.ang_acc = mBase->getCOMSpatialAcceleration().head(3);

    current.zmpPos = current.com.pos - current.com.acc / (omega*omega); //getZmpFromExternalForces(); //
    current.dcmPos = current.com.pos + b*current.com.vel;

    current.leftFoot.pos = mLeftFoot->getCOM();
    current.leftFoot.vel = mLeftFoot->getCOMLinearVelocity();
    current.leftFoot.acc = mLeftFoot->getCOMLinearAcceleration();
    current.leftFoot.ang_pos = getRPY(mLeftFoot);
    current.leftFoot.ang_vel = mLeftFoot->getCOMSpatialVelocity().head(3);
    current.leftFoot.ang_acc = mLeftFoot->getCOMSpatialAcceleration().head(3);

    current.rightFoot.pos = mRightFoot->getCOM();
    current.rightFoot.vel = mRightFoot->getCOMLinearVelocity();
    current.rightFoot.acc = mRightFoot->getCOMLinearAcceleration();
    current.rightFoot.ang_pos = getRPY(mRightFoot);
    current.rightFoot.ang_vel = mRightFoot->getCOMSpatialVelocity().head(3);
    current.rightFoot.ang_acc = mRightFoot->getCOMSpatialAcceleration().head(3);

    // Extract next desired state from the plan
    if (walkState.iter == 0) { // 3
        desired.vrpPos = VRPWindow.at(0); // 5
        desired.dcmPos = DCMTrajectory.at(0); // 6
    } else { // 7
        k = (int) floor(globalTime/timeILC);
        ILCIter = walkState.iter - footstepPlan->getFootstepStartTiming(learningIter*2); // 9
        ILCTime = (double) ILCIter*timeStep; // 9
        if (ILCIter == 0) { // 10
            if (learningIter < learningIterations) { // 11
                desired.vrpPos = VRPWindow.at(1);
                futureVRP = VRPTrajectory.at(walkState.iter + learningIterSamples - 1) +
                            kf * (desired.vrpPos - VRPTrajectory.at(walkState.iter-1)) + 
                            kl * (VRPTrajectory.at(walkState.iter-1) - VRPCommanded); // 13
                VRPWindow.erase(VRPWindow.begin()); // 14
                VRPWindow.push_back(futureVRP); // 14
            } else { // 15
                VRPWindow.erase(VRPWindow.begin()); // 16
                VRPWindow.push_back(VRPWindow.at(VRPWindow.size()-1)); // 16
            } // 17
            desired.vrpPos = (1 - ILCTime/learningIterDuration) * VRPWindow.at(0) + ILCTime/learningIterDuration * VRPWindow.at(1); // 18
            if (learningIter < learningIterations) { // 19
                desired.dcmPos = DCMTrajectory.at(learningIterSamples + k); // 20
            } else { // 21
                desired.dcmPos = VRPWindow.at(VRPWindow.size()-1); // 22
            }
            // 23
            double c = b/learningIterDuration;
            double alpha = 1 - c + exp(-1/c)*c;
            double beta = c - exp(-1/c)*(1+c);
            double gamma = exp(-1/c);
            for (int j = learningIterSamples-1; j > 2; j--) { // 24
                desired.dcmPos = alpha * VRPWindow.at(j-1) + beta * VRPWindow.at(j) + gamma * desired.dcmPos; // 25
            }
            alpha = 1 - ILCTime/learningIterDuration - c + exp(eta * (ILCTime - learningIterDuration))*c;
            beta = ILCTime/learningIterDuration + c - exp(eta * (ILCTime - learningIterDuration))*(1+c);
            gamma = exp(eta * (ILCTime - learningIterDuration));
            desired.dcmPos = alpha * VRPWindow.at(0) + beta * VRPWindow.at(1) + gamma * desired.dcmPos; // 26
        }
    }

    desired.dcmVel = eta * (desired.dcmPos - desired.vrpPos);

    desired.com.pos = desired.com.pos - eta * (desired.com.pos - DCMCommanded)*timeStep;
    DCMCommandedDot =  desired.dcmVel + kDCM * (desired.dcmPos - DCMCommanded);
    DCMCommanded = DCMCommanded + DCMCommandedDot * timeStep;
    desired.com.vel = -eta * (desired.com.pos - DCMCommanded);

    
    // how to compute below?
    desired.com.acc = Eigen::Vector3d(cm.xc_ddot(walkState.iter), cm.yc_ddot(walkState.iter), cm.zc_ddot(walkState.iter));
    desired.com.ang_pos = Eigen::Vector3d(cm.torso_roll(walkState.iter), cm.torso_pitch(walkState.iter), cm.torso_yaw(walkState.iter));
    desired.com.ang_vel = Eigen::Vector3d(cm.torso_roll_dot(walkState.iter), cm.torso_pitch_dot(walkState.iter), cm.torso_yaw_dot(walkState.iter));
    desired.com.ang_acc = Eigen::Vector3d(cm.torso_roll_ddot(walkState.iter), cm.torso_pitch_ddot(walkState.iter), cm.torso_yaw_ddot(walkState.iter));

    desired.zmpPos = Eigen::Vector3d(cm.xz(walkState.iter), cm.yz(walkState.iter), 0.0);

    desired.leftFoot.pos = Eigen::Vector3d(cm.left_x(walkState.iter), cm.left_y(walkState.iter), cm.left_z(walkState.iter));
    desired.leftFoot.vel = Eigen::Vector3d(cm.left_x_dot(walkState.iter), cm.left_y_dot(walkState.iter), cm.left_z_dot(walkState.iter));
    desired.leftFoot.acc = Eigen::Vector3d(cm.left_x_ddot(walkState.iter), cm.left_y_ddot(walkState.iter), cm.left_z_ddot(walkState.iter));
    desired.leftFoot.ang_pos = Eigen::Vector3d(cm.left_roll(walkState.iter), cm.left_pitch(walkState.iter), cm.left_yaw(walkState.iter));
    desired.leftFoot.ang_vel = Eigen::Vector3d(cm.left_roll_dot(walkState.iter), cm.left_pitch_dot(walkState.iter), cm.left_yaw_dot(walkState.iter));
    desired.leftFoot.ang_acc = Eigen::Vector3d(cm.left_roll_ddot(walkState.iter), cm.left_pitch_ddot(walkState.iter), cm.left_yaw_ddot(walkState.iter));

    desired.rightFoot.pos = Eigen::Vector3d(cm.right_x(walkState.iter), cm.right_y(walkState.iter), cm.right_z(walkState.iter));
    desired.rightFoot.vel = Eigen::Vector3d(cm.right_x_dot(walkState.iter), cm.right_y_dot(walkState.iter), cm.right_z_dot(walkState.iter));
    desired.rightFoot.acc = Eigen::Vector3d(cm.right_x_ddot(walkState.iter), cm.right_y_ddot(walkState.iter), cm.right_z_ddot(walkState.iter));
    desired.rightFoot.ang_pos = Eigen::Vector3d(cm.right_roll(walkState.iter), cm.right_pitch(walkState.iter), cm.right_yaw(walkState.iter));
    desired.rightFoot.ang_vel = Eigen::Vector3d(cm.right_roll_dot(walkState.iter), cm.right_pitch_dot(walkState.iter), cm.right_yaw_dot(walkState.iter));
    desired.rightFoot.ang_acc = Eigen::Vector3d(cm.right_roll_ddot(walkState.iter), cm.right_pitch_ddot(walkState.iter), cm.right_yaw_ddot(walkState.iter));

    // Compute inverse kinematics
    Eigen::VectorXd qDot =  getJointAccelerations(desired, current, walkState);

    // Set the velocity of the floating base to zero
    for (int i = 0; i < 6; ++i) {
        mRobot->setCommand(i, 0);
    }

    // Set the velocity of each joint as per inverse kinematics
    for (int i = 0; i < 50; ++i) {
        mRobot->setCommand(i+6,qDot(i));
    }

    // Store the results in files (for plotting)
    for (int i = 0; i < logList.size()-2; ++i) {
        logList.at(i)->log();
    }

    // Arm swing
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4+5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4-5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);

    // Update the iteration counters
    ++walkState.iter;
}

void Controller::VRPPlan() {
    std::cout << "executing VRPPlan" <<std::endl;
    Eigen::Vector3d currentVRP;
    logList.push_back(new Logger("vrp_trajectory", &currentVRP));
    for (int i = 0; i < cm.xz.size(); i++) {
        currentVRP = Eigen::Vector3d(cm.xz(i), cm.yz(i), comTargetHeight);
        VRPTrajectory.push_back(currentVRP);
        logList.at(logList.size()-2)->log();
        // std::cout << VRPTrajectory[i].x() << " " << VRPTrajectory[i].y() << " " << VRPTrajectory[i].z() << std::endl;
    }
}

void Controller::DCMPlan() {
    std::cout << "executing DCMPlan" <<std::endl;
    std::vector<Eigen::Vector3d>::iterator it;
    Eigen::Vector3d currentDCM;
    logList.push_back(new Logger("dcm_trajectory", &currentDCM));
    for (int i = VRPTrajectory.size()-1; i >= 0; i--) {
        int index = footstepPlan->getFootstepIndexAtTime(i);
        int start = footstepPlan->getFootstepStartTiming(index)+1;
        int end = footstepPlan->getFootstepEndTiming(index);
        int stepDuration = footstepPlan->getFootstepDuration(index);
        int startDS = end - doubleSupportSamples;
        double SSDuration = (double)(startDS - start + 1)*timeStep;
        double DSDuration = doubleSupportDuration;
        int tillEndOfPhase = i <= startDS ? startDS-i : end-i;
        // if (i == start) std::cout << "i == start" << std::endl;
        // if (i == end) std::cout << "i == end" << std::endl;
        if (end >= VRPTrajectory.size()) {
            learningIterations = index/2;
            currentDCM = VRPTrajectory.at(start);
            it = DCMTrajectory.begin();
            DCMTrajectory.insert(it, currentDCM);
            logList.at(logList.size()-1)->log();
            continue;
        }
        double alpha, beta, gamma, phaseTime;
        if (i <= startDS) { // we're in single support phase
            phaseTime = (double) (i - start)*timeStep;
            alpha = 1 + (-phaseTime - b + exp(eta*(phaseTime-SSDuration))*b)/SSDuration;
            beta = phaseTime/SSDuration + b/SSDuration - exp(eta*(phaseTime-SSDuration))*(1+b/SSDuration);
            gamma = exp(eta*(phaseTime-SSDuration));

            // std::cout << i << " + " << tillEndOfPhase << ": " << DCMTrajectory.at(tillEndOfPhase).x() << " " << DCMTrajectory.at(tillEndOfPhase).y() << " " << DCMTrajectory.at(tillEndOfPhase).z() << std::endl;
            currentDCM = (alpha+beta) * VRPTrajectory.at(start) + gamma*DCMTrajectory.at(tillEndOfPhase);
            // std::cout << i << ": " << currentDCM.x() << " " << currentDCM.y() << " " << currentDCM.z() << std::endl;
        } else { // we're in double support phase
            phaseTime = (double) (i - startDS - 1)*timeStep;
            alpha = 1 + (-phaseTime - b + exp(eta*(phaseTime-DSDuration))*b)/DSDuration;
            beta = phaseTime/DSDuration + b/DSDuration - exp(eta*(phaseTime-DSDuration))*(1+b/DSDuration);
            gamma = exp(eta*(phaseTime-DSDuration));

            // std::cout << i << " + " << tillEndOfPhase << ": " << DCMTrajectory.at(tillEndOfPhase).x() << " " << DCMTrajectory.at(tillEndOfPhase).y() << " " << DCMTrajectory.at(tillEndOfPhase).z() << std::endl;
            currentDCM = alpha * VRPTrajectory.at(startDS) + beta * VRPTrajectory.at(end) + gamma * DCMTrajectory.at(tillEndOfPhase);
            // std::cout << i << ": " << currentDCM.x() << " " << currentDCM.y() << " " << currentDCM.z() << std::endl;
        }
        // std::cout << start << ": " << VRPTrajectory.at(start).x() << " " << VRPTrajectory.at(start).y() << " " << VRPTrajectory.at(start).z() << std::endl;
        // std::cout << end << ": " << VRPTrajectory.at(end).x() << " " << VRPTrajectory.at(end).y() << " " << VRPTrajectory.at(end).z() << std::endl;
        it = DCMTrajectory.begin();
        DCMTrajectory.insert(it, currentDCM);
        logList.at(logList.size()-1)->log();
    }
}

Eigen::MatrixXd Controller::getJacobian() {

    Eigen::MatrixXd Jacobian_leftFoot, Jacobian_rightFoot;

    Jacobian_leftFoot =  mRobot->getJacobian(mLeftFoot,mBase) - mRobot->getCOMJacobian(mBase);
    Jacobian_rightFoot =  mRobot->getJacobian(mRightFoot,mBase) - mRobot->getCOMJacobian(mBase);

    for (unsigned int i=0; i<44; i++) { 
        if (i!=5 || i!=6) {
            Jacobian_leftFoot.col(i).setZero();
            Jacobian_rightFoot.col(i).setZero();
        }
    }

    Eigen::MatrixXd Jacobian_tot_(12, 56);
    Jacobian_tot_ << Jacobian_leftFoot, Jacobian_rightFoot;

    // Remove the floating base columns
    Eigen::MatrixXd Jacobian_tot(12, 50);
    Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);

    return Jacobian_tot;
}

Eigen::MatrixXd Controller::getJacobianDeriv() {

    Eigen::MatrixXd Jacobian_leftFoot, Jacobian_rightFoot;

    Jacobian_leftFoot =  mRobot->getJacobianSpatialDeriv(mLeftFoot,mBase) - mRobot->getCOMJacobianSpatialDeriv(mBase);
    Jacobian_rightFoot =  mRobot->getJacobianSpatialDeriv(mRightFoot,mBase) - mRobot->getCOMJacobianSpatialDeriv(mBase);

    for (unsigned int i=0; i<44; i++) { 
        if (i!=5 || i!=6) {
            Jacobian_leftFoot.col(i).setZero();
            Jacobian_rightFoot.col(i).setZero();
        }
    }

    Eigen::MatrixXd Jacobian_tot_(12, 56);
    Jacobian_tot_ << Jacobian_leftFoot, Jacobian_rightFoot;

    // Remove the floating base columns
    Eigen::MatrixXd Jacobian_tot(12, 50);
    Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);

    return Jacobian_tot;
}

Eigen::VectorXd Controller::getJointAccelerations(State desired, State current, WalkState walkState) {

    Eigen::VectorXd feedforward_vel = Eigen::VectorXd::Zero(12);
    feedforward_vel << desired.getRelLeftFootVelocity(), desired.getRelRightFootVelocity();

    Eigen::VectorXd feedforward_acc = Eigen::VectorXd::Zero(12);
    feedforward_acc << desired.getRelLeftFootAcceleration(), desired.getRelRightFootAcceleration();
     
    Eigen::VectorXd desired_pos(12);
    desired_pos << desired.getRelLeftFootPose(), desired.getRelRightFootPose();

    Eigen::VectorXd current_pos(12);
    current_pos << current.getRelLeftFootPose(), current.getRelRightFootPose();

    Eigen::VectorXd desired_vel(12);
    desired_vel << desired.getRelLeftFootVelocity(), desired.getRelRightFootVelocity();

    Eigen::VectorXd current_vel(12);
    current_vel << current.getRelLeftFootVelocity(), current.getRelRightFootVelocity();

    // Get the jacobian and pseudoinvert it
    Eigen::MatrixXd Jacobian_tot = getJacobian();
    Eigen::MatrixXd Jacobian_deriv = getJacobianDeriv();
    Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

    Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(12,12);

    // Torso Orientation
    _taskGain(0,0) = 1;
    _taskGain(1,1) = 1;
    _taskGain(2,2) = 1;

    // CoM Position
    _taskGain(3,3) = 1;
    _taskGain(4,4) = 1;
    _taskGain(5,5) = 1;

    // Swing Foot Orientation
    _taskGain(6,6) = 1;
    _taskGain(7,7) = 1;
    _taskGain(8,8) = 1;

    // Swing Foot Position
    _taskGain(9,9) = 1;
    _taskGain(10,10) = 1;
    _taskGain(11,11) = 1;

    double ikGain = 20;

    Eigen::VectorXd qDot_meas = mRobot->getVelocities().tail(50);

    Eigen::VectorXd qDot_des = PseudoJacobian_tot* (feedforward_vel + ikGain*_taskGain*(desired_pos - current_pos));
    //return qDot_des;
    Eigen::VectorXd qDDot = (qDot_des - qDot_meas) / mWorld->getTimeStep(); 

    //double k1 = 10;
    //double k2 = 10;
    //qDDot = PseudoJacobian_tot * (feedforward_acc + k1 * (desired_vel - current_vel) + k2 * (desired_pos - current_pos) - Jacobian_deriv * qDot_meas);

    return qDDot;
}


Eigen::Vector3d Controller::getRPY(dart::dynamics::BodyNode* body, dart::dynamics::BodyNode* referenceFrame) {
    Eigen::MatrixXd rotMatrix = body->getTransform(referenceFrame).rotation();

    Eigen::Vector3d RPY;
    RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
        atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
        atan2(rotMatrix(1,0),rotMatrix(0,0));

    return RPY;
}

Eigen::Vector3d Controller::getRPY(dart::dynamics::BodyNode* body) {
    Eigen::MatrixXd rotMatrix = body->getTransform().rotation();

    Eigen::Vector3d RPY;
    RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
        atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
        atan2(rotMatrix(1,0),rotMatrix(0,0));

    return RPY;
}

Eigen::Vector3d Controller::getZmpFromExternalForces()
{
    Eigen::Vector3d zmp_v;
    bool left_contact = false;
    bool right_contact = false;

    Eigen::Vector3d left_cop;
    if(abs(mLeftFoot->getConstraintImpulse()[5]) > 0.01){
        left_cop << -mLeftFoot->getConstraintImpulse()(1)/mLeftFoot->getConstraintImpulse()(5), mLeftFoot->getConstraintImpulse()(0)/mLeftFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mLeftFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mLeftFoot->getWorldTransform().translation();
        left_cop = iTransl + iRotation*left_cop;
        left_contact = true;
    }

    Eigen::Vector3d right_cop;
    if(abs(mRightFoot->getConstraintImpulse()[5]) > 0.01){
        right_cop << -mRightFoot->getConstraintImpulse()(1)/mRightFoot->getConstraintImpulse()(5), mRightFoot->getConstraintImpulse()(0)/mRightFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mRightFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mRightFoot->getWorldTransform().translation();
        right_cop = iTransl + iRotation*right_cop;
        right_contact = true;
    }

    if(left_contact && right_contact){
        zmp_v << (left_cop(0)*mLeftFoot->getConstraintImpulse()[5] + right_cop(0)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
                 (left_cop(1)*mLeftFoot->getConstraintImpulse()[5] + right_cop(1)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
		 0.0;
    }else if(left_contact){
        zmp_v << left_cop(0), left_cop(1), 0.0;
    }else if(right_contact){
        zmp_v << right_cop(0), right_cop(1), 0.0;
    }else{
        // No contact detected
        zmp_v << 0.0, 0.0, 0.0;
    }

    return zmp_v;
}

void Controller::setInitialConfiguration() {
    initialConfiguration = mRobot->getPositions();

    // Floating Base
    initialConfiguration[0] = 0.0;
    initialConfiguration[1] = 4*M_PI/180.0;
    initialConfiguration[2] = 0.0;
    initialConfiguration[3] = 0.00;
    initialConfiguration[4] = 0.00;
    initialConfiguration[5] = 0.753;

    // Right Leg
    initialConfiguration[44] = 0.0;           // hip yaw
    initialConfiguration[45] = 3*M_PI/180;    // hip roll
    initialConfiguration[46] = -25*M_PI/180;  // hip pitch
    initialConfiguration[47] = 50*M_PI/180;   // knee pitch
    initialConfiguration[48] = -30*M_PI/180;  // ankle pitch
    initialConfiguration[49] = -4*M_PI/180;   // ankle roll         
    // Left Leg
    initialConfiguration[50] = 0.0;           // hip yaw
    initialConfiguration[51] = -3*M_PI/180;   // hip roll
    initialConfiguration[52] = -25*M_PI/180;  // hip pitch
    initialConfiguration[53] = 50*M_PI/180;   // knee pitch
    initialConfiguration[54] = -30*M_PI/180;  // ankle pitch
    initialConfiguration[55] = 4*M_PI/180;    // ankle roll       

    mRobot->setPositions(initialConfiguration);

    // Additional arm position setting
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0 );

    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

    mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0 );

    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 ); 
    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 ); 

    initialConfiguration = mRobot->getPositions();
}
