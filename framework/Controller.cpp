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

    bool firstSupportFootIsLeft = true;
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
    logList.push_back(new Logger("desired.vrpPos", &desired.vrpPos));

    logList.push_back(new Logger("current.comPos", &current.com.pos));
    logList.push_back(new Logger("current.vrpPos", &current.vrpPos));
    logList.push_back(new Logger("VRPCommanded", &VRPCommanded));
    logList.push_back(new Logger("left_current", &current.leftFoot.pos));
    logList.push_back(new Logger("left_desired", &desired.leftFoot.pos));
    logList.push_back(new Logger("right_current", &current.rightFoot.pos));
    logList.push_back(new Logger("right_desired", &desired.rightFoot.pos));
    

    endOfLearning = footstepPlan->getFootstepEndTiming(desiredSteps);
    //test
    cm = computeCandidateMotion(footstepPlan, desired, 0, endOfLearning, endOfLearning);
    VRPPlan();
    DCMPlan();
    CoMPlan();
    alfa = current.leftFoot.ang_pos.z();
    RDelta= Eigen::AngleAxisd((current.com.ang_pos.z()-alfa),Eigen::Vector3d::UnitZ());
    desired.dcmPos = DCMTrajectory.at(0);
    desired.vrpPos = VRPTrajectory.at(0);
    desired.dcmVel = eta*(DCMTrajectory.at(0) - VRPTrajectory.at(0));
    VRPCommanded = desired.vrpPos;
    DCMCommanded = desired.dcmPos;
    for (int i = 0; i < footstepPlan->getFootstepEndTiming(1); i+=round(timeILC/timeStep)) { // 4
        VRPWindow.push_back(VRPTrajectory.at(i));
    }
}

Controller::~Controller() {}

void Controller::update() {
    double wind_vel = 44/3.6;
    double pressure = 0.613*wind_vel*wind_vel;
    double Force = pressure * 1.5 * 0.2 * 0.8;
    // This adds a push to the robot
    if (mWorld->getSimFrames()>=400 && mWorld->getSimFrames()<=1200) mBase->addExtForce(Eigen::Vector3d(0,10,0));
    if (mWorld->getSimFrames()>=1600&& mWorld->getSimFrames()<=2700) mBase->addExtForce(Eigen::Vector3d(0,-10,0));
    int index;
    //if (mWorld->getSimFrames()==500) system("gnuplot ../plotters/plot");
    walkState.simulationTime = mWorld->getSimFrames();
    if (walkState.iter < endOfLearning) {
        index = footstepPlan->getFootstepIndexAtTime(walkState.iter+1);
        if (learningIter < index/2) {
            nextIter = 1;
        }
        learningIter = index/2; // 1
        learningIterSamples = footstepPlan->getFootstepDuration(learningIter*2) + footstepPlan->getFootstepDuration(learningIter*2+1);
        learningIterDuration = (double) learningIterSamples * timeStep;
        globalTime = (double) walkState.iter * timeStep; // 2
        stepIter = walkState.iter - footstepPlan->getFootstepStartTiming(learningIter*2);
    } else {
        learningIter = learningIterations + 1;
        learningIterSamples = 100;
        stepIter = (walkState.iter-endOfLearning)%120;
    }
    time = stepIter * timeStep;


    // Retrieve current state in the world frame

    current.com.pos = mRobot->getCOM();
    current.com.vel = mRobot->getCOMLinearVelocity();
    current.com.acc = mRobot->getCOMLinearAcceleration();
    current.com.ang_pos = getRPY(mBase);
    current.com.ang_vel = mBase->getCOMSpatialVelocity().head(3);
    current.com.ang_acc = mBase->getCOMSpatialAcceleration().head(3);

    current.zmpPos = current.com.pos - current.com.acc / (omega*omega); //getZmpFromExternalForces(); //
    current.zmpPos[2] = 0.0;
    current.dcmPos = current.com.pos + b*current.com.vel;
    current.dcmVel = current.com.vel + b*current.com.acc;
    current.vrpPos = current.dcmPos - b*current.dcmVel;

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

    if (nextIter) {
        RDelta = Eigen::AngleAxisd((current.com.ang_pos.z()-alfa),Eigen::Vector3d::UnitZ());
        alfa = current.leftFoot.ang_pos.z();
        nextIter = 0;
    }
    

    // Extract next desired state from the plan
    if (walkState.iter == 0) { // 3
        desired.vrpPos = VRPWindow.at(0); // 5
        desired.dcmPos = DCMTrajectory.at(0); // 6
    } else { // 7
        k = stepIter / stepPerILC;
        ILCIter = stepIter % stepPerILC; // 9
        ILCTime = (double) ILCIter*timeStep; // 9
        if (ILCIter == 0) { // 10
            std::rotate(VRPWindow.begin(), VRPWindow.begin()+1, VRPWindow.end());
            VRPWindow.resize(learningIterSamples/stepPerILC-1);
            if (learningIter < learningIterations) { // 11
                desired.vrpPos = VRPWindow.at(0); // 12
                // with VRPc
                futureVRP = VRPTrajectory.at(walkState.iter + learningIterSamples - 1) +
                            kf * RDelta * (desired.vrpPos - VRPTrajectory.at(walkState.iter-1)) + 
                            kl.cwiseProduct(RDelta * (VRPTrajectory.at(walkState.iter-1) - VRPCommanded)); // 13
                // with VRPm
                // futureVRP = VRPTrajectory.at(walkState.iter + learningIterSamples) +
                //             kf * RDelta * (desired.vrpPos - VRPTrajectory.at(walkState.iter)) + 
                //             kl.cwiseProduct(RDelta * (VRPTrajectory.at(walkState.iter) - current.vrpPos));
                VRPWindow.push_back(futureVRP); // 14
            } else { // 15
                VRPWindow.push_back(VRPWindow.at(learningIterSamples/stepPerILC-2)); // 16
            } // 17
        }
        desired.vrpPos = (1 - ILCTime/timeILC) * VRPWindow.at(0) + ILCTime/timeILC * VRPWindow.at(1); // 18
        if (learningIter < learningIterations) { // 19
            desired.dcmPos = DCMTrajectory.at(learningIter*learningIterSamples + k*stepPerILC); // 20
        } else { // 21
            desired.dcmPos = VRPWindow.at(learningIterSamples/stepPerILC-1); // 22
        }
        // 23
        double c = b/timeILC;
        double alpha = 1 - c + exp(-1/c)*c;
        double beta = c - exp(-1/c)*(1+c);
        double gamma = exp(-1/c);
        for (int j = learningIterSamples/stepPerILC-1; j > 1; j--) { // 24
            desired.dcmPos = alpha * VRPWindow.at(j-1) + beta * VRPWindow.at(j) + gamma * desired.dcmPos; // 25
        }
        alpha = 1 - ILCTime/timeILC - c + exp(eta * (ILCTime - timeILC))*c;
        beta = ILCTime/timeILC + c - exp(eta * (ILCTime - timeILC))*(1+c);
        gamma = exp(eta * (ILCTime - timeILC));
        desired.dcmPos = alpha * VRPWindow.at(0) + beta * VRPWindow.at(1) + gamma * desired.dcmPos; // 26
    }

    desired.dcmVel = eta * (desired.dcmPos - desired.vrpPos);

    desired.com.pos = desired.com.pos - eta * (desired.com.pos - DCMCommanded)*timeStep;
    DCMCommandedDot =  desired.dcmVel + kDCM * (desired.dcmPos - DCMCommanded);
    DCMCommanded = DCMCommanded + DCMCommandedDot * timeStep;
    desired.com.vel = -eta * (desired.com.pos - DCMCommanded);

    VRPCommanded = desired.vrpPos + zeta * (current.dcmPos - desired.dcmPos);
    // desired.com.pos = desired.com.pos - eta * (desired.com.pos - desired.dcmPos)*timeStep;
    // desired.com.vel = -eta * (desired.com.pos - desired.dcmPos);
    
    // how to compute below?
    // desired.com.acc = Eigen::Vector3d(cm.xc_ddot(walkState.iter), cm.yc_ddot(walkState.iter), cm.zc_ddot(walkState.iter));
    // desired.com.ang_pos = Eigen::Vector3d(cm.torso_roll(walkState.iter), cm.torso_pitch(walkState.iter), cm.torso_yaw(walkState.iter));
    // desired.com.ang_vel = Eigen::Vector3d(cm.torso_roll_dot(walkState.iter), cm.torso_pitch_dot(walkState.iter), cm.torso_yaw_dot(walkState.iter));
    // desired.com.ang_acc = Eigen::Vector3d(cm.torso_roll_ddot(walkState.iter), cm.torso_pitch_ddot(walkState.iter), cm.torso_yaw_ddot(walkState.iter));
    // desired.zmpPos = Eigen::Vector3d(desired.vrpPos.x(), desired.vrpPos.y(), 0.0);
    if (walkState.iter < endOfLearning) {
        int stepTime = walkState.iter;
        if (false) {
            if (footstepPlan->getFootstepStartTiming(index)-walkState.iter == 0) {
                cm = computeCMForFootstep(footstepPlan, current, walkState.iter);
                stepTime = walkState.iter-footstepPlan->getFootstepStartTiming(index);
            }
        }
        desired.leftFoot.pos = Eigen::Vector3d(cm.left_x(stepTime), cm.left_y(stepTime), cm.left_z(stepTime));
        desired.leftFoot.vel = Eigen::Vector3d(cm.left_x_dot(stepTime), cm.left_y_dot(stepTime), cm.left_z_dot(stepTime));
        desired.leftFoot.acc = Eigen::Vector3d(cm.left_x_ddot(stepTime), cm.left_y_ddot(stepTime), cm.left_z_ddot(stepTime));
        desired.leftFoot.ang_pos = Eigen::Vector3d(cm.left_roll(stepTime), cm.left_pitch(stepTime), cm.left_yaw(stepTime));
        // desired.leftFoot.ang_vel = Eigen::Vector3d(cm.left_roll_dot(stepTime), cm.left_pitch_dot(stepTime), cm.left_yaw_dot(stepTime));
        // desired.leftFoot.ang_acc = Eigen::Vector3d(cm.left_roll_ddot(stepTime), cm.left_pitch_ddot(stepTime), cm.left_yaw_ddot(stepTime));

        desired.rightFoot.pos = Eigen::Vector3d(cm.right_x(stepTime), cm.right_y(stepTime), cm.right_z(stepTime));
        desired.rightFoot.vel = Eigen::Vector3d(cm.right_x_dot(stepTime), cm.right_y_dot(stepTime), cm.right_z_dot(stepTime));
        desired.rightFoot.acc = Eigen::Vector3d(cm.right_x_ddot(stepTime), cm.right_y_ddot(stepTime), cm.right_z_ddot(stepTime));
        desired.rightFoot.ang_pos = Eigen::Vector3d(cm.right_roll(stepTime), cm.right_pitch(stepTime), cm.right_yaw(stepTime));
        // desired.rightFoot.ang_vel = Eigen::Vector3d(cm.right_roll_dot(stepTime), cm.right_pitch_dot(stepTime), cm.right_yaw_dot(stepTime));
        // desired.rightFoot.ang_acc = Eigen::Vector3d(cm.right_roll_ddot(stepTime), cm.right_pitch_ddot(stepTime), cm.right_yaw_ddot(stepTime));
    }
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
    for (int i = 0; i < 10; ++i) {
        logList.at(i)->log();
    }

    // Arm swing
    if (walkState.iter < endOfLearning) {
        mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4+5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);
        mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4-5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180);
    }
    // Update the iteration counters
    ++walkState.iter;
}

CandidateMotion Controller::computeCMForFootstep(FootstepPlan* plan, State current, int startTime) {
    Eigen::VectorXd previousFootstepPos, currentFootstepPos, nextFootstepPos;
    CandidateMotion cm;
    int C = plan->getFootstepDuration(plan->getFootstepIndexAtTime(startTime+1));

    cm.left_x.resize(C);
    cm.left_y.resize(C);
    cm.left_z.resize(C);
    cm.right_x.resize(C);
    cm.right_y.resize(C);
    cm.right_z.resize(C);

    cm.left_x_dot.resize(C);
    cm.left_y_dot.resize(C);
    cm.left_z_dot.resize(C);
    cm.right_x_dot.resize(C);
    cm.right_y_dot.resize(C);
    cm.right_z_dot.resize(C);

    cm.left_x_ddot.resize(C);
    cm.left_y_ddot.resize(C);
    cm.left_z_ddot.resize(C);
    cm.right_x_ddot.resize(C);
    cm.right_y_ddot.resize(C);
    cm.right_z_ddot.resize(C);


    int currentIndex = plan->getFootstepIndexAtTime(startTime + 1);
    previousFootstepPos = plan->isSupportFootLeft(currentIndex)?current.rightFoot.pos:current.leftFoot.pos;
    currentFootstepPos = plan->isSupportFootLeft(currentIndex)?current.leftFoot.pos:current.rightFoot.pos;
    nextFootstepPos = plan->getFootstepPosition(currentIndex + 1);
    std::cout << currentIndex << " left= " << plan->isSupportFootLeft(currentIndex) 
    << ": " << previousFootstepPos(0) << ", " << previousFootstepPos(1) 
    << " | " << currentFootstepPos(0) << ", " << currentFootstepPos(1) 
    << " | " << nextFootstepPos(0) << ", " << nextFootstepPos(1) 
    << std::endl;


    for (int i = 0; i < C; i++) {

        // if (currentIndex == 0) previousFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        // else previousFootstepPos = plan->getFootstepPosition(currentIndex - 1);


        double actualStepHeight = currentIndex == 0 ? 0 : stepHeight;

        if (i < C - doubleSupportSamples) { // we are in single support
            double stepCompletion = (double)i/(C-doubleSupportSamples);

            // double stepCompletion = double(startTime + i - plan->getFootstepStartTiming(currentIndex)) /
            //        double(plan->getFootstepEndTiming(currentIndex) - plan->getFootstepStartTiming(currentIndex) - doubleSupportSamples);

            // std::cout << i << ", " << C << ", " << stepCompletion << std::endl;

            double dsDuration = double(C - doubleSupportSamples);

            if (plan->isSupportFootLeft(currentIndex)) { 
                // move right foot
                cm.left_x(i) = currentFootstepPos(0);
                cm.left_y(i) = currentFootstepPos(1);
                cm.left_z(i) = 0.0;
                cm.right_x(i) = nextFootstepPos(0) * stepCompletion + previousFootstepPos(0) * (1 - stepCompletion);
                cm.right_y(i) = nextFootstepPos(1) * stepCompletion + previousFootstepPos(1) * (1 - stepCompletion);
                cm.right_z(i) = 4 * actualStepHeight * stepCompletion * (1 - stepCompletion);

                cm.left_x_dot(i) = 0.0;
                cm.left_y_dot(i) = 0.0;
                cm.left_z_dot(i) = 0.0;
                cm.right_x_dot(i) = (nextFootstepPos(0) - previousFootstepPos(0)) / dsDuration;
                cm.right_y_dot(i) = (nextFootstepPos(1) - previousFootstepPos(1)) / dsDuration;
                cm.right_z_dot(i) = 4 * actualStepHeight * (1 - 2 * stepCompletion) / dsDuration;

                cm.left_x_ddot(i) = 0.0;
                cm.left_y_ddot(i) = 0.0;
                cm.left_z_ddot(i) = 0.0;
                cm.right_x_ddot(i) = 0.0;
                cm.right_y_ddot(i) = 0.0;
                cm.right_z_ddot(i) = - 8 * actualStepHeight / (dsDuration * dsDuration);
            } else {
                // move left foot
                cm.left_x(i) = nextFootstepPos(0) * stepCompletion + previousFootstepPos(0) * (1 - stepCompletion);
                cm.left_y(i) = nextFootstepPos(1) * stepCompletion + previousFootstepPos(1) * (1 - stepCompletion);
                cm.left_z(i) = 4 * actualStepHeight * stepCompletion * (1 - stepCompletion);
                cm.right_x(i) = currentFootstepPos(0);
                cm.right_y(i) = currentFootstepPos(1);
                cm.right_z(i) = 0.0;

                cm.left_x_dot(i) = (nextFootstepPos(0) - previousFootstepPos(0)) / dsDuration;
                cm.left_y_dot(i) = (nextFootstepPos(1) - previousFootstepPos(1)) / dsDuration;
                cm.left_z_dot(i) = 4 * actualStepHeight * (1 - 2 * stepCompletion) / dsDuration;
                cm.right_x_dot(i) = 0.0;
                cm.right_y_dot(i) = 0.0;
                cm.right_z_dot(i) = 0.0;

                cm.left_x_ddot(i) = 0.0;
                cm.left_y_ddot(i) = 0.0;
                cm.left_z_ddot(i) = 0.0;
                cm.right_x_ddot(i) = 0.0;
                cm.right_y_ddot(i) = 0.0;
                cm.right_z_ddot(i) = - 8 * actualStepHeight / (dsDuration * dsDuration);
            }
        } else { // we are in double support
            if (plan->isSupportFootLeft(currentIndex)) { 
                cm.left_x(i) = currentFootstepPos(0);
                cm.left_y(i) = currentFootstepPos(1);
                cm.left_z(i) = 0.0;
                cm.right_x(i) = nextFootstepPos(0);
                cm.right_y(i) = nextFootstepPos(1);
                cm.right_z(i) = 0.0;
            } else {
                cm.left_x(i) = nextFootstepPos(0);
                cm.left_y(i) = nextFootstepPos(1);
                cm.left_z(i) = 0.0;
                cm.right_x(i) = currentFootstepPos(0);
                cm.right_y(i) = currentFootstepPos(1);
                cm.right_z(i) = 0.0;
            }
            cm.left_x_dot(i) = 0.0;
            cm.left_y_dot(i) = 0.0;
            cm.left_z_dot(i) = 0.0;
            cm.right_x_dot(i) = 0.0;
            cm.right_y_dot(i) = 0.0;
            cm.right_z_dot(i) = 0.0;

            cm.left_x_ddot(i) = 0.0;
            cm.left_y_ddot(i) = 0.0;
            cm.left_z_ddot(i) = 0.0;
            cm.right_x_ddot(i) = 0.0;
            cm.right_y_ddot(i) = 0.0;
            cm.right_z_ddot(i) = 0.0;
        }
    }

    // Orientation (temporarily zero, orientation is fixed)

    cm.left_roll = Eigen::VectorXd::Zero(C);
    cm.left_pitch = Eigen::VectorXd::Zero(C);
    cm.left_yaw = Eigen::VectorXd::Zero(C);
    cm.left_roll_dot = Eigen::VectorXd::Zero(C);
    cm.left_pitch_dot = Eigen::VectorXd::Zero(C);
    cm.left_yaw_dot = Eigen::VectorXd::Zero(C);
    cm.left_roll_ddot = Eigen::VectorXd::Zero(C);
    cm.left_pitch_ddot = Eigen::VectorXd::Zero(C);
    cm.left_yaw_ddot = Eigen::VectorXd::Zero(C);

    cm.right_roll = Eigen::VectorXd::Zero(C);
    cm.right_pitch = Eigen::VectorXd::Zero(C);
    cm.right_yaw = Eigen::VectorXd::Zero(C);
    cm.right_roll_dot = Eigen::VectorXd::Zero(C);
    cm.right_pitch_dot = Eigen::VectorXd::Zero(C);
    cm.right_yaw_dot = Eigen::VectorXd::Zero(C);
    cm.right_roll_ddot = Eigen::VectorXd::Zero(C);
    cm.right_pitch_ddot = Eigen::VectorXd::Zero(C);
    cm.right_yaw_ddot = Eigen::VectorXd::Zero(C);

    cm.torso_roll = Eigen::VectorXd::Zero(C);
    cm.torso_pitch = Eigen::VectorXd::Zero(C);
    cm.torso_yaw = Eigen::VectorXd::Zero(C);
    cm.torso_roll_dot = Eigen::VectorXd::Zero(C);
    cm.torso_pitch_dot = Eigen::VectorXd::Zero(C);
    cm.torso_yaw_dot = Eigen::VectorXd::Zero(C);
    cm.torso_roll_ddot = Eigen::VectorXd::Zero(C);
    cm.torso_pitch_ddot = Eigen::VectorXd::Zero(C);
    cm.torso_yaw_ddot = Eigen::VectorXd::Zero(C);

    return cm;
}

void Controller::VRPPlan() {
    std::cout << "executing VRPPlan" <<std::endl;
    Eigen::Vector3d currentVRP;
    logList.push_back(new Logger("vrp_trajectory", &currentVRP));
    for (int i = 0; i < cm.xz.size(); i++) {
        currentVRP = Eigen::Vector3d(cm.xz(i), cm.yz(i), comTargetHeight);
        logList.at(10)->log();
        VRPTrajectory.push_back(currentVRP);
        // std::cout << currentVRP.x() << " " << currentVRP.y() << " " << currentVRP.z() << std::endl;
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
            logList.at(11)->log();
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
        logList.at(11)->log();
    }
}

void Controller::CoMPlan() {
    std::cout << "executing CoMPlan" <<std::endl;
    Eigen::Vector3d currentCoM = current.com.pos;
    logList.push_back(new Logger("com_trajectory", &currentCoM));
    for (int i = 0; i < VRPTrajectory.size(); i++) {
        logList.at(12)->log();
        currentCoM = currentCoM - eta * (currentCoM - DCMTrajectory.at(i))*timeStep;
    }
    logList.at(12)->log();
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
