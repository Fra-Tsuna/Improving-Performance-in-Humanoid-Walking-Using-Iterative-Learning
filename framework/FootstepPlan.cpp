#include "FootstepPlan.hpp"

FootstepPlan::FootstepPlan() {}

FootstepPlan::~FootstepPlan() {}

void FootstepPlan::plan(std::vector<Vref> vrefSequence, Eigen::VectorXd initialLeftFoot, Eigen::VectorXd initialRightFoot, bool _firstSupportFootIsLeft) {

    firstSupportFootIsLeft = _firstSupportFootIsLeft;

    int nSamples = vrefSequence.size();
    std::vector<double> stepStartingSequence; // when each step starts (in MPC samples)
    Vref vref = vrefSequence.at(0);
    stepStartingSequence.push_back(0);

    // Integrate reference velocities using omnidirectional model

    Eigen::VectorXd integrated_x(nSamples);
    Eigen::VectorXd integrated_y(nSamples);
    Eigen::VectorXd integrated_theta(nSamples);
    integrated_x(0) = (initialLeftFoot(3) + initialRightFoot(3)) / 2.0;
    integrated_y(0) = (initialLeftFoot(4) + initialRightFoot(4)) / 2.0;
    integrated_theta(0) = (initialLeftFoot(2) + initialRightFoot(2)) / 2.0;

    for (int i = 1; i < nSamples; i++) {
        integrated_theta(i) = integrated_theta(i-1) + vrefSequence.at(i-1).omega * timeStep;
        Eigen::Matrix2d rotationMatrix;
        rotationMatrix << cos(integrated_theta(i)), -sin(integrated_theta(i)), sin(integrated_theta(i)), cos(integrated_theta(i));
        Eigen::Vector2d temp = Eigen::Vector2d(integrated_x(i-1), integrated_y(i-1)) + rotationMatrix * Eigen::Vector2d(vrefSequence.at(i-1).x, vrefSequence.at(i-1).y) * timeStep;
        integrated_x(i) = temp(0);
        integrated_y(i) = temp(1);
    }

    // Plan step timings using heuristic rule
    
    int timingIndex = 0;
    while (timingIndex < vrefSequence.size()) {
        double velocityMagnitude = sqrt(vrefSequence.at(timingIndex).x * vrefSequence.at(timingIndex).x + vrefSequence.at(timingIndex).y * vrefSequence.at(timingIndex).y);
        double stepDuration = averageTime*(1 - (velocityMagnitude - averageVel)/(alpha + velocityMagnitude));
        if (stepDuration >= maxStepDuration) stepDuration = maxStepDuration;
        if (stepDuration <= minStepDuration) stepDuration = minStepDuration;

        timingIndex = timingIndex + (int)(stepDuration / timeStep);
        stepStartingSequence.push_back(timingIndex);
    }

    int nFootsteps = stepStartingSequence.size() - 1;

    Eigen::VectorXd xFootsteps(nFootsteps);
    Eigen::VectorXd yFootsteps(nFootsteps);
    Eigen::VectorXd thetaFootsteps(nFootsteps);

    // Select integrated footstep positions and orientations at planned timings

    Eigen::VectorXd xFootstepsSelected(nFootsteps);
    Eigen::VectorXd yFootstepsSelected(nFootsteps);
    Eigen::VectorXd thetaFootstepsSelected(nFootsteps);

    if (firstSupportFootIsLeft) {
        xFootstepsSelected(0) = initialLeftFoot(3);
        yFootstepsSelected(0) = initialLeftFoot(4);
        thetaFootstepsSelected(0) = initialLeftFoot(2);
    } else {
        xFootstepsSelected(0) = initialRightFoot(3);
        yFootstepsSelected(0) = initialRightFoot(4);
        thetaFootstepsSelected(0) = initialRightFoot(2);
    }

    for (int i = 1; i < nFootsteps; i++) {
        thetaFootstepsSelected(i) = integrated_theta(stepStartingSequence.at(i));
        Eigen::Matrix2d ortoRotationMatrix;

        double ortoTheta;
        if (firstSupportFootIsLeft) ortoTheta = thetaFootstepsSelected(i) + isEven(i) * M_PI/2.0 - isOdd(i) * M_PI/2.0;
        else ortoTheta = thetaFootstepsSelected(i) + isOdd(i) * M_PI/2.0 - isEven(i) * M_PI/2.0;

        ortoRotationMatrix << cos(ortoTheta), -sin(ortoTheta), sin(ortoTheta), cos(ortoTheta);
        Eigen::Vector2d temp = Eigen::Vector2d(integrated_x(stepStartingSequence.at(i)), integrated_y(stepStartingSequence.at(i))) + ortoRotationMatrix * Eigen::Vector2d(ell/2.0, 0.0);
        xFootstepsSelected(i) = temp(0);
        yFootstepsSelected(i) = temp(1);
        thetaFootstepsSelected(i) = integrated_theta(stepStartingSequence.at(i));
    }

    // Plan orientations using QP

    Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(nFootsteps, nFootsteps);
    for (int i = 0; i < nFootsteps - 1; i++) {
        differenceMatrix(i+1, i) = - 1;
    }

    Eigen::MatrixXd orientationsH = Eigen::MatrixXd::Identity(nFootsteps, nFootsteps);
    Eigen::VectorXd orientationsF = - thetaFootstepsSelected;
    Eigen::MatrixXd orientationsA = differenceMatrix;
    Eigen::VectorXd orientationsBmax = Eigen::VectorXd::Ones(nFootsteps) * thetaMax;
    Eigen::VectorXd orientationsBmin = - Eigen::VectorXd::Ones(nFootsteps) * thetaMax;

    thetaFootsteps = solveQP(orientationsH, orientationsF, orientationsA, orientationsBmin, orientationsBmax);

    for (int i = 0; i < nFootsteps; i++) {
        thetaFootsteps(i) = wrapToPi(thetaFootsteps(i));
    }

    // Plan positions using QP

    Eigen::MatrixXd doubleDifferenceMatrix = Eigen::MatrixXd::Identity(2 * nFootsteps, 2 * nFootsteps);
    for (int i = 0; i < 2 * (nFootsteps - 1); i++) {
        doubleDifferenceMatrix(i+2, i) = - 1;
    }

    Eigen::MatrixXd rotationMatrices = Eigen::MatrixXd::Zero(2 * nFootsteps, 2 * nFootsteps);
    for (int i = 0; i < nFootsteps; i++) {
        Eigen::Matrix2d rotationBlock;
        if (i == 0) {
            rotationBlock = Eigen::Matrix2d::Identity();
        } else {
            rotationBlock << cos(thetaFootsteps(i-1)), -sin(thetaFootsteps(i-1)), sin(thetaFootsteps(i-1)), cos(thetaFootsteps(i-1));
        }

        rotationMatrices.block(2*i, 2*i, 2, 2) = rotationBlock.transpose();
    }

    Eigen::VectorXd ellVector = Eigen::VectorXd::Zero(2 * nFootsteps);
    Eigen::VectorXd integratedFootstepsVector = Eigen::VectorXd::Zero(2 * nFootsteps);
    Eigen::VectorXd boxSizeVector = Eigen::VectorXd::Zero(2 * nFootsteps);
    for (int i = 0; i < nFootsteps; i++) {
        if (firstSupportFootIsLeft) ellVector(2*i + 1) = isEven(i) ? ell : -ell;
        else ellVector(2*i + 1) = isOdd(i) ? ell : -ell;
        integratedFootstepsVector(2*i) = xFootstepsSelected(i);
        integratedFootstepsVector(2*i + 1) = yFootstepsSelected(i);
        boxSizeVector(2*i) = sagittalDeviationMax;
        boxSizeVector(2*i + 1) = coronalDeviationMax;
    }

    Eigen::VectorXd initialFootstepVector = Eigen::VectorXd::Zero(2 * nFootsteps);
    initialFootstepVector(0) = integratedFootstepsVector(0);
    initialFootstepVector(1) = -integratedFootstepsVector(1);

    Eigen::MatrixXd positionsH = Eigen::MatrixXd::Identity(2 * nFootsteps, 2 * nFootsteps);
    Eigen::VectorXd positionsF = - integratedFootstepsVector;
    Eigen::MatrixXd positionsA = rotationMatrices * doubleDifferenceMatrix;
    Eigen::VectorXd positionsBmax = ellVector + boxSizeVector + initialFootstepVector;
    Eigen::VectorXd positionsBmin = ellVector - boxSizeVector + initialFootstepVector;

    Eigen::VectorXd positionsDecisionVariables = solveQP(positionsH, positionsF, positionsA, positionsBmin, positionsBmax);

    for (int i = 0; i < nFootsteps; i++) {
        xFootsteps(i) = positionsDecisionVariables(2*i);
        yFootsteps(i) = positionsDecisionVariables(2*i + 1);
    }

    // Fill footstep plan

    for (int i = 0; i < nFootsteps; i++) {
        Eigen::VectorXd tempFootstep(7);
        tempFootstep << xFootsteps(i), yFootsteps(i), 0.0, 0.0, 0.0, thetaFootsteps(i), stepStartingSequence.at(i);
        footstepPlan.push_back(tempFootstep);
    }


    /*for (int i = 0; i < nFootsteps; i++) {
        // it is in world frame!
        Eigen::VectorXd tempFootstep(7);
        Eigen::VectorXd initialCenter = (initialLeftFoot + initialRightFoot) / 2.0;
        int sign = (i%2 == 0) ? 1 : -1;
        if (i < 4) tempFootstep << initialCenter(3), initialCenter(4) + sign * 0.1, 0.0, 0.0, 0.0, thetaFootsteps(i), i*65;
        else tempFootstep << initialCenter(3) + (i-4)*0.21, initialCenter(4) + sign * 0.1, 0.0, 0.0, 0.0, thetaFootsteps(i), i*65;
        
        footstepPlan.push_back(tempFootstep);

    }*/

    // Write to file and plot

    std::ofstream foutReferenceVelocities(realpath("../data/referenceVelocities.txt", NULL), std::ofstream::trunc);
    for (int i = 0; i < vrefSequence.size(); i++) {
        foutReferenceVelocities << vrefSequence.at(i).x << " " << vrefSequence.at(i).y << " " << vrefSequence.at(i).omega << std::endl;
    }
    foutReferenceVelocities.close();

    std::ofstream foutIntegratedTrajectory(realpath("../data/integratedTrajectory.txt", NULL), std::ofstream::trunc);
    for (int i = 0; i < vrefSequence.size(); i++) {
        foutIntegratedTrajectory << integrated_x(i) << " " << integrated_y(i) << std::endl;
    }
    foutIntegratedTrajectory.close();

    std::ofstream foutSelectedFootsteps(realpath("../data/selectedFootsteps.txt", NULL), std::ofstream::trunc);
    for (int i = 0; i < xFootstepsSelected.size(); i++) {
        foutSelectedFootsteps << xFootstepsSelected(i) << " " << yFootstepsSelected(i) << " " << thetaFootstepsSelected(i) << std::endl;
    }
    foutSelectedFootsteps.close();

    std::ofstream foutFootstepPlan(realpath("../data/footstepPlan.txt", NULL), std::ofstream::trunc);
    for (int i = 0; i < footstepPlan.size(); i++) {
        foutFootstepPlan << footstepPlan.at(i).transpose() << std::endl;
    }
    foutFootstepPlan.close();
    
    //system("gnuplot ../plotters/plotReferenceVelocities");
}

Eigen::VectorXd FootstepPlan::getFootstep(int num) { return footstepPlan.at(num); }

Eigen::VectorXd FootstepPlan::getFootstepPosition(int num) { return footstepPlan.at(num).head(3); }

double FootstepPlan::getFootstepOrientation(int num) { return footstepPlan.at(num)(5); }

int FootstepPlan::getFootstepStartTiming(int num) { return (int)footstepPlan.at(num)(6); }

int FootstepPlan::getFootstepEndTiming(int num) { return (num + 1 < footstepPlan.size()) ? (int)footstepPlan.at(num + 1)(6) : -1; }

int FootstepPlan::getFootstepDuration(int num) { return (num + 1 < footstepPlan.size()) ? (int)(footstepPlan.at(num + 1)(6)-footstepPlan.at(num)(6)) : -1; }

int FootstepPlan::getFootstepIndexAtTime(int time) {
    int footstepIndex = 0;
    while (getFootstepEndTiming(footstepIndex) < time and footstepIndex < footstepPlan.size()) footstepIndex++;
    
    return footstepIndex;
}

bool FootstepPlan::isSupportFootLeft(int num) {
    if (firstSupportFootIsLeft) {
        if (num % 2 == 0) return true;
        else return false;
    } else {
        if (num % 2 == 0) return false;
        else return true;
    }
}

int FootstepPlan::getSize() { return footstepPlan.size(); }

std::vector<Eigen::VectorXd> FootstepPlan::getPlan() { return footstepPlan; }


