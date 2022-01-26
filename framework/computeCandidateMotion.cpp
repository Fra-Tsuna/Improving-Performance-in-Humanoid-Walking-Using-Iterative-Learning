#include "computeCandidateMotion.hpp"

double ramp(int time, int start)
{
    if (time < start) return 0;
    return time - start;
}

CandidateMotion computeCandidateMotion(FootstepPlan* plan, State current, int time, int C, int P)
{

std::ofstream file1 = std::ofstream(realpath("../data/zmpx.txt", NULL), std::ofstream::out);
std::ofstream file2 = std::ofstream(realpath("../data/zmpy.txt", NULL), std::ofstream::out);
std::ofstream file3 = std::ofstream(realpath("../data/xu.txt", NULL), std::ofstream::out);
std::ofstream file4 = std::ofstream(realpath("../data/yu.txt", NULL), std::ofstream::out);
std::ofstream file5 = std::ofstream(realpath("../data/xc.txt", NULL), std::ofstream::out);
std::ofstream file6 = std::ofstream(realpath("../data/yc.txt", NULL), std::ofstream::out);

std::ofstream file7 = std::ofstream(realpath("../data/left_x.txt", NULL), std::ofstream::out);
std::ofstream file8 = std::ofstream(realpath("../data/left_y.txt", NULL), std::ofstream::out);
std::ofstream file9 = std::ofstream(realpath("../data/left_z.txt", NULL), std::ofstream::out);
std::ofstream file10 = std::ofstream(realpath("../data/right_x.txt", NULL), std::ofstream::out);
std::ofstream file11 = std::ofstream(realpath("../data/right_y.txt", NULL), std::ofstream::out);
std::ofstream file12 = std::ofstream(realpath("../data/right_z.txt", NULL), std::ofstream::out);

    CandidateMotion cm;

    // Compute candidate ZMP trajectory
    Eigen::VectorXd xz(P), yz(P), mc(P);
    Eigen::VectorXd previousFootstepPos, currentFootstepPos, nextFootstepPos, movingConstraint;

    for (int i = 0; i < P; i++) {
        int currentIndex = plan->getFootstepIndexAtTime(time + i);

        currentFootstepPos = plan->getFootstepPosition(currentIndex);
        nextFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        if (currentIndex == 0) currentFootstepPos = (currentFootstepPos + nextFootstepPos) / 2;

        movingConstraint = currentFootstepPos + (nextFootstepPos - currentFootstepPos) * ramp(time + i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;

        if (plan->isSupportFootLeft(currentIndex)) {
             mc(i) = 1 - ramp(time + i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;
        } else {
             mc(i) = ramp(time + i, plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) / doubleSupportSamples;
        }
        
        xz(i) = movingConstraint(0);
        yz(i) = movingConstraint(1);
    }

    cm.xz = xz;
    cm.yz = yz;
    cm.mc = mc;

    // Compute tail

    double tail_x = exp(-omega*(P-C)*timeStep) * xz(P-1); // truncated ending of the preview
    double tail_y = exp(-omega*(P-C)*timeStep) * yz(P-1);

    for (int i = P - 1; i >= C; i--) {
        tail_x += (1 - exp(-omega*timeStep)) * exp(-omega*(i-C)*timeStep) * xz(i);
        tail_y += (1 - exp(-omega*timeStep)) * exp(-omega*(i-C)*timeStep) * yz(i);
    }

    cm.tail_x = tail_x;
    cm.tail_y = tail_y;

    // Compute bounded xu and yu

    Eigen::VectorXd xu(C);
    Eigen::VectorXd yu(C);
    xu(C-1) = tail_x;
    yu(C-1) = tail_y;

    for (int i = C - 2; i >= 0; i--) {
        xu(i) = xu(i+1) * exp(-omega*timeStep) + (1 - exp(-omega*timeStep)) * xz(i);
        yu(i) = yu(i+1) * exp(-omega*timeStep) + (1 - exp(-omega*timeStep)) * yz(i);
    }

    // Compute xs and ys

    Eigen::VectorXd xs(C);
    Eigen::VectorXd ys(C);

    xs(0) = 2 * current.com.pos(0) - xu(0);
    ys(0) = 2 * current.com.pos(1) - yu(0);

    for (int i = 1; i < C; i++) {
        xs(i) = xs(i-1) + timeStep * omega * (xz(i-1) - xs(i-1));
        ys(i) = ys(i-1) + timeStep * omega * (yz(i-1) - ys(i-1));
    }

    // Compute CoM trajectory

    cm.xc = (xu + xs) / 2;
    cm.yc = (yu + ys) / 2;
    cm.zc = Eigen::VectorXd::Ones(C) * current.com.pos(2);

    cm.xc_dot = omega * (xu - xs) / 2;
    cm.yc_dot = omega * (yu - ys) / 2;
    cm.zc_dot = Eigen::VectorXd::Zero(C);

    cm.xc_ddot = omega * omega * (cm.xc - cm.xz.head(C));
    cm.yc_ddot = omega * omega * (cm.yc - cm.yz.head(C));
    cm.zc_ddot = Eigen::VectorXd::Zero(C);

    // Compute foot trajectories

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

    for (int i = 0; i < C; i++) {
        int currentIndex = plan->getFootstepIndexAtTime(time + i);

        if (currentIndex == 0) previousFootstepPos = plan->getFootstepPosition(currentIndex + 1);
        else previousFootstepPos = plan->getFootstepPosition(currentIndex - 1);

        currentFootstepPos = plan->getFootstepPosition(currentIndex);
        nextFootstepPos = plan->getFootstepPosition(currentIndex + 1);

        double actualStepHeight = currentIndex == 0 ? 0 : stepHeight;

        if (time + i < plan->getFootstepEndTiming(currentIndex) - doubleSupportSamples) { // we are in single support

            double stepCompletion = double(time + i - plan->getFootstepStartTiming(currentIndex)) /
                   double(plan->getFootstepEndTiming(currentIndex) - plan->getFootstepStartTiming(currentIndex) - doubleSupportSamples);

            double dsDuration = double(plan->getFootstepEndTiming(currentIndex) - plan->getFootstepStartTiming(currentIndex) - doubleSupportSamples);

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


    // Plot

    file1 << xz << std::endl;
    file2 << yz << std::endl;
    file3 << xu << std::endl;
    file4 << yu << std::endl;
    file5 << (xs+xu)/2.0 << std::endl;
    file6 << (ys+yu)/2.0 << std::endl;

    file7 << cm.left_x << std::endl;
    file8 << cm.left_y << std::endl;
    file9 << cm.left_z << std::endl;
    file10 << cm.right_x << std::endl;
    file11 << cm.right_y << std::endl;
    file12 << cm.right_z << std::endl;

    //system("gnuplot ../plotters/plot_joint_level");
    //system("gnuplot ../plotters/plot_feet");

    return cm;
}
