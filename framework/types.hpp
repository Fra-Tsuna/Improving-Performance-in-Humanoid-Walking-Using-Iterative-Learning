#pragma once

#include <Eigen/Core>
#include "utils.cpp"
#include <vector>

struct endEffector {
    Eigen::Vector3d pos, vel, acc, ang_pos, ang_vel, ang_acc;
};

// Contains the state of the LIP robot
struct State {
    endEffector com, leftFoot, rightFoot;
    Eigen::Vector3d zmpPos, vrpPos, dcmPos, dcmVel;

    inline Eigen::VectorXd getComPose() {
	Eigen::VectorXd comPose(6);
        comPose << com.ang_pos, com.pos;
        return comPose;
    }

    inline Eigen::VectorXd getSupportFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 0) sfPose << leftFoot.ang_pos, leftFoot.pos;
        else sfPose << rightFoot.ang_pos, rightFoot.pos;
        return sfPose;
    }

    inline Eigen::VectorXd getSupportFootOrientation(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 0) sfPose << leftFoot.ang_pos, Eigen::Vector3d::Zero();
        else sfPose << rightFoot.ang_pos, Eigen::Vector3d::Zero();
        return sfPose;
    }

    inline Eigen::VectorXd getSwingFootPose(bool supportFoot) {
	Eigen::VectorXd sfPose(6);
        if (supportFoot == 1) sfPose << leftFoot.ang_pos, leftFoot.pos;
        else sfPose << rightFoot.ang_pos, rightFoot.pos;
        return sfPose;
    }

    inline Eigen::VectorXd getComVelocity() {
	Eigen::VectorXd comVelocity(6);
        comVelocity << Eigen::Vector3d::Zero(), com.vel; //FIXME not zero
        return comVelocity;
    }

    inline Eigen::VectorXd getSwingFootVelocity(bool supportFoot) {
	Eigen::VectorXd sfVelocity(6);
        if (supportFoot == 1) sfVelocity << Eigen::Vector3d::Zero(), leftFoot.vel;
        else sfVelocity << Eigen::Vector3d::Zero(), rightFoot.vel; //FIXME not zero
        return sfVelocity;
    }

    inline Eigen::VectorXd getRelComPose(bool supportFoot) {
	return vvRel(getComPose(), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelSwingFootPose(bool supportFoot) {
	return vvRel(getSwingFootPose(supportFoot), getSupportFootPose(supportFoot));
    }

    inline Eigen::VectorXd getRelComVelocity(bool supportFoot) {
	return vvRel(getComVelocity(), getSupportFootOrientation(supportFoot));
    }

    inline Eigen::VectorXd getRelSwingFootVelocity(bool supportFoot) {
	return vvRel(getSwingFootVelocity(supportFoot), getSupportFootOrientation(supportFoot));
    }

    //------------

    inline Eigen::VectorXd getLeftFootPose() {
	Eigen::VectorXd pose(6);
        pose << leftFoot.ang_pos, leftFoot.pos;
        return pose;
    }

    inline Eigen::VectorXd getRightFootPose() {
	Eigen::VectorXd pose(6);
        pose << rightFoot.ang_pos, rightFoot.pos;
        return pose;
    }

    inline Eigen::VectorXd getRelLeftFootPose() {
	return vvRel(getLeftFootPose(), getComPose());
    }

    inline Eigen::VectorXd getRelRightFootPose() {
	return vvRel(getRightFootPose(), getComPose());
    }

    inline Eigen::VectorXd getRelLeftFootVelocity() {
	return vvRel(leftFoot.vel, com.ang_pos);
    }

    inline Eigen::VectorXd getRelRightFootVelocity() {
	return vvRel(rightFoot.vel, com.ang_pos);
    }

    inline Eigen::VectorXd getRelLeftFootAcceleration() {
	return vvRel(leftFoot.acc, com.ang_pos);
    }

    inline Eigen::VectorXd getRelRightFootAcceleration() {
	return vvRel(rightFoot.acc, com.ang_pos);
    }

    //------------

    inline void print() {
        std::cout << "com position: " << com.pos;
        std::cout << "left foot position: " << leftFoot.pos;
        std::cout << "right foot position: " << rightFoot.pos;
    }
};

struct WalkState {
    bool supportFoot;
    double simulationTime;
    int iter, footstepCounter, indInitial;
};

struct Vref {
    Vref(double _x, double _y, double _omega) : x(_x), y(_y), omega(_omega) {}

    double x = 0;
    double y = 0;
    double omega = 0;
};

struct CandidateMotion {
    Eigen::VectorXd xc, yc, zc;
    Eigen::VectorXd torso_roll, torso_pitch, torso_yaw;
    Eigen::VectorXd left_x, left_y, left_z, left_roll, left_pitch, left_yaw;
    Eigen::VectorXd right_x, right_y, right_z, right_roll, right_pitch, right_yaw;

    Eigen::VectorXd xc_dot, yc_dot, zc_dot;
    Eigen::VectorXd torso_roll_dot, torso_pitch_dot, torso_yaw_dot;
    Eigen::VectorXd left_x_dot, left_y_dot, left_z_dot, left_roll_dot, left_pitch_dot, left_yaw_dot;
    Eigen::VectorXd right_x_dot, right_y_dot, right_z_dot, right_roll_dot, right_pitch_dot, right_yaw_dot;

    Eigen::VectorXd xc_ddot, yc_ddot, zc_ddot;
    Eigen::VectorXd torso_roll_ddot, torso_pitch_ddot, torso_yaw_ddot;
    Eigen::VectorXd left_x_ddot, left_y_ddot, left_z_ddot, left_roll_ddot, left_pitch_ddot, left_yaw_ddot;
    Eigen::VectorXd right_x_ddot, right_y_ddot, right_z_ddot, right_roll_ddot, right_pitch_ddot, right_yaw_ddot;

    Eigen::VectorXd xz, yz;
    double tail_x, tail_y;

    Eigen::VectorXd mc;
};

struct CandidateJacobians {
    std::vector<Eigen::MatrixXd> jacobians;
};
