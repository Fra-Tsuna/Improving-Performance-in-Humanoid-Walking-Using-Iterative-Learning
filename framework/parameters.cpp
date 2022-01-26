#pragma once

#include <math.h>

// Definable parameters
// ********************

// Times
const double timeStep = 0.01;
const double singleSupportDuration = 0.3;
const double doubleSupportDuration = 0.2;
const double predictionTime = 1.0;
const int prev = 200;

// Walk parameters
const double stepHeight = 0.03;
const double comTargetHeight = 0.71;
const double kSwingFoot = 0.05;

// Fixed step parameters
const double stepx = 0.06;
const double stepy = 0.1;

// Constraints
const double thetaMax = 0.30;
const double footConstraintSquareWidth = 0.08;
const double deltaXMax = 0.25;
const double deltaYIn = 0.15;
const double deltaYOut = 0.28;

// Cost function weights
const double qZd = 1;
const double qVx = 0;
const double qVy = 0;
const double qZ = 0;
const double qF = 10000000000;

// Kinematic control
const double IKerrorGain = 1.0;

// Footstep planner
const double ell = 0.2;
const double coronalDeviationMax = 0.05;
const double sagittalDeviationMax = 0.5;
const double alpha = 0.1;
const double averageVel = 0.05;
const double averageTime = 0.5;
const double maxStepDuration = 2.0;
const double minStepDuration = 0.6;

// Used in the code
// ****************

const double omega = sqrt(9.81/comTargetHeight);
const int N = round(predictionTime/timeStep);
const int S = round(singleSupportDuration/timeStep);
const int D = round(doubleSupportDuration/timeStep);
const int M = 4; //ceil(N/(S+D));
const int doubleSupportSamples = 20;
