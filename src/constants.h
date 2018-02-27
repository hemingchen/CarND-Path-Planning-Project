#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

using namespace std;

/*****************************************************************************
 *  System parameters
 ****************************************************************************/
const double SYSTEM_SAMPLING_DT = 0.02;
const double LANE_WIDTH = 4.0;

/*****************************************************************************
 *  Trajectory generation parameters
 ****************************************************************************/
const int USE_N_PREV_WPS = 2;
const int GEN_N_WPS = 50;
const double PREDICT_HORIZON_IN_DIST = 30.; // m
const double CAR_SEPARATION = 30.; //m
const double MAX_ACC = .1;  // m/s^2
const double MAX_SPEED = 49.8 * 0.44704;  // m/s

#endif //PATH_PLANNING_CONSTANTS_H
