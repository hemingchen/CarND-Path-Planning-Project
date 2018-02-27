#include <iostream>
#include <math.h>
#include "car.h"
#include "constants.h"

using namespace std;

/**
  * Constructor
  */
Car::Car() {};

Car::Car(int id, double x, double y, double vx, double vy, double s, double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
}

/**
  * Destructor
  */
Car::~Car() {}

/**
  * Other functions
  */
double Car::speed() {
  return sqrt(vx * vx + vy * vy);
}

int Car::lane() {
  int lane = -1;
  if (d > 0 && d < LANE_WIDTH) {
    lane = 0;
  } else if (d > LANE_WIDTH && d < 2 * LANE_WIDTH) {
    lane = 1;
  } else if (d > 2 * LANE_WIDTH && d < 3 * LANE_WIDTH) {
    lane = 2;
  }

  return lane;
}

double Car::s_in_t(double t) {
  // Assume t is small and car speed doesn't change
  return s + t * this->speed();
}