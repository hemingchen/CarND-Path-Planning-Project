#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

using namespace std;


class Car {

public:
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;

  /**
    * Constructor
    */
  Car();

  Car(int id, double x, double y, double vx, double vy, double s, double d);

  /**
   * Destructor
   */
  virtual ~Car();

  double speed();

  int lane();

  double s_in_t(double t);

};

#endif //PATH_PLANNING_CAR_H
