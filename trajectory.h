#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <data_mutex.h>

class trajectory
{
private:
  bool Homming_checked = false;
  bool Homming_clicked = false;
  int Homming_t = 0;
  double Homming_duration = 5;
  double Homming_gain = 10;
  double Motor_Home_pos[14];
  double Motor_pos_init[14];
  double Motor_pos[14];
  double Homming_input[14];
public:
  trajectory();
  double* homming();
  void exchagne_mutex();
};

#endif // TRAJECTORY_H
