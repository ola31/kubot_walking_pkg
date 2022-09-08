#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#define PI 3.141593

using namespace std;

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct XY
{
  double x;
  double y;
};


class FootstepPlanner
{
private:
  double fb_step_size;
  double step_time;
  double step_num;
  int start_foot;
  double dsp_ratio;
  double goal_turn_angle;
  double foot_height;
  double foot_distance;
  //vector<vector<double>> FootSteps;

  int last_foot;
  double last_side_foot[3];

  double LHR_add_q;
  double RHR_add_q;
  double L_foot_y_adding;
  double R_foot_y_adding;

  double Hip_roll_add_q;
  double Foot_y_adding;
  double func_1_cos_param;
  bool two_feet_on_ground;
  double compensation_start_time_param;





public:
  vector<vector<double>> FootSteps;
  FootstepPlanner();
  FootstepPlanner(double fb_step_size_, double step_time_, double step_num_, int start_foot_, double dsp_ratio_, double goal_turn_angle_,double foot_height);

  void set_fb_step_size(double fb_step_size_);
  void set_step_time(double step_time_);
  void set_step_num(int step_num_);
  void set_start_foot(double start_foot);
  void set_dsp_ratio(double dsp_ratio);
  void set_goal_turn_angle(double goal_turn_angle_);

  double get_fb_step_size();
  double get_step_time();
  int get_step_num();
  double get_start_foot();
  double get_dsp_ratio();
  double get_goal_turn_angle();
  double get_LHR_add_q();
  double get_RHR_add_q();
  void Plan();
  struct XY get_zmp_ref(double t);
  MatrixXd get_Left_foot(double t);
  MatrixXd get_Right_foot(double t);
  double ellipse_traj(double present_time, double start_time, double T, double foot_Height);
  double ellipse_traj(double t, double T, double foot_Height);
  double Bezier_curve_8th(double time, double start_t, double T);
  double func_1_cos(double t, double start_t, double T, double max);
  int factorial(int n);


};



#endif // FOOTSTEP_PLANNER_H
