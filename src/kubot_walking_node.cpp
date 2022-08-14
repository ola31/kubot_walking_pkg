#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

//* Header file for C++
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/bind.hpp>

#include <time.h>

#include "footstep_planner.h"
#include "kubot_dxl_controller/dxl_controller.h"

#define X_ 0
#define Y_ 1
#define Z_ 2

//#define PI      3.141592 //->defined in "footstep_planner.h"
#define D2R     PI/180.
#define R2D     180./PI

#include <functional>

#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace Eigen;

/*******************
 * p_thread Settings
 * *****************/
bool is_run = true;
int Control_Cycle = 10; //ms


//* Joint Angle ****
VectorXd q_present_L(6);
VectorXd q_command_L(6);

VectorXd q_present_R(6);
VectorXd q_command_R(6);

VectorXd q_SyncRead_R(6);  //position from dynamixel syncRead(radian)
VectorXd q_SyncRead_L(6);


//* Initial guess joint (IK) ****
VectorXd q0_L(6);
VectorXd q0_R(6);

/*kinematic parameter*/
double L1 = 0.05;
double L2 = 0.0;
double L3 = 0.133;
double L4 = 0.138;
double L5 = 0.037;


/********************* Preview Control ************************/
/*
struct XY{
  double x;
  double y;
};
*/

double All_time_trajectory = 10000;//15.0;  //(sec)
double dt = 0.01;  //sampling time
int N = 300;  //preview NL
int n = (int)((double)All_time_trajectory/dt)+1;

double z_c = 0.22; //Height of CoM
double g = 9.81; //Gravity Acceleration

MatrixXd A(3,3);
MatrixXd B(3,1);
MatrixXd C(1,3);

int Qe = 1;
MatrixXd Qx(3,3);
Eigen::Matrix4d Q;
MatrixXd R(1,1); // 1*10^(-6)

Eigen::Vector4d BB;
MatrixXd II(4,1);
MatrixXd FF(4,3);
Eigen::Matrix4d AA;
MatrixXd KK;
MatrixXd Gi(1,1);
MatrixXd Gx;
VectorXd Gp(N);
MatrixXd XX;
MatrixXd AAc;

Eigen::EigenSolver<MatrixXd> eZMPSolver;

MatrixXd State_X(3,1);  // State(posi, vel, acc) for X_axis direction
MatrixXd State_Y(3,1);

struct XY input_u   = {.x=0, .y=0}; // control input jerk

struct XY zmp       = {.x=0, .y=0};
struct XY CoM       = {.x=0, .y=0};
struct XY zmp_error = {.x=0, .y=0}; // error or ZMP
struct XY sum_error = {.x=0, .y=0}; //sum of error
struct XY u_sum_p   = {.x=0, .y=0}; //sum or future Reference

struct XY get_zmp_ref(int step_time_index);
void set_system_model(void);
void set_Weight_Q(void);
void get_gain_G(void);
void initialize_CoM_State_Zero(void);
void initialize_starting_ZMP_Zero(void);
void Preview_Init_Setting(void);
MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R);

MatrixXd rotMat_X(double pitch);
MatrixXd rotMat_Y(double pitch);
MatrixXd rotMat_Z(double pitch);
VectorXd IK_Geometric(MatrixXd Body,double D, double A,double B, double AH, MatrixXd Foot);


double func_1_cos(double t, double init, double final, double T);
Vector3d func_1_cos(double t, Vector3d init, Vector3d final, double T);
VectorXd func_1_cos(double t, VectorXd init, VectorXd final, double T);
double Tick_to_Radian(int Tick_4095, int joint_index);
int Radian_to_Tick(int Radian, int joint_index);  //joint_index : 0,1,2...11

int time_index = 0;

//double dt = 0.01; //10ms
double T = 3;
double t = T + 1.0;
double dt_ms = 1.0;  //ms
int phase = 0;

FootstepPlanner FootPlaner;  //FootstepPlanner class object declaration
dxl_controller Dxl_;         //dxl_controller class object declaration

enum
{
    WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
};
int nDoF = 12;

VectorXd joint_direction(12);


void process(void){
  //dt = current_time.Double() - last_update_time.Double();
   //   cout << "dt:" << dt << endl;
  //time = time + dt;

  //double time = t;
  //cout << "time:" << time << endl;

  //* setting for getting dt at next step
  //last_update_time = current_time;

  //MatrixXd leftfoot(4,4);
  //MatrixXd rightfoot(4,4);
  //leftfoot = FootPlaner.get_Left_foot(time);
  //rightfoot = FootPlaner.get_Right_foot(time);

  ///test_msg.data = leftfoot(0,3);
  ///test_msg2.data = rightfoot(0,3);
  ///test_msg_L_z.data = leftfoot(2,3);
  ///test_msg_R_z.data = rightfoot(2,3);
  ///test_pub.publish(test_msg);
  ///test_pub2.publish(test_msg2);
  ///test_pub_L_z.publish(test_msg_L_z);
  ///test_pub_R_z.publish(test_msg_R_z);


  //* Read Sensors data
 ///GetjointData();

  MatrixXd Body(4,4);
  MatrixXd Foot_L(4,4);
  MatrixXd Foot_R(4,4);

  double body_z = z_c;//0.5;//m
  double L_foot_z = 0.0; //m
  double R_foot_z = 0.0; //m
  double foot_y = L1;//m
  double foot_x = 0;//m

  double step_length = 0.05;
  double step_height = 0.1;

  if((phase == 1) && time_index < n-N){ //use Preview Control

    zmp_error.x = zmp.x - FootPlaner.get_zmp_ref(time_index*dt).x;
    zmp_error.y = zmp.y - FootPlaner.get_zmp_ref(time_index*dt).y;

    sum_error.x = sum_error.x + zmp_error.x;
    sum_error.y = sum_error.y + zmp_error.y;

    for(int j=0;j<N;j++){
      u_sum_p.x = u_sum_p.x + Gp(j)*FootPlaner.get_zmp_ref(dt*(time_index + j + 1)).x;
      u_sum_p.y = u_sum_p.y + Gp(j)*FootPlaner.get_zmp_ref(dt*(time_index + j + 1)).y;
    }

    input_u.x = -Gi(0,0)*sum_error.x - (Gx*State_X)(0,0) - u_sum_p.x;
    input_u.y = -Gi(0,0)*sum_error.y - (Gx*State_Y)(0,0) - u_sum_p.y;

    //get CoM
    CoM.x = State_X(0,0);
    CoM.y = State_Y(0,0);

    //get new System State based on System state-space Eq
    State_X = A*State_X + B*input_u.x;
    State_Y = A*State_Y + B*input_u.y;

    //get ZMP based on System state-space Eq
    zmp.x = (C*State_X)(0,0);
    zmp.y = (C*State_Y)(0,0);

    u_sum_p.x = 0;
    u_sum_p.y = 0;

   }

  //std::cout<<"com_y : "<<CoM.y<<std::endl;


  //t = 0.0;
  if(phase == 0){
    if(t<T){
      Body <<  1,0,0,  CoM.x,
               0,1,0,  CoM.y,
               0,0,1, body_z,
               0,0,0,      1;

      Foot_L <<1,0,0,         0,
               0,1,0,    foot_y,
               0,0,1,  L_foot_z,
               0,0,0,         1;

      Foot_R <<1,0,0,      0,
               0,1,0, -foot_y,
               0,0,1, R_foot_z,
               0,0,0,      1;

      VectorXd q_L(6);
      VectorXd q_R(6);
      VectorXd init_L(6);
      VectorXd init_R(6);
      //init<<0,0,0,0,0,0;
      init_L = q_SyncRead_L;
      init_R = q_SyncRead_R;


      q_L = IK_Geometric(Body, L1, L3, L4, L5, Foot_L);
      q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);
      q_command_L = func_1_cos(t,init_L,q_L,T);
      q_command_R = func_1_cos(t,init_R,q_R,T);
      t += dt;
    }
    else {
      phase++;
      t = 0;
    }
  }
  else if(phase == 1){
/*
    Body <<  1,0,0,  CoM.x,
             0,1,0,  CoM.y,
             0,0,1, body_z,
             0,0,0,      1;
*/
    Body <<  1,0,0,  CoM.x,
             0,1,0,  CoM.y,
             0,0,1, body_z,
             0,0,0,      1;

  // Foot_L <<1,0,0,         0,
  //          0,1,0,    foot_y,
  //          0,0,1,  L_foot_z,
  //          0,0,0,         1;
  //
  // Foot_R <<1,0,0,      0,
  //          0,1,0, -foot_y,
  //          0,0,1, R_foot_z,
  //          0,0,0,      1;

    Foot_L = FootPlaner.get_Left_foot(time_index*dt);
    Foot_R = FootPlaner.get_Right_foot(time_index*dt);

    //std::cout<<"Foot_L"<<std::endl;
    //std:cout<<Foot_L<<std::endl;
/*
    test_msg_L_z.data = CoM.y; //Foot_R(0,3);
    test_msg_R_z.data = zmp.y;//Foot_R(1,3);

    test_pub_L_z.publish(test_msg_L_z);
    test_pub_R_z.publish(test_msg_R_z);

    zmp_y_msg.data = FootPlaner.get_zmp_ref(time_index*dt).y;
    L_foot_z_msg.data = Foot_L(2,3);
    R_foot_z_msg.data = Foot_R(2,3);

    L_foot_z_pub.publish(L_foot_z_msg);
    R_foot_z_pub.publish(R_foot_z_msg);
    zmp_y_pub.publish(zmp_y_msg);
*/

    VectorXd q_L(6);
    VectorXd q_R(6);
    VectorXd init(6);
    init<<0,0,0,0,0,0;

    q_L = IK_Geometric(Body, L1, L3, L4, L5, Foot_L);
    q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);

    if(time_index >= n-N){
      /*
      FILE *fp; // DH : for save the data
      fp = fopen("/home/ola/catkin_test_ws/src/Kubot_Sim_Pkg/src/data_save_ola/zmp_com_l_footup.txt", "w");
      for(int i=0;i<save.size();i++){
        if(i == 0){
          fprintf(fp,"time\t");
          fprintf(fp,"zmp_ref_y\t");
          fprintf(fp,"zmp_y\t");
          fprintf(fp,"COM_y\t");
          fprintf(fp,"r_Foot_z\n");
        }
        fprintf(fp, "%.4f\t", save[i][0]); //time
        fprintf(fp, "%.4f\t", save[i][1]); //zmp_ref_y
        fprintf(fp, "%.4f\t", save[i][2]); //zmp y
        fprintf(fp, "%.4f\t", save[i][3]); //CoM y
        fprintf(fp, "%.4f\n", save[i][4]); //r_foot_z
      }
      fclose(fp);
      */

      phase++;

    }
/*
    if(time_index*dt > 5){
      R_foot_z = func_1_cos(t,0,foot_height,T);
      Foot_R <<1,0,0,      0,
               0,1,0, -foot_y,
               0,0,1,  R_foot_z,
               0,0,0,      1;
      q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);
      if(t < T) t+=dt;
    }
*/
/*
    std::vector<double> data;
    data.push_back(time_index*dt);
    data.push_back(get_zmp_ref(time_index).y);
    data.push_back(zmp.y);
    data.push_back(CoM.y);
    data.push_back(R_foot_z);
    save.push_back(data);
    */

    time_index++;
    q_command_L = q_L;
    q_command_R = q_R;
  }
  else if(phase = 2){

  }




  //q_command_L = q_L;
  //q_command_R = q_R;


  //* Target Angles
/*
  joint[LHY].targetRadian = q_command_L(0);//*D2R;
  joint[LHR].targetRadian = q_command_L(1);//*D2R;
  joint[LHP].targetRadian = q_command_L(2);//*D2R;
  joint[LKN].targetRadian = q_command_L(3);//*D2R;
  joint[LAP].targetRadian = q_command_L(4);//*D2R;
  joint[LAR].targetRadian = q_command_L(5);//*D2R;

  joint[RHY].targetRadian = q_command_R(0);//*D2R;
  joint[RHR].targetRadian = q_command_R(1);//*D2R;
  joint[RHP].targetRadian = q_command_R(2);//*D2R;
  joint[RKN].targetRadian = q_command_R(3);//*D2R;
  joint[RAP].targetRadian = q_command_R(4);//*D2R;
  joint[RAR].targetRadian = q_command_R(5);//*D2R;
*/
  int joint_command[12];
  joint_command[0]  = Radian_to_Tick(q_command_R(0),0);
  joint_command[2]  = Radian_to_Tick(q_command_R(1),1);
  joint_command[4]  = Radian_to_Tick(q_command_R(2),2);
  joint_command[6]  = Radian_to_Tick(q_command_R(3),3);
  joint_command[8]  = Radian_to_Tick(q_command_R(4),4);
  joint_command[10] = Radian_to_Tick(q_command_R(5),5);

  joint_command[1]  = Radian_to_Tick(q_command_L(0),0);
  joint_command[3]  = Radian_to_Tick(q_command_L(1),1);
  joint_command[5]  = Radian_to_Tick(q_command_L(2),2);
  joint_command[7]  = Radian_to_Tick(q_command_L(3),3);
  joint_command[8]  = Radian_to_Tick(q_command_L(4),4);
  joint_command[11] = Radian_to_Tick(q_command_L(5),5);


  //* Publish topics
  /*
  LHY_msg.data = q_command_L(0);
  LHR_msg.data = q_command_L(1);
  LHP_msg.data = q_command_L(2);
  LKN_msg.data = q_command_L(3);
  LAP_msg.data = q_command_L(4);
  LAR_msg.data = q_command_L(5);

  LHY_pub.publish(LHY_msg);
  LHR_pub.publish(LHR_msg);
  LHP_pub.publish(LHP_msg);
  LKN_pub.publish(LKN_msg);
  LAP_pub.publish(LAP_msg);
  LAR_pub.publish(LAR_msg);
  */


/*First motion Complete.*/



  //* Joint Controller
  //jointController();
  Dxl_.Sync_Position_command_TxOnly(joint_command);

  t = t+dt;

}


void *p_function(void * data)
{
  ROS_INFO("thread fuction start");

  static struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC,&next_time);

  Preview_Init_Setting();
  FootPlaner.Plan();

  joint_direction <<  1,  1,  1,  1, -1,  1, -1, 1, -1, 1,  1,  1;
                   //RHY LHY RHR LHR RHP LHP RN LN RAP LAP RAR LAR

  int encoder_read[12];

  Dxl_.Initialize();
  Dxl_.Read_Dxl_Encoder_Once(encoder_read);
  q_SyncRead_R << Tick_to_Radian(encoder_read[0],0),\
                  Tick_to_Radian(encoder_read[1],1),\
                  Tick_to_Radian(encoder_read[2],2),\
                  Tick_to_Radian(encoder_read[3],3),\
                  Tick_to_Radian(encoder_read[4],4),\
                  Tick_to_Radian(encoder_read[5],5);

  q_SyncRead_L << Tick_to_Radian(encoder_read[6],6),\
                  Tick_to_Radian(encoder_read[7],7),\
                  Tick_to_Radian(encoder_read[8],8),\
                  Tick_to_Radian(encoder_read[9],9),\
                  Tick_to_Radian(encoder_read[10],10),\
                  Tick_to_Radian(encoder_read[11],11);

  int j = 0;

  while(is_run){

    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;
    process();
    j++;
    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next_time,NULL);
    std::cout<<" : "<<j<<std::endl;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kubot_walking_node");
  ros::NodeHandle nh;

  /* P Thread */
  pthread_t pthread;
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1);

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Publisher LHY_pub = nh.advertise<std_msgs::Float64>("command_joint/LHY", 1000);
  ros::Publisher LHR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);
  ros::Publisher LHP_pub = nh.advertise<std_msgs::Float64>("command_joint/LHP", 1000);
  ros::Publisher LKN_pub = nh.advertise<std_msgs::Float64>("command_joint/LKN", 1000);
  ros::Publisher LAP_pub = nh.advertise<std_msgs::Float64>("command_joint/LAP", 1000);
  ros::Publisher LAR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);

  ros::Publisher test_pub = nh.advertise<std_msgs::Float64>("kubotsim/test", 1000);
  ros::Publisher test_pub2 = nh.advertise<std_msgs::Float64>("kubotsim/test2", 1000);
  ros::Publisher test_pub_L_z = nh.advertise<std_msgs::Float64>("kubotsim/CoM_y", 1000);
  ros::Publisher test_pub_R_z = nh.advertise<std_msgs::Float64>("kubotsim/zmp_y", 1000);

  ros::Publisher zmp_y_pub = nh.advertise<std_msgs::Float64>("kubotsim/zmp_y_ref", 1000);
  ros::Publisher L_foot_z_pub = nh.advertise<std_msgs::Float64>("kubotsim/L_foot_z", 1000);
  ros::Publisher R_foot_z_pub = nh.advertise<std_msgs::Float64>("kubotsim/R_foot_z", 1000);

  std_msgs::Float64 LHY_msg;
  std_msgs::Float64 LHR_msg;
  std_msgs::Float64 LHP_msg;
  std_msgs::Float64 LKN_msg;
  std_msgs::Float64 LAP_msg;
  std_msgs::Float64 LAR_msg;

  std_msgs::Float64 test_msg;
  std_msgs::Float64 test_msg2;
  std_msgs::Float64 test_msg_L_z;
  std_msgs::Float64 test_msg_R_z;

  std_msgs::Float64 zmp_y_msg;
  std_msgs::Float64 L_foot_z_msg;
  std_msgs::Float64 R_foot_z_msg;
  std_msgs::Float64 foot_x_msg;

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
  if(thr_id < 0){
    ROS_ERROR("pthread create error");
    exit(EXIT_FAILURE);
  }
  else{
    ROS_INFO("pthread start...");
  }
  sleep(1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


MatrixXd rotMat_X(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<<1,  0,  0,
       0,  c, -s,
       0,  s,  s;
  return tmp;
}
MatrixXd rotMat_Y(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<< c,  0,  s,
        0,  1,  0,
       -s,  0,  c;
  return tmp;
}
MatrixXd rotMat_Z(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<< c, -s,  0,
        s,  c,  0,
        0,  0,  1;
  return tmp;
}


VectorXd IK_Geometric(MatrixXd Body,double D, double A,double B, double AH, MatrixXd Foot){
  Matrix3d Body_R, Foot_R;
  Vector3d Body_P, Foot_P;

  VectorXd q(6);

  Body_R = Body.block(0,0,3,3);
  Foot_R = Foot.block(0,0,3,3);

  Body_P = Body.block(0,3,3,1);
  Foot_P = Foot.block(0,3,3,1);

  Vector3d D_ = {0,D,0};
  Vector3d AH_ = {0,0,AH};

  Vector3d r = Foot_R.transpose() * ((Body_P + Body_R * D_) - (Foot_P + Foot_R*AH_));

  double C = r.norm();

  double cos_alpha = (A*A + B*B - C*C)/(2.0*A*B);
  double alpha;

  if(cos_alpha>=1) alpha = 0.0;
  else if(cos_alpha<=0.0) alpha = PI;
  else alpha = acos((A*A + B*B - C*C)/(2.0*A*B));

  double alpha2 = acos((C*C + B*B - A*A)/(2.0*A*C));

  double beta = atan2(r(Z_),r(Y_));
  q(3) = abs(PI - alpha); //Knee
  q(4) = atan2(r(Z_), r(X_)) - alpha2 - (PI/2.0); //ankle pitch
  q(5) = (PI/2.0) - beta; //ankle roll


  MatrixXd R(3,3);
  R = Body_R.transpose()*Foot_R*rotMat_X(-q(5))*rotMat_Y(-q(4))*rotMat_Y(-q(3));

  q(0) = atan2(-R(0,1), R(1,1)); //hip yaw
  q(1) = atan2(R(2,1), -R(0,1)*sin(q(0)) + R(1,1)*cos(q(0)) ); //hip roll
  q(2) = atan2(-R(2,0), R(2,2));  //hip pitch
  if(q(2)>PI/2) q(2) = q(2)-PI; //(ola)

 //cout<<"-----------"<<endl;
 //cout<<q*R2D<<endl;
 //cout<<"-----------"<<endl;
  return q;
}


void set_system_model(void){
  A << 1, dt, dt*dt/2,\
       0, 1,      dt,\
       0, 0,       1;

  B << dt*dt*dt/6,\
       dt*dt/2,\
       dt;
  C << 1, 0, -z_c/g;

}

void set_Weight_Q(void){
  Qe = 1;

  Qx << 0,0,0,\
        0,0,0,\
        0,0,0;

  Q(0,0) = Qe; Q.block(0,1,1,3) = MatrixXd::Zero(1,3);
  Q.block(1,0,3,1) = MatrixXd::Zero(3,1); Q.block(1,1,3,3)= Qx;

  R << 0.000001;
}

void initialize_CoM_State_Zero(void){
   State_X<<0,0,0;
   State_Y<<0,0,0;
}
void initialize_starting_ZMP_Zero(void){
   zmp.x = 0;
   zmp.y = 0;
}

void get_gain_G(void){

  BB << (C*B)(0,0),\
        B(0,0),\
        B(1,0),\
        B(2,0);

  II <<1,\
       0,\
       0,\
       0;

  FF.block(0,0,1,3) = C*A;
  FF.block(1,0,3,3) = A;

  AA.block(0,0,4,1) = II;
  AA.block(0,1,4,3) = FF;

  KK = ZMP_DARE(AA,BB,Q,R);

  Gi = (R + (BB.transpose()*KK*BB)).inverse()*(BB.transpose()*KK*II);
  Gx = (R + (BB.transpose()*KK*BB)).inverse()*(BB.transpose()*KK*FF);
  AAc = AA - BB*((R + (BB.transpose()*KK*BB)).inverse())*BB.transpose()*KK*AA;


  for(int i=0;i<N;i++){
    if(i==0){
      XX = -AAc.transpose()*KK*II;
      Gp(i) = -Gi(0,0);
    }
    else{
      Gp(i) = (((R + (BB.transpose()*KK*BB)).inverse())*(BB.transpose()*XX))(0,0);
      XX = AAc.transpose()*XX;
    }


    /** this is for data save **/
    /*
    FILE *fp2; // DH : for save the data
    fp2 = fopen("/home/ola/catkin_test_ws/src/Kubot_Sim_Pkg/src/data_save_ola/Gp.txt", "w");
    fprintf(fp2, "index\t");
    fprintf(fp2, "Gp\n");
    for(int i=0;i<N;i++){
      fprintf(fp2,"%d\t",i);
      fprintf(fp2, "%.4f\n", Gp(i)); //time
    }
    fclose(fp2);
    */

  }

}

void Preview_Init_Setting(void){
  set_system_model();
  set_Weight_Q();
  get_gain_G();
  initialize_CoM_State_Zero();
  initialize_starting_ZMP_Zero();
}

MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R)//Kookmin.Univ Preview
{
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize* 2, nSize* 2);

    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

    eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}




double func_1_cos(double t, double init, double final, double T){

    // t : current time

    double des;
    des = (final - init)*0.5*(1.0 - cos(PI*(t/T))) + init;
    return des;
}

Vector3d func_1_cos(double t, Vector3d init, Vector3d final, double T){
    Vector3d des;
    des(0) = (final(0) - init(0))*0.5*(1.0 - cos(PI*(t/T))) + init(0);
    des(1) = (final(1) - init(1))*0.5*(1.0 - cos(PI*(t/T))) + init(1);
    des(2) = (final(2) - init(2))*0.5*(1.0 - cos(PI*(t/T))) + init(2);
    return des;
}
VectorXd func_1_cos(double t, VectorXd init, VectorXd final, double T){
    VectorXd des(6);
    des(0) = (final(0) - init(0))*0.5*(1.0 - cos(PI*(t/T))) + init(0);
    des(1) = (final(1) - init(1))*0.5*(1.0 - cos(PI*(t/T))) + init(1);
    des(2) = (final(2) - init(2))*0.5*(1.0 - cos(PI*(t/T))) + init(2);
    des(3) = (final(3) - init(3))*0.5*(1.0 - cos(PI*(t/T))) + init(3);
    des(4) = (final(4) - init(4))*0.5*(1.0 - cos(PI*(t/T))) + init(4);
    des(5) = (final(5) - init(5))*0.5*(1.0 - cos(PI*(t/T))) + init(5);
    return des;
}


double Tick_to_Radian(int Tick_4095, int joint_index){ //joint_index : 0,1,2...11
  double Radian = joint_direction(joint_index) * Tick_4095*(PI/2048.0);
  return Radian;
}

int Radian_to_Tick(int Radian, int joint_index){  //joint_index : 0,1,2...11
  int Tick_4095;
  Tick_4095 = (int)(joint_direction(joint_index) * Radian*(2048.0/PI));
  return Tick_4095;
}
