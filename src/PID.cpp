#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  dKp = Kp==0? 0.1:Kp/5.;
  dKi = Ki==0? 0.001:Ki/5.;
  dKd = Kd==0? 1:Kd/5.;
}

void PID::UpdateError(double cte, double speed) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;  
  if (n_step > n_thres) {
    // cost error fuction.  
    t_error += cte*cte;
    t_error += pow((speed-speed_limit)/speed_limit, 2);
  }
  n_step++;
}

double PID::TotalError() {
  
  Twiddle();
  
  double steering = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  steering = steering > 1? 1:steering;
  steering = steering < -1? -1:steering;
  
  return steering;
}

double PID::GetThrottle() {
  
  // twiddle seems no effect to pid_thro hyperparameters optimization and so ignored here. 
  // through twiddle do not works here, but pid_thro can increase vehcile speed to about 55km/h 
  Twiddle();

  double throttle_position = 1.0-Kp*fabs(p_error) - Kd*fabs(d_error) - Ki*fabs(i_error);
  throttle_position = throttle_position > 1? 1:throttle_position;
  throttle_position = throttle_position <-1? -1:throttle_position;

  return throttle_position;
}

void PID::Twiddle() {
  
  // twiddle
  if (fabs(dKp)+fabs(dKd)+fabs(dKi) > 1e-5 && n_step > n_thres) {
  //if (n_step > n_thres) {

    if (n_step == n_thres + 1) {
      Kp += dKp;
      is_loop = true;
      return;
    }
    
    double current_error = t_error / (n_step - n_thres);
    //double current_error = t_error/n_step;

    // to get the optimized Kp, Ki and Kd
    cout << "n:" << n_step << "/   Kp:" << Kp << "/   Kd:"<< Kd << "/   Ki:"<< Ki << "/   dKp:"<< dKp << "/   dKd:"<< dKd << "/   dKi:"<< dKi << "/   current_error:"<< current_error << "/   best_error:"<< best_error << "/   num_para:" << num_para << endl;

    if(is_loop == true) {
      if(current_error < best_error) {
        best_error = current_error;
        if(num_para == 0) dKp *= 1.1;
        if(num_para == 1) dKd *= 1.1;
        if(num_para == 2) dKi *= 1.1;

        cout << "111111" << endl;
      }else {
        if(num_para == 0) Kp -= 2*dKp;
        if(num_para == 1) Kd -= 2*dKd;
        if(num_para == 2) Ki -= 2*dKi;
        is_loop =false;
        num_para++;
        num_para = num_para % 3;
        cout << "222222" << endl;
        return;
      }
    }else {
      if(current_error < best_error) {
        best_error = current_error;
        if(num_para == 0) dKp *= 1.1;
        if(num_para == 1) dKd *= 1.1;
        if(num_para == 2) dKi *= 1.1;
   
        cout << "333333" << endl;   
      }else{
        if(num_para == 0) {
	  Kp += dKp;
          dKp *= 0.9;
        }
        if(num_para == 1) {
	  Kd += dKd;
          dKd *= 0.9;
        }
        if(num_para == 2) {
	  Ki += dKi;
          dKi *= 0.9;
        }     
        
        cout << "444444" << endl;
      }  
      is_loop = true;   
    } 
    if(num_para == 0) Kp += dKp; 
    if(num_para == 1) Kd += dKd; 
    if(num_para == 2) Ki += dKi; 
    num_para++;
    num_para = num_para % 3;
  } 
}


