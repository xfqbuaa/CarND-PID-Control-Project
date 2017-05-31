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
  
  Kp = Kp==0? 0.1:Kp;
  Ki = Ki==0? 0.1:Ki;
  Kd = Kd==0? 0.1:Kd;
  
  dKp = Kp==0? 0.1:Kp/10.;
  dKi = Ki==0? 0.1:Ki/10.;
  dKd = Kd==0? 0.1:Kd/10.;
}

void PID::UpdateError(double cte, double speed) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  n_step++;
  if (n_step > n_thres) {
    //t_error += cte*cte;
    //t_error += pow(speed-speed_limit, 2);
    t_error += pow(speed_limit*cte/speed,2);
  }
}

double PID::TotalError() {
  
  Twiddle();
  
  double steering = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  steering = steering > 1? 1:steering;
  steering = steering < -1? -1:steering;
  
  return steering;
}

double PID::GetThrottle() {

  Twiddle();

  double throttle_position = 1-Kp*fabs(p_error) - Kd*fabs(d_error) - Ki*fabs(i_error);
  throttle_position = throttle_position > 1? 1:throttle_position;
  throttle_position = throttle_position <-1? -1:throttle_position;

  return throttle_position;
}

void PID::Twiddle() {
  
  // twiddle Kp 
  if (fabs(dKp)+fabs(dKd)+fabs(dKi) > 1e-5 && n_step > n_thres) {
    // to get the optimized Kp
    cout << "n:" << n_step << "/   Kp:" << Kp << "/   Kd:"<< Kd << "/   Ki:"<< Ki << "/   dKp:"<< dKp << "/   dKd:"<< dKd << "/   dKi:"<< dKi << "/   t_error:"<< t_error/n_step << "/   best_error:"<< best_error << "/   num_para:" << num_para << endl;
    
    if (n_step == n_thres) {
      Kp += dKp;
      is_loop = true;
      return;
    }
    
    if(is_loop == true) {
      if((t_error/n_step) < best_error) {
        best_error = t_error/n_step;
        if(num_para == 0) dKp *= 1.1;
        if(num_para == 1) dKd *= 1.1;
        if(num_para == 2) dKi *= 1.1;

        //cout << "111111" << endl;
      }else {
        if(num_para == 0) Kp -= 2*dKp;
        if(num_para == 1) Kd -= 2*dKd;
        if(num_para == 2) Ki -= 2*dKi;
        is_loop =false;
        num_para++;
        num_para = num_para % 3;
        //cout << "222222" << endl;
        return;
      }
    }else {
      if((t_error/n_step) < best_error) {
        best_error = t_error/n_step;
        if(num_para == 0) dKp *= 1.1;
        if(num_para == 1) dKd *= 1.1;
        if(num_para == 2) dKi *= 1.1;
   
        //cout << "333333" << endl;   
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
        
        //cout << "444444" << endl;
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

