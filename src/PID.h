#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;

  double t_error = 0.0; 	// total average error
  double best_error = 1.0e5; 	// best total average error 
  int n_step = 0; 		// n for iteration 
  
  int n_thres = 100; 		// n limits to switch Kp, Ki, Kd
  bool is_loop = true;		// flag to show whether the first loop in twiddle
  int num_para = 0;     	// to switch from Kp, Kd and Ki
  int speed_limit = 100;        // speed limit 

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dKp;
  double dKi;
  double dKd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed);

  /*
  * Calculate the total PID error and return steering angle.
  */
  double TotalError();

  /*
  * Twiddle.
  */
  void Twiddle();
 /*
  * Calculate the total PID error and return throttle.
  */
  double GetThrottle();
};

#endif /* PID_H */
