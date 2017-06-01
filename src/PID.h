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

  double t_error = 0.0; 	// total error t_error += pow(speed_limit*cte/speed,2);
  double best_error = 1.0e5; 	// best total average error 
  int n_step = 0; 		// n for iteration steps
  
  int n_thres = 50; 		// n limits to start calcuate total error and twiddle
  bool is_loop = true;		// flag to show whether the first loop in twiddle
  int num_para = 0;     	// flag to switch from Kp, Kd and Ki
  int speed_limit = 100;        // speed limit 

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dKp;			// variable for Kp
  double dKi;			// varialbe for Ki
  double dKd;			// variable for Kd

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
  * Calculate PID and return steering angle.
  */
  double TotalError();

  /*
  * Twiddle to get optimized parameters.
  */
  void Twiddle();
 /*
  * Calculate PID and return throttle.
  */
  double GetThrottle();
};

#endif /* PID_H */
