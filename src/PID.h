#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError(std::vector<double> dp);
  
  void AddParam(int index, double amount);
  
  double Output();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  double min_output_limit = -1.0;
  double max_output_limit = 1.0;
  
  /**
   * Twiddle parameters
   */
  std::vector<double> dp;
  int it, param_idx, N, count;
//   int n_steps;// number of steps for twiddle
  double total_error, best_error,avg_error, tol;
  bool twiddle_switch, added, subtracted;
};

#endif  // PID_H