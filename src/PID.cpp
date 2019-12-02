#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  twiddle_switch = true;//set twiddle or not
  it = 1;  
  N = 3;
  tol = 0.2;
  dp = [1.0 for row in range(3)];
  n_eval_steps = 100;
  best_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  // initial p_error is needed for calculating the first d_error
  if (it == 1){
    p_error = cte；
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  if (it % (n_steps * N)> n_steps){
    total_error += pow(cte,2);
  }
  //total step is 2*n_eval_steps, only update after n_eval_steps
  while (TotalError(dp)>tol){   
    if (twiddle_switch && it % (N * n_steps) == 0){
      for (int i = 0; i< dp.size(); i++){
        if (total_error < best_error){
          best_error = total_error;        
          dp[i] *= 1.1;              
        }
        else {
          p[i] -= 2* dp[i];
          if (total_error < best_error){
            best_error = best_error;
            dp[i] *= 1.1;
          }
          else{
            p[i] += dp[i];
            dp[i] *=0.9;
          }
        }
      }
      it += 1;
    }
  }
}

double PID::TotalError(std::vector<double> dp) {
  /**
   * TODO: Calculate and return the total error
   */
  total_error = std::accumulate(dp.begin(),dp.end(),0);
  return total_error;  // TODO: Add your total error calc here!
}