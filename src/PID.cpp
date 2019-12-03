#include "PID.h"
#include <vector>
#include <limits>
#include <cmath>
#include <numeric>
#include <iostream>
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
  param_idx = 1;
  tol = 1E-5;
//   dp = {0.1*Kp,0.1*Ki,0.1*Kd};
  dp = {1,1,1};
  p = {Kp,Ki,Kd};
  n_steps = 100;
  best_error = std::numeric_limits<double>::max();
  added = false;
  subtracted = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  // initial p_error is needed for calculating the first d_error
  
  if (it == 1){
    p_error = cte;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  if (it % (n_steps * N)> n_steps){
    total_error += pow(cte,2);
  }
  //total step is 2*n_eval_steps, only update after n_eval_steps

  while (TotalError(dp)>tol){   
    if (it % (N * n_steps) == 0 && twiddle_switch){   
      std::cout << "iteration: " << it << " | total error: "<< total_error << " | best error: "<< best_error <<std::endl;
      if (total_error < best_error){
          best_error = total_error;        
          dp[param_idx] *= 1.1;
          
          //continue to next parameter
          param_idx = (param_idx + 1) % 3;// idx may go over 3
          added = subtracted = false;            
        }
        if (!added && !subtracted){
          AddParam(param_idx, dp[param_idx]);
          added = true;
        }
      else if (added && !subtracted){
        AddParam(param_idx, - 2 * dp[param_idx]);
        subtracted = true;
        }
      else{
        AddParam(param_idx, dp[param_idx]);
        dp[param_idx] *=0.9;
        
        //continue to next parameter
        param_idx = (param_idx + 1) % 3;// idx may go over 3
        added = subtracted = false;
      }
      //next parameter, total error needs to be initialized.
      total_error = 0;
      std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;  
    }
    it++;
  }
}

double PID::TotalError(std::vector<double> dp) {
  /**
   * TODO: Calculate and return the total error
   */
  total_error = std::accumulate(dp.begin(),dp.end(),0);
  return total_error;  // TODO: Add your total error calc here!
}

void PID::AddParam(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}

double PID::Output() {
 /**
  * This function returns the output value for throttle/steering
  **/
  total_error = - Kp * p_error - Kd * d_error - Ki * i_error;
  return total_error;
}