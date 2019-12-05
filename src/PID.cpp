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
  
  twiddle_switch = false;//set twiddle or not
  it = 1;  
  N = 5;
  param_idx = 1;
  count = 0;
  dp = {0.1*Kp,0.1*Kd,0.1*Ki};//twiddle on p first, d second, and i last, depending on their importance
//   dp = {1,1,1};
  tol = 1E-5;
//   n_steps = 50;
  best_error = std::numeric_limits<double>::max();
  total_error = 0;
  avg_error = 0;
  added = false;
  subtracted = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  // initial p_error is needed for calculating the first d_error
  std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;  
  if (it == 1){
    p_error = cte;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  //calculate avg error
  count += 1;
//   if (it % (N*n_steps) > 0){
  total_error += pow(cte,2);
  avg_error = total_error / count;
//   }
  
  //total step is N*n_steps, only update after n_steps
  std::cout << twiddle_switch << "|" << TotalError(dp)<<std::endl;
  if (TotalError(dp) <= tol) {
    twiddle_switch = false;
  }
  
//   if (twiddle_switch && it % (n_steps * N)> n_steps){ 
  if (twiddle_switch){ 
    std::cout << "iteration: " << it << " | avg error: "<< avg_error << " | best error: "<< best_error <<std::endl;
 
    if (avg_error < best_error){
        best_error = avg_error;        
        dp[param_idx] *= 1.1;
        
        //continue to next parameter
        param_idx = (param_idx + 1) % 3;// idx may go over 3
        added = subtracted = false;   
    }
    
    if(!added && !subtracted){//just started twiddle or new param
      AddParam(param_idx, dp[param_idx]);
      added = true;
    }
    else if (added && !subtracted){//just added at last iteration and no improvement
      AddParam(param_idx, - 2 * dp[param_idx]);
      subtracted = true;
    }
    else{ //just subtracted 
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

double PID::TotalError(std::vector<double> dp) {
  /**
   * TODO: Calculate and return the total error
   */
  double sum = 0.0;
  for (int i = 0; i < 3; i++) {
    sum += dp[i];
  }
  return sum; // TODO: Add your total error calc here!
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
//   std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;  

  double output = - Kp * p_error - Kd * d_error - Ki * i_error;
  if (output > max_output_limit) {
    output = max_output_limit;
  }
  if (output < min_output_limit) {
    output = min_output_limit;
  }
  return output;
}