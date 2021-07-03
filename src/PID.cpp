#include "PID.h"
#include <iostream>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {

}

PID::~PID() {

}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  cte_prev = 0.0;
}

void PID::InitTuning(int size, double nBatches){

  run_twiddle = true;
  
  dp[0] = Kp*0.1;
  dp[1] = Kd*0.1;
  dp[2] = Ki*0.1;

  selected_param = 0;
  
  step = 1;
  batch_size = size;
  max_batches = nBatches;
  confirm_steps = false;

  err = 0;
  best_err = numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = cte - cte_prev;

  cte_prev = cte;

  if(run_twiddle && step / batch_size < max_batches){
    // cout << step << endl;
    if( step % batch_size > 0){
      // Add square to avoid negative numbers
      err+=cte*cte;
    }

    if(step % batch_size == 0){
      cout<< "[" << step << "]: ";
      cout<< "Err: " << err;
      cout<< " | BestErr: " << best_err;

      if(!confirm_steps){
        switch(selected_param){
          case 0: Kp += dp[selected_param];break;
          case 1: Kd += dp[selected_param];break;
          case 2: Ki += dp[selected_param];break;
        }


        if(err < best_err){
          best_err = err;
          dp[selected_param] *= 1.1;
          
          selected_param = (selected_param + 1) % 3;
        } 
        else
        {
          switch(selected_param){
            case 0: Kp -= 2 *  dp[selected_param];break;
            case 1: Kd -= 2 *  dp[selected_param];break;
            case 2: Ki -= 2 *  dp[selected_param];break;
          }
          confirm_steps = true;
        }
      }
      else
      {
        if(err < best_err){
          best_err = err;
          dp[selected_param] *= 1.1;
          selected_param = (selected_param + 1) % 3;
        }
        else{

          switch(selected_param){
            case 0: Kp += dp[selected_param];break;
            case 1: Kd += dp[selected_param];break;
            case 2: Ki += dp[selected_param];break;
          }

          dp[selected_param] *= 0.9;
          selected_param = (selected_param + 1) % 3;
        }

        confirm_steps = false;
        

      }
      cout << "\tKp: " << Kp << " - Ki: " << Ki << " - Kd: " << Kd << endl;
      err = 0;
    }

    step++;
  }

  if(run_twiddle && step / batch_size == max_batches){
    run_twiddle = false;
    cout << endl << "End tuning cycle!" << endl;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return - Kp * p_error - Kd * d_error - Ki *  i_error;
}