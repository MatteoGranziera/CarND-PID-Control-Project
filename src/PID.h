#ifndef PID_H
#define PID_H

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
   * Initialize tuning session run.
   * @param (size, nBatches) The initial tuning parameters
   */
  void InitTuning(int size, double nBatches);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double cte_prev;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle parameters
  */
 bool run_twiddle;
 int batch_size;
 int max_batches;

 int step;
 int selected_param;
 bool confirm_steps;

 double dp[3];
 double err;
 double best_err;


};

#endif  // PID_H