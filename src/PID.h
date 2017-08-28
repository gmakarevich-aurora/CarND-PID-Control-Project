#ifndef PID_H
#define PID_H

#include <cstddef>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  class ParamTuner {
    public:
      ParamTuner(double* k)
        : koefficient(k),
          koefficient_diff(0.1 * (*k)),
          state(START) {}

      bool tuneUp(double error, double best_error);

    private:
      double* koefficient;
      double  koefficient_diff;
      enum State {
          START = 0,
          INCREASED = 1,
          DECREASED = 2,
      };
      State state;
  };

  double total_error;
  double best_error;
  unsigned long steps_since_last_twiddle;
  unsigned long step;
  size_t tuner_idx;
  std::vector<ParamTuner> param_tuners;
};

#endif /* PID_H */
