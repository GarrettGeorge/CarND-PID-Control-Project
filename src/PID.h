#ifndef PID_H
#define PID_H

#include <functional>
#include <chrono>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double pid_error;
  /*
  * Parameters
  */ 
  std::vector<double> params;

  double max_steer;
  double min_steer;

  /*
  * Deltas
  */
  std::vector<double> deltas;
  int delta_index;

  int grad_desc_passes;
  double grad_desc_tol;
  bool should_grad_desc;
  double min_error;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
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
  void Init(
    double Kp = 0,
    double Ki = 0,
    double Kd = 0,
    double grad_desc_tol = 0.2,
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now()
  );

  /* 
   * Handle changes to car state
   */
  void change(double cte, const std::function<void(double)>& callback);

  double run(double cte, int ms);

  double grad_desc(double cte, int ms);

  static double normalize(double x, double max_x, double min_x);
};

#endif /* PID_H */
