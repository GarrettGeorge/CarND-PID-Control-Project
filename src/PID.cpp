#include "PID.h"
#include <functional>
#include <iostream>
#include <cmath>
#include <chrono>
#include <numeric>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(
  double Kp,
  double Kd,
  double Ki,
  double grad_desc_tol,
  std::chrono::time_point<std::chrono::high_resolution_clock> now
) {
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  pid_error = 0;
  
  params = {Kp, Kd, Ki};

  max_steer = std::numeric_limits<double>::min();
  min_steer = std::numeric_limits<double>::max();

  deltas = {0.1, 0.1, 0.1};
  delta_index = 0;
  
  grad_desc_passes = 0;  
  this->grad_desc_tol = grad_desc_tol;
  should_grad_desc = true;
  min_error = std::numeric_limits<double>::max();
  
  start_time = now;
}

void PID::change(double cte, const std::function<void(double)>& callback) {
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds chrono_ms;
  typedef std::chrono::duration<float> chrono_f_sec;

  chrono_f_sec f_sec = Time::now() - start_time;
  chrono_ms duration = std::chrono::duration_cast<chrono_ms>(f_sec);
  // Ignore first 500 ms
  int ms = ((int)duration.count() + 100) % 1000;

  if (ms >= 100) {
    should_grad_desc = false;
  } else if (grad_desc_passes < (int)(duration.count() + 100) / 1000) {
    should_grad_desc = true;
  }

  // Failed to implement gradient descent/twiddle
  callback(
    (false)
      ? grad_desc(cte, ms)
      : run(cte, ms)
  );
}

double PID::run(double cte, int ms) {
  d_error = cte - p_error;
  p_error = cte;
  i_error +=  cte;

  if (ms > 550) {
    pid_error += pow(cte, 2);
  } else if (100 < ms && ms < 550) {
    pid_error = 0;
  }

  double steer = (-1 * params.at(0) * p_error) -
    (params.at(1) * d_error) -
    (params.at(2) * i_error);

  printf("p_ = %f | d_ = %f | i_ = %f\n",
    (-1 * params.at(0) * p_error),
    (params.at(1) * d_error),
    (params.at(2) * i_error)
  );
  
  if (steer > max_steer) {
    max_steer = steer;
  }
  if (steer < min_steer) {
    min_steer = steer;
  }

  return PID::normalize(steer, max_steer, min_steer);
}

double PID::grad_desc(double cte, int ms) {
  if (accumulate(begin(deltas), end(deltas), 0.0, plus<double>()) <= grad_desc_tol) {
    return run(cte, ms);
  }

  should_grad_desc = false;
  grad_desc_passes++;

  int interation = delta_index % 9;
  int index = floor(interation / 3);
  int step = interation % 3;

  printf("min_error = %4f", min_error);

  switch (step) {
    case 0:
      params.at(index) += deltas.at(index);
      break;
    case 1:
      if (pid_error < min_error) {
        min_error = pid_error;
        deltas.at(index) = deltas.at(index) * 1.1;
        delta_index++; // Jump forward to next index
        break;
      }
      params.at(index) -= 2.0 * deltas.at(index);
      break;
    case 2:
      if (pid_error < min_error) {
        min_error = pid_error;
        deltas.at(index) = deltas.at(index) * 1.1;
        break;
      }
      params.at(index) += deltas.at(index);
      deltas.at(index) = deltas.at(index) * 0.9;
      break;
    default:
      break;
  }

  delta_index++;
  printf("kp = %f\tkd = %f\tki = %f\n", params.at(0), params.at(1), params.at(2));
  
  return run(cte, ms);
}

double PID::normalize(double x, double max_x, double min_x) {
  return 2 * (x - min_x)/(max_x - min_x) - 1;
}

