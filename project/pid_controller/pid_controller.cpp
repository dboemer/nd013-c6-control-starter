/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp_ = Kpi;
   Ki_ = Kii;
   Kd_ = Kdi;

   output_lim_min_ = output_lim_mini;
   output_lim_max_ = output_lim_maxi;

   p_cte_ = 0.0;
   i_cte_ = 0.0;
   d_cte_ = 0.0;

   dt_ = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   double prev_cte = p_cte_;

   p_cte_ = cte;

   if (dt_ > 0) {
      d_cte_ = (cte - prev_cte) / dt_;
   }
   else {
      d_cte_ = 0.0;
   }

   i_cte_ += (cte * dt_);
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control = Kp_*p_cte_ + Ki_*i_cte_ + Kd_*d_cte_;

   if (control < output_lim_min_) {
      control = output_lim_min_;
   }
   else if (control > output_lim_max_) {
      control = output_lim_max_;
   }

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt_ = new_delta_time;
   return dt_;
}