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
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_min = output_lim_mini;
  output_lim_max = output_lim_maxi;
}


void PID::UpdateError(double cte) {
   prev_cte = current_cte;
   current_cte = cte;
   int_cte += cte;
}

double PID::TotalError() {
    double diff_cte = current_cte - prev_cte;
    double control = -Kp * current_cte - Kd * diff_cte - Ki * int_cte;
    if (control < output_lim_min) {
      control = output_lim_min;
    } else if (control > output_lim_max) {
      control = output_lim_max;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   delta_time = new_delta_time;
}