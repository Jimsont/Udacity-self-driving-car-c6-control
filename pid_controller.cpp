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
  	Kp = Kpi;
  	Ki = Kii;
  	Kd = Kdi;
  	output_lim_max = output_lim_maxi;
  	output_lim_min = output_lim_mini;
  
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  	// update current error and total error
  if (abs(delta_time) < 0.000001)
  {
    return;
  }
  else
  {
  	cted = cte - ctep; 
  	ctep = cte;
	// anti-windup	
    if (abs(ctei + cte*delta_time) < 50.0)
    {
      ctei = ctei + cte*delta_time;
    }
  }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
  	
    double control;
  	if (abs(delta_time) < 0.000001)
    {
      control = output;
    }
  	else
    {
      control = (Kp*ctep + Kd*cted/delta_time + Ki*ctei);
      output = control;
    }
  	
    if (control < output_lim_min) control = output_lim_min;
  	if (control > output_lim_max) control = output_lim_max;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  	delta_time = new_delta_time;
}