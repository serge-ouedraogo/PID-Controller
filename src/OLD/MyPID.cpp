#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
 
  
  p[0] = 0.1 * Kp;
  p[1] = 0.1 * Ki;
  p[2] = 0.1 * Kd;
  
  p_index = 0;
  step_num = 1;
  best_err = std::numeric_limits<double>::max(); 
  numSteps = 200;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte;
   */
 
  if(step_num ==1 )
  {
    p_error = cte;
  }
  
  //PID equations
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;
  
  
  total_err += cte * cte;
  
  
  if(step_num % numSteps ==0)
  {
    if(total_err < best_err)
    {
      best_err = total_err;
      p[p_index] *=1.1;
      //index = (index +1) % 3; //focus is on next parameter
      p_add = p_sub = false;
    }
    
    if(!p_add && !p_sub)
    {
     
      Twiddler(p_index, p[p_index]);
      p_add = true;
    }
    else if(p_add && !p_sub)
    {
      Twiddler(p_index, -2*p[p_index]);
      p_sub = true;
    }
    else
    {
      Twiddler(p_index, p[p_index]);
      p[p_index] *=0.9;
      p_add =false;
      p_sub =false;
      
      p_index = (p_index +1) % 3; //focus is on next parameter
    }
    total_err = 0;
    //Debugging prompts
    std::cout << "Adjusted parameters ..." << "\n";
    std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << "\n\n";
    
  }
  step_num++;
}

void PID:: Twiddler(int index, double value)
{
  if(index == 0)
  {
    Kp += value;
  }
  
  if(index == 1)
  {
    Ki += value;
  }
  
  if(index == 2)
  {
    Kd += value;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  return -Kp * p_error - Ki * i_error - Kd * d_error;  // TODO: Add your total error calc here!
}