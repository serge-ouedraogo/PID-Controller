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
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  
 
  
  dp[0] = 0.1 * this->Kp;
  dp[1] = 0.1 * this->Ki;
  dp[2] = 0.1 * this->Kd;
  
  this->index = 0;
  this->i_step = 1;
  this->best_error =1E12; 
  this->num_steps= 600;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte;
   */
 
  if(this->i_step ==1 )
  {
    this->p_error = cte;
  }
  
  //PID equations
   this->i_error += cte;
   this->d_error = cte - this->p_error;
   this->p_error = cte;
  
  
  
  this->total_error += pow(cte,2);
  
  if(this->i_step % this->num_steps ==0)
  {
    if(this->total_error < this->best_error)
    {
      this->best_error = this->total_error;
      dp[this->index] *=1.1;
      //index = (index +1) % 3; //focus is on next parameteric
      this->twiddle_sum = false;
      this->twiddle_sub = false;
    }
    
    if(!this->twiddle_sum && !this->twiddle_sub)
    {
     
      PID::Twiddle(this->index, this->dp[this->index]);
      this->twiddle_sum = true;
    }
    else if(this->twiddle_sum && !this->twiddle_sub)
    {
      PID::Twiddle(this->index, -2*dp[this->index]);
      this->twiddle_sub = true;
    }
    else
    {
      PID::Twiddle(this->index, dp[this->index]);
      this->twiddle_sum =false;
      this->twiddle_sub =false;
      
      this->index = (this->index +1) % 3; //focus is on next parameter
    }
    this->total_error = 0;
 
    std::cout << "Kp = " << this->Kp << " Ki = " << this->Ki << " Kd = " << this->Kd << "\n\n";
    
  }
  this->i_step++;
}

void PID:: Twiddle(int index, double value)
{
  if(index == 0)
  {
    this->Kp += value;
  }
  
  if(index == 1)
  {
    this->Ki += value;
  }
  
  if(index == 2)
  {
    this->Kd += value;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  return -this->Kp * this->p_error - this->Ki * this->i_error - this->Kd * this->d_error;  // TODO: Add your total error calc here!
}