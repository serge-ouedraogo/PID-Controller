# PID COntroller
Self-Driving Car Engineer Nanodegree Program

---

## The Proportional Component: P
The proportional component P is a gain directly proportional to the cross-track error (cte). An increase of this gain leads to a strady oscillation of the vehicle.

## The Integral Component: I
This component is a gain that reflects the integral of the cte over time. For this code, this component is calculated every 600 steps, after the vehicle has accumulated errors over time. The visual effect is a reduction of the oscillations observed when P only was turned on. 

## The Derivative Component: D
The D component is a gain that is proportional to the rate of change of the cte. A perfectly tuned D component will decrease the amplitude of the oscillations (damped oscillation)


# Optimization
 The twiddle algorithm was used to choose the hyperparameters. Twiddle continuously tunes the PID controller hyperparameters (P, I, D) by first calculating the cte and incrementally ajusts the coefficients until the cte is minimized. 