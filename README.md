# Matlab Inverse Kinematics Calculator

This project benchmarks three different ways to solve Inverse Kinematics for a custom 3 degrees of freedom robotic arm. The goal was to see if a Deep Neural Network could match the speed of a mathematical formula while retaining the wide application advantage of numerical solvers.

## The Results
I ran a benchmark on **100,000 random samples** using an Intel Core i3-1215U.

| Method | Latency (Per Sample) | Median Error | Speed Per Sample Compared to Numeric |
| :--- | :--- | :--- | :--- |
| **Closed-Form (Analytic)** | ~2 µs | 0.00 mm | N/A (base benchmark) |
| **Numerical IK** | ~487,000 µs (487ms) | 0.00 mm | 1x (Baseline) |
| **Neural Network** | **~13 µs** | **1.12 mm** | **~37,000x faster** |

*The Neural Network is 37,000x faster than the standard numerical solver, with an average position error of just 1.1mm.*

*Note: results are hardware dependant except position error*

![App Preview](figures/app_preview.png)
*(Screenshot of the custom MATLAB dashboard developed for this project)*

## Why This Matters
Robots need to calculate joint angles (alpha, beta, gamma) to reach a target position (x, y, z) in 3d space. There are usually two ways to do this, and understanding the tradeoffs is one of the goals of this program:

1.  **Analytic Solvers or Straight Up Math:**
    I wrote a specific equation for *this exact robot*.
    * *Pros:* Instant calculation.
    * *Cons:* **Not General Purpose.** If you make the robot arm 0.1 inches longer, or add a new joint, the formula breaks. You have to re-do the math.

2.  **Numerical Solvers or Algorithms:**
    I wrote an algorithm that uses the Levenberg-Marquardt method to find the answer.
    * *Pros:* **General Purpose.** This same algorithm works on *any* robot—a 3-joint arm, a 6-joint arm, or a humanoid. You don't need new math; you just give it the new robot model.
    * *Cons:* **Very Slow.** Because it has to guess-and-check thousands of times, it is slow for fast irl control.

3.  **Neural Network or AI Inference:**
    I trained an AI to learn the kinematics from data.
    * *Result:* It gives us the best of both worlds. It has the speed of the Formula (~13µs) but learns from data like the General Purpose method, so we don't have to derive complex equations by hand.

## Project Features
* **Hybrid Neural Network:** A 3-layer MLP with structure as foilows, 512-256-128 neurons trained on 100,000 dataset points.
* **Custom Data Generation:** A script that creates valid training data using Forward Kinematics to ensure math is actually possible.
* **Interactive App:** Currently a work in progress, come back soon :)
