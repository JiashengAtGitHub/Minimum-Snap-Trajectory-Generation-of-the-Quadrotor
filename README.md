# Minimum-Snap-Trajectory-Generation
* This work is an attempt that imitates [Minimum Snap Trajectory Generation and Control for Quadrotors.pdf](https://github.com/JiashengAtGitHub/Minimum-Snap-Trajectory-Generation/files/7230819/2011.-.cited.1050.-.Minimum.snap.trajectorygeneration.and.control.for.quadrotors.pdf) with my personal modification. 
* I only leave the information which I think is worth record. This README file does not include enough content for a completed wrap-up, but now I'm too busy to add more. I  apologize.:sweat_smile:
* The MATLAB code is integrated, however.

## 1. Examples Presentation
### Example 1: The quadrotor goes through the waypoints (0 0 0),(2 0 0),(2 2 0),(0 2 0), in order.
https://user-images.githubusercontent.com/77440902/134763321-8723978e-53fb-4360-b020-77714e85c66f.mp4

![Example 1 jpg](https://user-images.githubusercontent.com/77440902/134763342-5abc6835-1633-4ff3-a85d-bf342e442d09.jpg)
### Example 2: The quadrotor goes through the waypoints (0 0 0),(1 1 1),(1 1 0),(0 0 0), in order.
https://user-images.githubusercontent.com/77440902/134763703-2c8b7d8d-9476-45d2-8a57-d1e04b82ec4a.mp4

## 2. About the optimzation problem.
### 2.1 Minimum Snap
I use the same objective function as the one in the paper, namely minimum snap. This function, though not sufficiently, tends to make the trajectory feasible for a **fixed-pitch-propeller** quadrotor. If the trajectory is still infeasible after doing minimum snap, lengthen its time until it's feasible.
### 2.2 I think insuring 4th continuity is enough, and good.
The whole trajectory consists of m pieces of segments. Each segment is between 2 waypoints. On each waypoint, we have to decide how well the transition is, i.e., how many degrees of continuity you want. The paper used 4th continuity without an explanation, so I'm gonna add some of my personal thoughts.

Theoretically, the higher continuity, the smoother the trajectory is, the better. In practice, enforcing more and higher continuity means we have to add more constraints to our optimization problem. This leads to two problems: feasibility and solver's computational capacity.
* Feasibility. Too many constriants might make the problem have no feasible solutions.
* Solver's computational capacity. Even there are feasible solutions, too many constraints might make the solver be not able to solve the math. 

Now the question turns into a trade-off problem: how much continuity is proper? Through my simulations, I think the 4th continuity is a good choice.  


Structure of matrix P, A & b

4th continuity

## The problems I might work on in the future.
