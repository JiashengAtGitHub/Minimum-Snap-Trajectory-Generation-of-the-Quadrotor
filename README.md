# Minimum-Snap-Trajectory-Generation
* This work is an attempt to imitate [Minimum Snap Trajectory Generation and Control for Quadrotors.pdf](https://github.com/JiashengAtGitHub/Minimum-Snap-Trajectory-Generation/files/7230819/2011.-.cited.1050.-.Minimum.snap.trajectorygeneration.and.control.for.quadrotors.pdf) with my personal modification. 
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

Now the question turns into a trade-off problem: how much continuity is proper? Let's discuss it in two part: the theoretical part and the experimental part.
* In theoretical part, it depends on what math model you use. I use the commonly-used model which seems "thrust + 3 toruqes" as inputs. Since torques correspond to 4th derivative of the reference, you only have to make it 3rd derivative continuity to make you simulation succedd. (In simulation we can assume inputs can rapidly change.)
* In experiemntal part, 3rd derivative continuity might work or not. In reality, torques can not change at infinite rate, so error must exist. You can use feedback controller to vanish the error. If your feebacd control is not good, you may have to enforce 4th derivative continuity. In rotor's model that means rotor speed value is continous. 
* I haven't conduct experiments. But only from the simulation part I guess 4th continuity is a proper choice. How about 3rd, 5th or even 6th continuity? I don't know until I will do the experiment.
  
### 2.3 Structure of matrix P, A 
This is a QP (Quadratic Program) problem which can be formed as:
<p align="center"> minimize  x^TPx </p> 
<p align="center">  s.t  Ax=b  </p> 
Below are structures of P and A. Here i use m=3 as an example, which means the whole trajectory consists of 3 segments.

* Matrix P
![P](https://user-images.githubusercontent.com/77440902/134834859-c033ec08-572d-4196-8524-f5aa26df608e.jpg)

* Matrix A
![A](https://user-images.githubusercontent.com/77440902/134834877-31e0a134-4edb-4533-b43c-045fb128f078.jpg)
* The 1st part of A corresponds to desired waypoints.
* The 2nd part of A corresponds to initial condition, here I specify derivative of positions up to 4th.
* The 3rd part of A corresponds to continuity on each waypoints, here I use 4th derivative.

## 3. The problems I might work on in the future.
* Do trial of making the whole trajectory expressed as one polynimial. See what will happen.
* **specify full desired condition at each waypoints.**
* Study on spatial scaling & temporal scaling & optimal segments.
