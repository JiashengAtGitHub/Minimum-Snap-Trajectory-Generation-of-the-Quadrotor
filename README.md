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

Theoretically, the higher continuity, the smoother the trajectory is, the better. In practice, enforcing more and higher continuity means we have to add more constraints to our optimization problem. This leads to two issues: feasibility and solver's computational capacity.
* Feasibility. Too many constriants might make the problem have no feasible solutions.
* Solver's computational capacity. Even there are feasible solutions, too many constraints might make the solver be not able to solve the math. 

Now the question turns into a trade-off problem: how much continuity is proper? Let's discuss it in two parts: the theoretical part and the experimental part.
* In theoretical part, it depends on what math model you use. I use the commonly-used model which seems "thrust + 3 toruqes" as inputs. Since torques correspond to 4th derivative of the reference, you only have to make it 3rd derivative continuity to make you simulation succeed. (In simulation we assume inputs can rapidly change.)
* In experiemntal part, 3rd derivative continuity might work or not. In reality, torques can not change at infinite rate, so error must exist. You can use feedback controller to vanish the error. If your feebacd control is not good, you may have to enforce 4th derivative continuity, or even more. Insuring 4th derivative of position continuity means you have a continous value of rotor speed if you look at the rotor model.
* I haven't conduct experiments. But only from the simulation part I guess 4th continuity is a proper choice. How about 3rd, 5th or even 6th continuity? I don't know until I will do the experiment.
  
### 2.3 Structure of matrix P, A 
This is a QP (Quadratic Program) problem which can be formed as:
<p align="center"> minimize  x^TPx </p> 
<p align="center">  s.t  Ax=b  </p> 
Below are structures of P and A. Here I use m=3 as an example, which means the whole trajectory consists of 3 segments.

* Matrix P
![P](https://user-images.githubusercontent.com/77440902/134834859-c033ec08-572d-4196-8524-f5aa26df608e.jpg)

* Matrix A
![A](https://user-images.githubusercontent.com/77440902/134834877-31e0a134-4edb-4533-b43c-045fb128f078.jpg)
* The 1st part of A corresponds to our desired position at each waypoint.
* The 2nd part of A corresponds to initial condition, here I specify the value of derivative of positions up to 4th.

Unlike other segments, the first segment of the trajectory does not have a "former neighbor". That means you have to specify the value of initial condition. However on the transitional waypoints we only have to make the two "neighbors" equal on the 1st, 2nd, 3rd, and 4th derivative of the position, which saves DOF (Degree of Freedom), rather than specifying values.
* The 3rd part of A corresponds to continuity on each waypoints, here I use 4th derivative.


### 2.4 How do you know the initial condition up to 4th derivative of position in reality? 
One simple answer is to make the initial state in hover state, which is what I suggested. In this way we all know, without any calculation, the 1st, 2nd, 3rd, 4th derivative are all zero. But what if the initial condition is somenthing else or like a random value?

Well before I talk about it please let me make one point clear in case anyone has no idea about the correlation between refernece and their correlated parameters:
* The position's 0th derivative is position. (Sounds stupid:sweat_smile:)
* The position's 1st derivative is velocity.
* The position's 2nd derivative, acceleration, corresponds to the quadrotor's attitude. 
* The position's 3rd derivative, jerk, corresponds to the quadrotor's anguler velocity.
* The position's 4th derivative, snap, corresponds to the quadrotor's anguler acceleration, which also corresponds to the motors' rotational speed.

The decription above merely depicts a rough picture. If you would like to know the exact correlation, you have to derive the math. I don't show my personal derivation here because it was a long and bad-looking derivation, but it makes sense and has passed the test.

Thus, we need to measure position, velocity, acceleration, anguler velocity, and anguler acceleration to obtain 0th, 1st, 2nd, 3rd, 4th derivative of position.
* The position can be measured by Vicon or GPS.
* The velocity can be obtained by displacement divided by sample time.
* The acceleration can be measured by accelerator.
* The anguler velocity can be measured by on-board gyroscope.
* The anguler acceleration can be measured by information above row with sample time. 

So many measurements which has to been done immediately since the quadrotor's initial condition may not be sustainable. That's why I recommend set initial condition at hover state.


## 3. The problems I might work on in the future.
* Do trial of making the whole trajectory expressed as one polynomial. See what will happen.
* **Specify full desired condition at each waypoints.**
* Study on spatial scaling & temporal scaling & optimal segments.
