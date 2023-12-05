# gjk-toi

This repo contains a Python implementation of a time of impact detector for general convex convex meshes undergoing arbitrary constant acceleration motion. GJK-TOI is a conservative advancement algorithm which uses GJK for  closest point estimations.  

### Motivation

Due to the discrete nature of realtime physics simulations, clipping(tunneling) can occur under high motion scenarios. In order to prevent these issues, most modern physics engines employ continuous collision detection algorithms. This enables the detection of impacts/collisions that occur in-between two consecutive physics update ticks. Continuous collision detection relies on robust time of impact algorithms which accurately determine the moment two meshes intersect assuming a certain type of motion model (typically: constant velocity, constant acceleration, general motion rigidbody motion, etc..)       


### Algorithm overview 

0) Set current time $t=0$
1) Compute the closest points $cp_a(t)$ and $cp_b(t)$ between the two convex meshes using GJK at time $t$
2) If the points are close enough ($||cp_a(t) - cp_b(t)||^2 \lt \epsilon^2 $)  exit. Return current time as time of impact
3) Find $dt$ such that $[cp_a(t+dt)-cp_a(t)] \cdot n - [cp_b(t+dt)-cp_b(t)] \cdot n = d$ where:
    - $d = ||cp_b(t) - cp_a(t)||_2$ and  
    - $n = [cp_b(t) - cp_a(t)] /d $
    - It can be shown that solving for dt is equivalent to finding the roots of a quadratic polynomial (refer to code comments for derivation) 
4) If not such dt exists exit. Objects can't possibly collide. 
5) Choose the smallest $dt$ and updated the current time $t = t + dt$
6) Go to step 1)

## Setup and running
The code in this repo requires Python >=3.9 and relies on a series of standard packages. To simplify installation a `requirements.txt` is provided in the root directory of the repo.  

The following steps are required to setup and run the demo: 
- setup conda / virtualenv with with Python >= 3.9 
- `pip install -r requirements.txt` - install python dependencies 
- `python main.py` - run the visualization demo 


## Demo 
TO DO
