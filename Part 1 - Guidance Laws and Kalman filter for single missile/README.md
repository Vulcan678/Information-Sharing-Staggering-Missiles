# Part 1 - Guidance Laws and Kalman filter for a single interceptor

Constructing a simulation of the BMD interception scenario for researching various guidance laws and the effect of the Kalman filter on overcoming stochastic measurements.

The scenario setup:

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/f4e3acdd-daba-47a7-aa30-622d6290c13a)

Creating the simulation using a 4th-order Runge-Kutta method to solve the ODEs of the missile dynamics.

Implementing different guidance laws:

| Guidance Law      | Interceptor's Dynamic Assumption | Target's Dynamic Assumption |
| ----------------- |:--------------------------------:|:---------------------------:|
| PN                | No Delay                         | No Delay                    |
| APN               | No Delay                         | No Delay                    |
| OGL               | Delayed                          | No Delay                    |
| LQDG (Ideal)      | No Delay                         | No Delay                    |
| LQDG (Semi-Ideal) | Delayed                          | No Delay                    |
| LQDG (Non-Ideal)  | Delayed                          | Delayed                     |
| DGL0 (Ideal)      | No Delay                         | No Delay                    |
| DGL0 (Semi-Ideal) | Delayed                          | No Delay                    |
| DGL1              | Delayed                          | Delayed                     |

The file main.m is summarizing the implementations of use and testing for different scenarios.

Testing the guidance laws without noise and with the target performing constant or sinusoidal maneuvers 

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/f775b30d-c7cd-49bd-91ff-7635b57e110a)

Implementing a Kalman filter to deal with stochastic measurements to better estimate the target's state for choosing the best acceleration command.

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/b4768922-43bb-4ef3-b10d-c016d12a9bbf)

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/07ee6658-448b-4476-bcb9-e0717cbf0b83)

Full explanation in :
[Taran_Missile Guidance and Data Estimation.pdf](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/files/15118927/Taran_Missile.Guidance.and.Data.Estimation.pdf)
