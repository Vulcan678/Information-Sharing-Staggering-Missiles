# Part 3 - Investigations of utilizing negative information

Interception scenario of 2 interceptors against a single target.

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/364a3f0a-e5d6-4112-96ad-d75f2b37aca4)

The project is based on a scenario of interceptors with a delay between the launch of each missile (staggering missiles) and a measurement share between the interceptors, based on the results of Part 2.

The investigation presented here includes 3 methods to utilize negative information - the miss of the first interceptor:
1. **Mode Assumption** - The acceleration mode after the switch is determined based on the estimation procedure presented. Using the mode after the switch, the missile continues the flight with an EKF estimator (with high target acceleration estimation certainty) instead of an IMM, because the acceleration mode is assumed to be known. If there was no switch, the second missile continues as before.
2. **Inner EKF Simulation** - The acceleration mode after the switch and the switch time are determined based on the estimation procedure presented. The current state vector is calculated from the beginning based on the known modes and the switch time with an EKF estimator (with high target acceleration estimation certainty) instead of an IMM, because the acceleration mode is assumed to be known.
3. **Inner IMM Simulation** - The acceleration mode after the switch and the switch time are determined based on the estimation procedure presented. The current state vector is calculated from the beginning based on the known modes and the switch time with an IMM estimator (with high target acceleration estimation certainty and a more stable transition matrix) because the acceleration mode is assumed to be known.

[Taran_Negative_Information_Utilization1.pdf](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/files/15028725/Taran_Negative_Information_Utilization1.pdf)
