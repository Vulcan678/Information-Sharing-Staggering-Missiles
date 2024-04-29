# Part 2 - Investigations of utilizing negative information

Reconstructing the article: V. Shaferman and Y. Oshman, Stochastic cooperative interception using information sharing based on engagement staggering," Journal of Guidance, Control, and Dynamics, 2016. https://doi.org/10.2514/1.G000437

Extending the simulation for a scenario of 2 interceptors against a single target with initialization measurements from ground radar.

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/364a3f0a-e5d6-4112-96ad-d75f2b37aca4)

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/e34eb8bd-c3ff-4cb2-bd8f-5232993bc50e)

Utilizing MATLAB built-in ODE solver - ode45. Using in the simulations only DGL1 guidance law as presented as one of the best laws found in Part 1.

The project is based on a scenario of interceptors with an initial distance between the interceptors (staggering missiles) and a measurement filtering improved to an IMM filter for better estimation against bang-bang target maneuvers.

Comparing state vector estimation by the second interceptor for sharing and non-sharing interceptors in open-loop (using true data for acceleration command).

![check_error_rho_lambda](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/004b05ea-79a7-46f0-886c-183a37dc7e31)

![check_error_gamma_t_at](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/a52ff23d-7c84-4436-af14-424ab314d48c)

Testing different switch times of the target's bang-bang maneuver and its impact on interceptor team miss distance for sharing and non-sharing interceptors.

![CDF_team_miss_vs_T_sw](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/de539ec4-813d-4db5-bf1c-3d8ef0607799)

Based on Monte-Carlo simulations evaluating the CDF function for each interception scenario:
* 1 interceptor
* 2 interceptors without information sharing
* 2 interceptors with information sharing

![CDF](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/5a2dd367-7bc5-49c0-aaa0-c22834d5b7ce)

Extending  the article's line of thought by utilizing the miss of the first interceptor (negative information) as additional information vital for the target's initial acceleration command and switch time estimation:
1. **Mode Assumption** - The acceleration mode after the switch is determined based on the estimation procedure presented. Using the mode after the switch, the missile continues the flight with an EKF estimator (with high target acceleration estimation certainty) instead of an IMM, because the acceleration mode is assumed to be known. If there was no switch, the second missile continues as before.
2. **Inner EKF Simulation** - The acceleration mode after the switch and the switch time are determined based on the estimation procedure presented. The current state vector is calculated from the beginning based on the known modes and the switch time with an EKF estimator (with high target acceleration estimation certainty) instead of an IMM, because the acceleration mode is assumed to be known.
3. **Inner IMM Simulation** - The acceleration mode after the switch and the switch time are determined based on the estimation procedure presented. The current state vector is calculated from the beginning based on the known modes and the switch time with an IMM estimator (with high target acceleration estimation certainty and a more stable transition matrix) because the acceleration mode is assumed to be known.

The initial acceleration command and switch time estimation are based on the IMM's mode probabilities:
* Initial acceleration command - the acceleration associated mode with the largest mean value of mode probabilities until mean mode probability flip
* Switch time - using DB with PDF (Probability Density Function) of miss distance of a single missile for each switch time, estimating the switch time by calculating $E\left[T_{sw}|miss_1\right]$

[Taran_Negative_Information_Utilization.pdf](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/files/15028725/Taran_Negative_Information_Utilization1.pdf)
