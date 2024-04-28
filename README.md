# Information-Sharing-Staggering-Missiles
Exploration of guidance laws and estimation methods in ballistic missile defense (BMD) scenarios, for single and staggering missiles.

## Part 1
Researching different guidance laws for single interceptor BMD scenarios of:
* Constant target maneuver
* Sinusoidal target maneuver
* Assuming true and wrong target dynamic delay
* Using true and stochastic measurements

Implementing and testing Kalman Filter for true target state estimation.

## Part 2
Reconstructing the article:
V. Shaferman and Y. Oshman, Stochastic cooperative interception using information sharing
based on engagement staggering," Journal of Guidance, Control, and Dynamics, 2016. https://doi.org/10.2514/1.G000437
* Extending the simulation for 2 staggering interceptors
* Implementing an IMM Filter for bang-bang target maneuver estimation
* Adding a scenario with shared measurements between the interceptors
* Performing Monte Carlo simulations for performance evaluation

Suggesting and building advanced methods for using inner simulation after the miss of the first missile to improve the estimation for the second staggaring missile.
