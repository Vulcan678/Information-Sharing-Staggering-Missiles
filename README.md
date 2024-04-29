# Information-Sharing-Staggering-Missiles
Exploration of guidance laws and estimation methods in ballistic missile defense (BMD) scenarios, for single and staggering missiles.

## Part 1
Researching different guidance laws for single interceptor BMD scenarios of:
* Constant target maneuver
* Sinusoidal target maneuver
* Assuming true and wrong target dynamic delay
* Using true and stochastic measurements

Implementing and testing Kalman Filter for true target state estimation.

Resualting quantile (95%) of miss distance for different guidance laws, dynamic delay estimation and stochastic measurements

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/faac3119-2095-4fc5-8e72-46131632066a)

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/90fa4de7-7226-4d6d-9ef2-b8ebfc329d00)


## Part 2
Reconstructing the article:
V. Shaferman and Y. Oshman, Stochastic cooperative interception using information sharing
based on engagement staggering," Journal of Guidance, Control, and Dynamics, 2016. https://doi.org/10.2514/1.G000437
* Extending the simulation for 2 staggering interceptors
* Implementing an IMM Filter for bang-bang target maneuver estimation
* Adding a scenario with shared measurements between the interceptors
* Performing Monte Carlo simulations for performance evaluation

Resualting comparison CDF different interception strategies for the worst switch time

![image](https://github.com/Vulcan678/Information-Sharing-Staggering-Missiles/assets/153300908/f7ce440c-20a2-45ff-8fd4-35a647caa349)

Suggesting and building advanced methods for using inner simulation after the miss of the first missile to improve the estimation for the second staggaring missile.
