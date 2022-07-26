# Goal
- Maintain the current centroid
- X: 0
- Y: 0
- Vx: 0
- Vy: 0
- Orientation 0

Experiment premises
Formation control of 3 crazyflies center at 0,0 position and 0,0 velocity without orientation

# comment on the results obtained during test
We increased the maximum integration action to the system to see whether it recuded the oscillations in the formation: This did produce lesserror and indeed drive the system closer to the desired interagent distance



#Paramters
# Mission configuration
n_drones: 3
n_ugvs: 0
hz_freq: 50 # controller frquency
include_obstacle: false
include_orientation: false
n_obs: 0
## Formation configuration
### Uav
uav_c1_alpha: 0.20
uav_c2_alpha: 0.3 # this is recomputed
uav_c1_beta: 0.3
uav_c2_beta: 0.1 # this is recomputed
uav_c1_gamma: 0.25
uav_c2_gamma: 0.2
uav_c1_theta: 0.2
uav_c1_delta: 0.05



uav_d: 0.8
uav_formation_t:  3 #/* type must be minimum 2, 3: tuangle formation, 4 suare, 5 pentagon*/
uav_k: 7
uav_ratio:  0.8 # /* ratio between dist inter and obstacle*/
uav_eps: 0.1
uav_a: 5
uav_b: 5
uav_h_alpha: 0.2
uav_h_beta: 0.9
uav_d_obs: 0
uav_nav_type:  1 #/* convergence approach -1, parallel approach 2*/
uav_integrator: 2 #/* single/ double integrator*/
uav_dt:  0.1 #/* sample time*/
uav_int_max:  0.01 #/* maximum integral*/
