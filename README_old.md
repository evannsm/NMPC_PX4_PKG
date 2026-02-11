# Nonlinear NMPC for PX4-ROS2 Deployment
This package was created during my PhD originally as a basis of comparison with the well-established and well-respected NMPC technique in order to make useful comparisons against novel control strategies (namely, Newton-Raphson Flow) developed at Georgia Tech's FACTSLab. We have compared this against the Newton-Raphson controller available in my other repository [`NRFlow_PX4_PKG`](https://github.com/evannsm/NRFlow_PX4_PKG)


This package allows for fast, compiled NMPC computations via Acados for a quadrotor UAV running PX4 alongside the MicroXRCE ROS2 bridge. The simulation version should work well with the `gz_x500` SITL model as well as the older `iris` model. The code has CLI inputs that allow the user to switch between sim/hardware mode as well as trajectory speeds. Trajectories are hard-coded into the code. Just change the `reffunc` variable to one of the available trajectory functions inside the code.


To change the model for your specific quadrotor, simply change the mass `self.m` in `quad_casadi_model.py`. Our model takes inputs to be thrust and angular rates. This means we don't use rotational dynamics, so we have no need for the inertia matrix.

## How to run:
Pre-requisites are below, complete those first before running
1. Clone this directory into your ROS2 workspace's source directory and build:
```bash
cd <your_ros2_ws/src>
git init
git remote add origin git@github.com:evannsm/NMPC_PX4_PKG.git
git fetch origin
git checkout -b main --track origin/main
git submodule update --init --recursive
cd ..
colcon build --symlink-install
```
2. If the above is successful (px4_msgs should take a while), set up your PX4 SITL as per their [user guide](https://docs.px4.io/main/en/ros2/user_guide.html)
3. Once complete:
   - initialize your PX4 gz_x500 (or iris) SITL simulation
   - initialize the MicroXRCEAgent
4. With a prepared simulation and PX4-ROS2 bridge initialized, run the code:
```bash
ros2 run nmpc_acados_px4 nmpc_quad log.log
```

I have set up inputs from the command line for simulation/hardware as well as trajectory speed settings.
To change the trajectory type, change `reffunc` inside the code

## Prerequisites

Install the following:
- **pyJoules** (Python energy/power measurement)
- **ACADOS** (built from source with shared libraries)
- **ACADOS Python interface** (`acados_template`)

---

## Setup Steps

### 1) Install pyJoules
~~~bash
pip install pyJoules
~~~

### 2) Install ACADOS
Follow the official guide to build ACADOS from source as per the [official intructions](https://docs.acados.org/installation/index.html)
1. Clone it and initialize all submodules:
```bash
# Clone it and initialize all submodules:
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```
2. Build with CMake:
```bash
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_DAQP=ON, a list of CMake options is provided below
make install -j4
```

### 3) Install the ACADOS Python interface
1. Install template as per the [python interface instructions](https://docs.acados.org/python_interface/index.html)
~~~bash
pip install -e <acados_root>/interfaces/acados_template
~~~

2. Set environment variables
Add these to your shell init (e.g., `~/.bashrc`), then `source ~/.bashrc`:
~~~bash
acados_root="your_acados_root" #probably: "home/user/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$acados_root/lib"
export ACADOS_SOURCE_DIR="$acados_root"
~~~

### 4) Install t_renderer binaries in order to be able to successfully render C code templates:
1. Go to the [t_renderer repo](https://github.com/acados/tera_renderer/releases/) and download the correct binaries
2. Place the binaries in <acados_root>/bin
3. Strip the name of everything after `t_renderer` : (e.g. `t_renderer-v0.2.0-linux-arm64 -> t_renderer`)
4. Make it an executable
```bash
cd <acados_root>/bin
chmod +x t_renderer
```

### 5) Run a Python example to check that everything works:
```bash
python3 <acados_root>/examples/acados_python/getting_started/minimal_example_ocp.py
```

If it runs and plots with no errors, you're done!

---

## Final Verification:
~~~bash
python - <<'PY'
import acados_template, pyJoules
print("acados_template: OK")
print("pyJoules: OK")
PY
~~~

# Papers and their repositories:
American Control Conference 2024 - [see paper here](https://coogan.ece.gatech.edu/papers/pdf/cuadrado2024tracking.pdf)  
[Personal Version of Repository](https://github.com/evannsm/MoralesCuadrado_ACC2024)  
[Official FACTSLab Repository](https://github.com/gtfactslab/MoralesCuadrado_Llanes_ACC2024)  

Transactions on Control Systems Technology 2025 - [see paper here](https://arxiv.org/abs/2508.14185)  
[Personal Version of Repository](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)  
[Official FACTSLab Repository](https://github.com/gtfactslab/Baird_MoralesCuadrado_TRO_2025)  

Transactions on Robotics 2025  
[Personal Version of Repository](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)  
[Official FACTSLab Repository](https://github.com/gtfactslab/MoralesCuadrado_Baird_TCST2025)  

# Works:
Repositories that hold other Newton-Raphson work  
[2025_NewtonRaphson_QuadrotorComplete](https://github.com/evannsm/2025_NewtonRaphson_QuadrotorComplete)    
[Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024](https://github.com/evannsm/Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024)  

