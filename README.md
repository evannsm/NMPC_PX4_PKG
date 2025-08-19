# NMPC_PX4_PKG

## Prerequisites

Install the following:
- **pyJoules** (Python energy/power measurement)
- **ACADOS** (built from source with shared libraries)
- **ACADOS Python interface** (`acados_template`)

---

## Install Steps

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

### 3) Install the ACADOS Python interface as per the [python interface instructions](https://docs.acados.org/python_interface/index.html)
1. Install template
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
3. Strip the name of everything after ```t_renderer``` : (e.g. ```t_renderer-v0.2.0-linux-arm64 -> t_renderer```)
4. Make it an executable
```bash
cd <acados_root>/bin
chmod +x t_renderer
```
---

## Verify
~~~bash
python - <<'PY'
import acados_template, pyJoules
print("acados_template: OK")
print("pyJoules: OK")
PY
~~~
