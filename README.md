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
```bash
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

### 3) Install the ACADOS Python interface
~~~bash
# assuming ACADOS is cloned at $ACADOS_SOURCE_DIR
cd "$ACADOS_SOURCE_DIR/interfaces/acados_template"
pip install -e .
~~~

### 4) Set environment variables
Add these to your shell init (e.g., `~/.bashrc`), then `source ~/.bashrc`:
~~~bash
export ACADOS_SOURCE_DIR=/home/<your-user>/acados
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib"
~~~

---

## Verify
~~~bash
python - <<'PY'
import acados_template, pyJoules
print("acados_template: OK")
print("pyJoules: OK")
PY
~~~
