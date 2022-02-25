[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

quadrotor_mpc_acados
=====================

# Description
This package implements different MPC controllers for planar and 3D quadrotors. The MPC controllers were designed using acados, which is a set of embedded solvers for nonlinear optimal control.

## Installation
1. Clone the acados reposetory
```bash
git clone https://github.com/acados/acados
```

2. Switch to version 0.1.6
```bash
cd <acados_root_folder>
git checkout tags/0.1.6
```

3. Go inside the <acados_root_folder> and initialze all submodules
```bash
git submodule update --recursive --init
```

4. Build and install acados using CMake while still inside the <acados_root_folder>
```bash
mkdir -p build
cd build 
cmake .. -DACADOS_WITH_QPOASES=ON
make install
```

5. Install the Python Interface with pip3
```bash
pip3 install -e <acados_root_foler>/interfaces/acados_template
```

6. Add the path the compiled shared libraries
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```

7. In order to be able to successfully render C code templates, you need to download the t_renderer binaries for your platform from
   https://github.com/acados/tera_renderer/releases/ and place them in <acados_root_folder>/bin (please strip the version and platform from the binaries (e.g. ```t_renderer-v0.0.34 -> t_renderer```). Notice that you might need to make ```t_renderer``` executable. Run ```export ACADOS_SOURCE_DIR=<acados_root_folder>``` such that the location of acados will be known to the Python package at run time.

8. Change the path to acados installation folder in each files in the models folder.
