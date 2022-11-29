# AutoCRAFT: Layout Automation for <ins>C</ins>ustom Ci<ins>r</ins>cuits in <ins>A</ins>dvanced <ins>F</ins>inFET <ins>T</ins>echnologies

AutoCRAFT is a semi-automated custom layout framework to improve the productivity of analog and high-speed custom digital circuit layout implementation.
AutoCRAFT has been developed and tested on x64 linux machines.

## Modules Overview
```
AutoCRAFT
|---- Circuit
|     |---- Placement grid
|     |---- Routing grid
|     |---- Device mapping
|     |---- Primitive lib
|     |---- Region
|     |---- Cell
|     |     |---- Pin
|     |     |---- Obs (obstacle)
|     |---- Net
|     |     |---- Pin
|     |     |---- Via
|     |     |---- Wire
|     |     |---- TopoTree
|     |     |---- Routable
|     |---- Layer
|     |     |---- Metal layer
|     |     |---- Cut layer
|     |---- Cstr (Constraint)
|           |---- Placement cstr
|           |     |---- Net weighting
|           |     |---- Array
|           |     |---- Alignment (row/col)
|           |     |---- Cluster
|           |     |---- Disjoint (row/col)
|           |     |---- Edge distance (to left/right/top/bottom boundaries)
|           |     |---- Extension
|           |     |---- Row (odd/even)
|           |     |---- Symmetry
|           |     |---- Pre-placed cell
|           |---- Routing cstr
|                 |---- Symmetry
|                 |---- Path matching (developing)
|---- Placer
|     |---- SMT placer
|---- Router
|     |---- Global routing manager (deprecated)
|     |---- Detailed routing manager
|     |     |---- Path searching detailed router
|     |---- Post routing
|---- I/O
      |---- Parser
      |     |---- LEF primitive templates parser
      |     |---- Hspice netlist parser
      |     |---- Grid file parser
      |     |---- Constraint file parser
      |---- Writer
            |---- GV writer (.gv files for ICC2)
            |---- GDSII writer (deprecated, now use ICC2 for final layout)
            |---- Tcl writer (ICC2 Tcl commands)
```

## Dependencies
* [CMake](https://cmake.org) >= 3.14.0
* [pybind11](https://github.com/pybind/pybind11) >= 2.7.0 (Python 3.7/3.8)
* [Boost](https://www.boost.org/) >= 1.6
* [spdlog](https://github.com/gabime/spdlog)
* [Limbo](https://github.com/limbo018/Limbo)
* [phmap](https://github.com/greg7mdp/parallel-hashmap)
* [z3](https://github.com/Z3Prover/z3) >= 4.8.11
* [nanoflann](https://github.com/jlblancoc/nanoflann)
* [lemon](https://lemon.cs.elte.hu/trac/lemon) >= 1.3.1
* [zlib](https://github.com/madler/zlib)
* [cxxopts](https://github.com/jarro2783/cxxopts) == 2.2.0
* [lef/def](https://si2.org/oa-tools-utils-libs/) >= 5.8

## Installation

Recommended GCC version: >= 8.3.0 (Tested on 8.3.0)

```sh
mkdir -p cpp/build/ && cd cpp/build/
cmake .. -DPYTHON_EXECUTABLE=<path to python> -DPYTHON_LIBRARY=<path to python lib>
make
```
A binary executable `autocraftE` and a python library `autocraft.cpython-3x-x86_64-linux-gnu.so` will be generated after compilation. Copy the file to your python site-packages path by

```sh
cp autocraft.cpython-3x-x86_64-linux-gnu.so <your site-packages path>
```

or put it under the path with the files importing `autocraft`.


## Usage

Using the Python interface is recommended.
The binary executable `autocraftE` can still be used for debugging purpose. Run
```sh
autocraftE [OPTION...]
    --laygo_config=<file>       # laygo configuration (.yml)
    --tech_grid=<file>          # placement anf routing grids definition (.yml)
    --tech_map=<file>           # device-to-primitive mapping (.yml)
    --constraint=<file>         # constraint specification
    --lef=<files>               # primitive and via templates (.lef)
    --netlist=<file>            # spice netlist (.cir)
    --drc=<file>                # abstracted drc rules (.yml)
    --in_place=<file>           # placement input (optional)
    --out_place=<file>          # cells location after placement (optional)
    --out_gv=<file>             # gated verilog netlist output (optional)
    --out_tcl=<file>            # ICC2 Tcl commands output (optional)
    --threads=<int>             # number of CPU threads for placement (optional, not necessarily faster)
    --place_ar=<float>          # user-defined aspect ratio of the design area
    --place_ur=<float>          # user-defined utilization ratio of the design
    --place_ur_reg_name=<strs>  # utilization ratio for some regions (should be used together with --place_ur_reg_val)
    --place_ur_reg_val=<floats> # utilization ratio values for some regions
    --place_bbox=<ints>         # design bounding box
    --place_iter=<int>          # placemnt WL optimization iterations
-h, --help                      # print usage

Example: 
autocraftE --laygo_config=laygo_config.yaml \
           --tech_grid=dv5t_crg_fll_ana_obsd.grid.yaml \
           --tech_map=dv5t_crg_fll_ana_obsd.map.yaml \
           --constraint=dv5t_crg_fll_ana_obsd.constraint.yaml \
           --lef=dv5t_crg_fll_ana_obsd.template.lef,dv5t_crg_fll_ana_obsd.via.lef \
           --netlist=dv5t_crg_fll_ana_obsd.cir \
           --drc=layers.drc.yaml \
           --threads=1 \
           --place_ar=1.1 \
           --place_ur=0.4 \
           --place_ur_reg_name=region1 \
           --place_ur_reg_val=0.4 \
           --place_iter=2 \
           --out_gv=dv5t_crg_fll_ana_obsd.gv \
           --out_tcl=dv5t_crg_fll_ana_obsd_pnr.tcl
```
to execute the program

