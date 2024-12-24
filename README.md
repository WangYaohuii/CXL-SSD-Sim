# CXL-SSD-Sim

## Introduction

CXL-SSD-Sim is a CXL-based Solid State Drive (SSD) simulator that is developed on top of the open-source platforms gem5 and SimpleSSD. For users looking to understand how to configure and run simulations with gem5, which forms a critical part of the CXL-SSD-Sim environment, it is recommended to consult the [Documentation](http://www.gem5.org/Documentation) and [Tutorials](http://www.gem5.org/Tutorials). This tutorial provides comprehensive guidance on setting up the simulation environment and running simulations, all of which are essential skills for effectively utilizing CXL-SSD-Sim.

## Requirements

- Setup on Ubuntu 20.04 or 22.04

```bash
sudo apt update
sudo apt install vim

sudo apt install build-essential git m4 scons zlib1g zlib1g-dev \
  libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
  python3-dev python-is-python3 libboost-all-dev pkg-config libhdf5-dev libpng-dev
```

## Building CXL-SSD-Sim

- Linux Kernel: In full-system mode, gem5 boots the operating system by loading a compiled Linux Kernel, which includes necessary resource management modules and device drivers.  We provide a Linux Kernel file with added drivers for CXL devices, version 5.4.49.  The CXL device driver provides an mmap system call to allocate CXL memory.

- Disk Image: In full-system mode, the simulator relies on the Disk Image to run the simulation programs.   Users can copy the executable programs to be tested into the Disk Image for execution after the full system starts.   We provide a Disk Image that includes the LMbench, Stream, Viper, and Meric test programs. 

  - Acquiring the Disk Image:  https://drive.google.com/file/d/1I911l3X3PFzrx-sb_HqOGcRiK7py6Kzh/view?usp=sharing

  - Place the image file in the `CXL-SSD-Sim/full-system-image/disks` directory. 

- Start the compilation process by entering the root directory of CXL-SSD-Sim

  CXL-SSD-Sim currently only supports modeling of the x86 architecture.

  Type the following command to build the simulator:

  ```
  cd CXL-SSD-Sim
  scons build/X86/gem5.opt -j`nproc`
  ```

​	The `-j` flag is optional and allows for parallelization of compilation with `nproc` specifying the number of threads.
​	This will compile and generate the `build/X86/gem5.opt` executable for the X86 architecture.

## Start Full-System Simulation 

- Enter the CXL-SSD-Sim Root Directory

You can refer to the following command template to start a full-system simulation.  Please adjust the paths and options according to your specific setup and requirements:

```
build/X86/gem5.opt configs/example/fs.py --kernel full-system-image/kernel/vmlinux-5.4.49 --disk-image  full-system-image/disks/parsec.img  --num-cpus=1 --cpu-type=X86TimingSimpleCPU 
```

For more configurable system component parameters, please refer to the [gem5 standard library](https://www.gem5.org/documentation/gem5-stdlib/overview). 

- Install `m5term`.

`m5term` allows users to connect to the simulated terminal provided by the full system. 

While waiting for the terminal output, we can open a new termicnal and navigate to the CXL-SSD-Sim/util/term directory, enter the following command:  

```
make
sudo install -o root -m 555 m5term /usr/local/bin
```

- After starting the full system, enter the terminal with the following command:

```
m5term localhost 3456
```



