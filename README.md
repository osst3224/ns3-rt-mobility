# RT-enabled ns-3 with NVIDIA Sionna and Advanced Vehicular Mobility

This repository is an extension of the original *RT-enabled ns-3 with NVIDIA Sionna* framework developed by Roberto Pegurri *et al.* (https://arxiv.org/abs/2501.00372).  
It integrates ray tracing–based wireless channel simulation with a user-friendly and realistic vehicular mobility framework, enabling the generation of high-fidelity datasets for machine learning–based predictive quality of service (pQoS) analysis.

---

## Overview

This project builds upon the original integration of **NVIDIA Sionna RT** into **ns-3**, which enables **GPU-accelerated, deterministic ray tracing** for wireless channel simulation in complex environments.

This work extends the original work by:
- Introducing a **user-friendly mobility framework** tailored for vehicular scenarios
- Enabling **realistic vehicle motion**, trajectories, and heterogeneous behaviors
- Providing **detailed and heterogeneous simulation scenarios** suitable for **machine-learning dataset generation**, with a focus on **predictive QoS (pQoS)**

The framework is particularly well suited for:
- Vehicular and urban network simulations
- Digital twin–inspired wireless system studies
- Data generation for ML-driven network optimization

---

## Installation

The framework consists of two interacting components: **ns-3** and **Sionna**, which communicate via a UDP socket.

### 1. Installing ns3-rt (this repository)

```bash
git clone https://github.com/osst3224/ns3-rt-mobility.git
cd ns3-rt
./ns3 configure --disable-python --enable-examples
./ns3 build
```


### 2. Installing Sionna RT

Sionna RT v1.0.1 and later has the same requiremets as Mitsuba 3 [available at this page](https://mitsuba.readthedocs.io/en/stable/). Detailed instructions are available in the official Sionna RT installation guide. Ubuntu 22.04 is recommended.

**If you are running Sionna RT on a GPU**, install the required drivers (refer to your GPU vendor). After the drivers are properly setup, TensorFlow GPU can be installed with:

```bash
python3 -m pip install 'tensorflow[and-cuda]' 
```

and verify the installation with:

```bash
python3 -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"
```

**If you are running Sionna RT on a CPU**, install TensorFlow and LLVM (also required in this case) with:

```bash
sudo apt install llvm
python3 -m pip install tensorflow 
```

and verify the installation with:

```bash
python3 -c "import tensorflow as tf; print(tf.reduce_sum(tf.random.normal([1000, 1000])))"
```

In case of any issues with TensorFlow or TensorFlow GPU, please refer to the official installation guide [at this page](https://www.tensorflow.org/install).

**At this point, install Sionna RT** with:

```bash
pip install sionna-rt
```

## Running the example

To run the `simple-sionna-example` example, you first need to start Sionna RT. It is recommended to run Sionna remotely using a GPU, which is expected by this instruction.

Run the Python example script `sionna_server_script.py` from the `/src/sionna` folder with the following command and options:

```bash
python3 'sionna_server_script.py' --frequency=2.1e9
```

After Sionna RT has started, run the ns-3 simulation in parallel with:

```bash
./ns3 run simple-sionna-example ADDRESS HOUR
```

Here, replace "ADDRES" with your local IP-address where you are running Sionna, and "HOUR" with the hour (in two digits) of the day that you want to run the simulation.

## Simulating with Sionna RT and ns-3 locally

ns3-rt was created with the possibility to run Sionna RT both locally (on the same machine with ns-3) and remotely (in a server with). To run the simulation locally, run Sionna with:

```bash
python3 'sionna_server_script.py' --local-machine --frequency=2.1e9
```

and set the "local_machine" flag in the "simple-sionna-example.cc" to True.