# Installation

## Install with conda (Linux)

### Basic Env

> ***Note**: This will only install the basic module. For other algorithm submodules, please follow the instructions [Algorithm Submodule Env](#algorithm-submodule-env) to install as needed.*

1. Pull the repository and update the submodule
```
cd /home/$(whoami)
git clone https://github.com/yding25/BestMan_Xarm.git
```

2. Install xArm SDK
```
cd ~
git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
python3 setup.py install
```

3. Run the following script to add the project to the PYTHON search path

```
/home/$(whoami)/BestMan_Ｘarm/Install
chmod 777 pythonpath.sh
bash pythonpath.sh
source ~/.bashrc
```

4. Create basic conda environment
```
cd /home/$(whoami)/BestMan_Xarm/Install
conda env create -f basic_environment.yaml
```

5. Install ROS environment (optional)
```
cd /home/$(whoami)/BestMan_Flexiv/Install
chmod 777 install_ros_noetic.sh
bash install_ros_noetic.sh
source ~/.bashrc
```

6. Check ROS environment (optional)
```
roscore
echo $ROS_DISTRO
```
