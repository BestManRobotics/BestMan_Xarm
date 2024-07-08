# BestMan_Xarm
<!-- <img src="docs/BestMan_logo.png" alt="BestMan Logo" width="700"/> -->

<!-- # BestMan_Flexiv - A Pybullet-based Mobile Manipulator Simulator -->
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/facebookresearch/home-robot/blob/main/LICENSE)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3.8](https://img.shields.io/badge/python-3.8-blue.svg)](https://www.python.org/downloads/release/python-370/)
<!-- [![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/) -->
<!-- [![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Imports: isort](https://img.shields.io/badge/%20imports-isort-%231674b1?style=flat)](https://timothycrosley.github.io/isort/) -->

Welcome to the BestMan_Xarm repository, a codebase dedicated to the XArm 6 robotic arm.


## 💻 Installation

- (Optional) Create conda environment

    ```
    conda env create -f basic_environment.yaml
    ```

- Install xArm-Python-SDK

    ```bash
    cd ~
    git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
    python3 setup.py install
    ```

- Clone the BestMan_Xarm Repository

    ```bash
    git clone https://github.com/yding25/BestMan_Xarm.git
    ```



## 🔎 Project Structure

Generate and view the project structure:

```bash
doxygen Doxyfile
firefox /home/$(whoami)/BestMan_Xarm/docs/html/index.html # API document
```

## 👨‍💻 Basic Demos

```bash
cd ~/BestMan_Xarm/Examples/
# Demo: Go Home
python3 move_arm_to_home.py 192.168.1.208

# Demo: Move Joints
python3 move_arm_to_joint_angles.py 192.168.1.208

# Demo: Move gripper
python3 close_gripper.py 192.168.1.208
python3 open_gripper.py 192.168.1.208
python3 open_gripper_width.py 192.168.1.208
```

(We are fixing other demos.)


## 📧 Contact Information

If you have any questions or need further assistance, please feel free to reach out via email:
- dingyan at pjlab.org.cn
- zhaxizhuoma at pjlab.org.cn
- wuziniu at pjlab.org.cn

##  :handshake: Reference
- [IKPy’s documentation](https://ikpy.readthedocs.io/en/latest/index.html)
- [xArm-Python-SDK ](https://github.com/xArm-Developer/xArm-Python-SDK)
