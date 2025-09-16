# MPCDrive

***MPCDrive*** is a project focused on controlling differntial drive robots using ***Model Predective control.***

# Project Structure
```
.
.
└── mpcDrive
    ├── config
    │   ├── gz_bridge.yaml
    │   ├── mapper.rviz
    │   ├── mappers_params_online_async.yaml
    │   ├── nav2_params.yaml
    │   └── twist_mux.yaml
    ├── description
    │   ├── gazebo_control.xacro
    │   ├── inertial_macros.xacro
    │   ├── robot_core2.xacro
    │   ├── robot_macros.xacro
    │   └── rover.urdf.xacro
    ├── launch
    │   ├── description.launch.py
    │   └── gzsim.launch.py
    ├── LICENSE
    ├── mpcDrive
    │   ├── __init__.py
    │   ├── mpc.py
    │   └── __pycache__
    │       ├── __init__.cpython-310.pyc
    │       └── mpc.cpython-310.pyc
    ├── package.xml
    ├── resource
    │   └── mpcDrive
    ├── setup.cfg
    ├── setup.py
    ├── test
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    └── worlds
        ├── empty.world
        └── random.sdf


```


# Pip install requirements :
```
pip install numpy cvxpy

```

[mpc.webm](https://github.com/user-attachments/assets/74c29d03-ef1a-46f0-8f87-9aadd4025bb2)

