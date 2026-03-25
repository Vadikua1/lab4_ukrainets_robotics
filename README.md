
# Lab 4: Dead Reckoning


## Installation & Setup

1.  **Clone the repository**:
    
    ```    
    git clone https://github.com/Vadikua1/lab4_ukrainets_robotics.git
    ```
2. **Build and run docker**:
	 ```
	./scripts/cmd build-docker
	./scripts/cmd run 
	```
3.  **Build the package (in 2 terminals)**:
    
    
    ```
    cd /opt/ws
    rm -rf build/ install/ log/
    colcon build --packages-select lab3 lab4
    source install/setup.bash
    source /opt/ros/jazzy/setup.bash
    
    ```
    

----------

## How to Run

## 1. Launch Simulation and Visualization

Launch the TurtleBot3 world and the bridge:


``` bash
ros2 launch lab4 dead_reckoning_bringup.launch.py

```

## 2. Execute Movement

In a new terminal, run the circle trajectory script:


```
ros2 run lab3 circle_path

```