# 2024-crescendo

## Code Layout and Structure
This year's robot is using the Commands-based framework of wpilib.  The robot code is organized mainly into subsystems, that define the physical components and how they operate, and commands that implement the logic and control of the subsystems.  The subsystems folder has files for each robot subsystem (like swerve, arm, shooter, etc.).  The commands folder holds files for various robot operations like autonomous routines, robot driving, and arm movements.

In robotcontainer.py the subsystems get instantiated and the commands are matched with various "triggers" such as button presses or when other events occur that require the robot to do something.

We also have a folder this year called FROGlib.  This folder will hold custom classes that we've created to use in subsystems.


## Laptop Setup

You'll need a laptop set up with Python, Git, and vscode as described [here](https://frog3160.github.io/setup)

NOTE:  Your VSCode installation needs the following extensions:
* Python Environment Manager by Don Jayamanne
* Python by Microsoft
* Pylance by Microsoft
* GitHub Pull Requests by GitHub

### Use VScode to create a new virtual environment
Open terminal and cd to the new repository location
run 
> py -3 -m venv .venv-crescendo

### Install the python packages into the new virtual environment
In vscode, use the Python extension to open a terminal in the new virtual environment and run the following:
> pip install -r requirements.txt


