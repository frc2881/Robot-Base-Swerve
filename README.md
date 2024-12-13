# Robot-Base-Swerve

[![PyFRC Tests](https://github.com/frc2881/Robot-Base-Swerve/actions/workflows/python-app.yml/badge.svg?branch=2025-dev)](https://github.com/frc2881/Robot-Base-Swerve/actions/workflows/python-app.yml)

## Installation & Deployment
* Follow the official documentation for installing Python in your development environment (if Python 3.12 is not already installed): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
* For Visual Studio Code users:
  * Open this project in Visual Studio Code and install the Python extensions from Microsoft: https://marketplace.visualstudio.com/items?itemName=ms-python.python
  * Open the Command Palette (Ctrl/Command-Shift-P) -> `Python: Create Environment` to create a local Python virtual environment (.venv) for the project using the installed Python 3.12 interpreter
* For PyCharm Community Edition users:
  * Open this project in PyCharm and follow the documented steps to create a local Python virtual environment found at: https://www.jetbrains.com/help/pycharm/creating-virtual-environment.html
    * Note: configure the virtual environment directory path with `.venv` as the root (as opposed to `venv`) to maintain compatibility between PyCharm and Visual Studio Code environments when needed
* After the Python virtual environment is created and ready in either Visual Studio Code or PyCharm, access and run the the install, build, and deployment tasks for the project as needed (Command Palette in Visual Studio Code and Run Configurations in PyCharm):
    * `RobotPy: Step 1 - Install RobotPy`
    * `RobotPy: Step 2 - Sync RobotPy`
    * `RobotPy: Step 3 - Test Robot Code (Optional)`
    * `RobotPy: Step 4 - Simulate Robot Code (Optional)`
    * `RobotPy: Step 5 - Deploy Robot Code`

## Project Notes & Status
* RobotPy API documentation and guides are available: https://robotpy.readthedocs.io/en/stable/index.html