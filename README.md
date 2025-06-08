# RobotKinematicControl

A simple Python project for simulating humanoid robot kinematics using [PyBullet](https://pybullet.org/).

## Features

- Loads a humanoid model in PyBullet.
- Easily extensible for more advanced robot control experiments.

## Requirements

- Python 3.8+
- [pybullet](https://pypi.org/project/pybullet/)

## Setup

1. Clone this repository:
    ```sh
    git clone https://github.com/<your-username>/RobotKinematicControl.git
    cd RobotKinematicControl
    ```

2. (Optional) Create and activate a virtual environment:
    ```sh
    python -m venv venv
    venv\Scripts\activate
    ```

3. Install dependencies:
    ```sh
    pip install pybullet
    ```

## Usage

Run the simulation:
```sh
python humanoid.py
```

A PyBullet GUI window will open and the humanoid will start moving its legs.

## Notes

- Joint names may need to be updated in `humanoid.py` depending on the humanoid model used.