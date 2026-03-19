# Hand x Arm (SO-100 + Amazing Hand)

This project is about integrating the **SO-100 arm** with the **Amazing Hand** into one MuJoCo model and running simple motion + grasp demos.

---

## What this project is

The goal is to combine:

- **SO-100** (5-DOF arm)
- **Amazing Hand** (multi-finger hand)

into a single robot setup for:

- simulation in MuJoCo,
- testing coordinated arm + hand movement,
- and experimenting with basic pick-and-lift behavior.



<p align="center">
  <img src="assets/image.png" width="48%" />
  <img src="assets/pick.png" width="48%" height="240"/>
</p>

<p align="center">
  <em> Arm + Hand movement &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp; Pick-and-place demo</em>
</p>


---



## Project structure

- `so+hands/` → combined MuJoCo model and scene files
- `simu.py` → open the MuJoCo scene quickly
- `movement.py` → simple scripted arm + hand motion
- `pick_block.py` → basic pick/block grasp sequence demo
- `servo.py` → basic serial servo control test (hardware)

---

## Requirements

- Python 3.10+
- MuJoCo (`mujoco`)
- PySerial (`pyserial`)

### Setup (using uv)

**uv** is a fast Python package manager. Install it first:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then sync dependencies:

```bash
uv sync
```

### Setup (using pip)

Alternatively, use pip:

```bash
pip install mujoco pyserial
```

---

## How to use

### With uv (recommended)

**1) Run simple simulation viewer**

```bash
uv run simu.py
```

and similarly for other files

### With Python (pip)

If you used pip to install dependencies, run scripts directly:

```bash
python simu.py
python movement.py
python pick_block.py

```

---

## Reference links

- SO-100: https://github.com/TheRobotStudio/SO-ARM100
- Amazing Hand: <ADD_HAND_LINK_HERE>

---

## Notes

- Current scripts are basic demos intended for quick testing.
- You can tune grasp/motion behavior in `pick_block.py`.
- Main MuJoCo scenes are in `so+hands/scene.xml` and `so+hands/scene1.xml`.
