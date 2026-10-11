# Installation

bteng-nav2 requires **Python 3.10 or newer** and a sourced **ROS 2 Humble, Iron, or Jazzy** workspace.

---

## Prerequisites

Install and source ROS 2 before installing bteng-nav2. The package imports `rclpy`, `nav2_msgs`, and other ROS 2 Python packages at runtime, so those must be on your `PYTHONPATH`.

```bash
# Humble example — adjust the distro name as needed
source /opt/ros/humble/setup.bash
```

Verify the ROS environment is active:

```bash
python3 -c "import rclpy; print('rclpy ok')"
```

---

## Standard install

With ROS 2 sourced:

```bash
pip install bteng-nav2
```

This pulls in `bteng >= 0.3.1` and `bteng-ros2 >= 0.2.3`. Both pins are load-bearing rather than
cosmetic: `bteng-ros2` 0.2.3 provides `RosBTExecutor.run()`, its `ros_node=` argument, the
server-discovery wait every service node needs on a real graph, and the endpoint ports described in
[Retargeting endpoints](../guide/endpoints.md).

Verify the install:

```bash
python3 -c "import bteng_nav2; print(bteng_nav2.__version__)"
bteng-nav2 nodes | head            # the CLI is installed as an entry point
```

---

## Virtual environment (recommended)

Use a virtual environment to avoid polluting the system Python. When creating the environment, pass `--system-site-packages` so that `rclpy` and the ROS 2 message packages remain accessible:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

pip install bteng-nav2
```

---

## Development install (from source)

If you are contributing to bteng-nav2 or want to run the test suite:

```bash
git clone https://github.com/mdirzpr/BTEng_nav2.git
cd BTEng_nav2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

pip install -e ".[dev]"
pytest -q
```

The `-e` flag installs in editable mode — changes to the source are reflected immediately without reinstalling.

### Running tests

The test suite runs without a live ROS 2 instance. The bundled `conftest.py` stubs all ROS 2 message and service types.

```bash
# Full test suite
pytest tests/ -v

# Individual files
pytest tests/test_actions.py
pytest tests/test_services.py
pytest tests/test_conditions.py
```

All tests should pass with or without ROS 2 installed.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `ModuleNotFoundError: No module named 'rclpy'` | Source your ROS 2 workspace: `source /opt/ros/humble/setup.bash` |
| `ModuleNotFoundError: No module named 'bteng_nav2'` | Activate the virtual environment: `source .venv/bin/activate` |
| `ModuleNotFoundError: No module named 'nav2_msgs'` | Install the Nav2 packages for your distro: `sudo apt install ros-humble-nav2-msgs` |
| Old version after `pip install` | Run `pip install --upgrade bteng-nav2` |

Next: [Quick Start](quickstart.md).
