
import json
import time
from pathlib import Path

from pymodbus.client import ModbusTcpClient


# ============================================================
# MODBUS CONFIGURATION
# ============================================================

HOST = "127.0.0.1"
PORT = 5020
SLAVE_ID = 1

# Output registers
RESET = 0
SET_X = 1
SET_Y = 2
VACUUM = 3
START_P1 = 4
START_P2 = 5

# Input/status registers
AT_X = 15
AT_Y = 16
SOURCE1_SENSOR = 17
SOURCE2_SENSOR = 18
P1_RUNNING = 19
P2_RUNNING = 20
P1_SENSOR = 21
P2_SENSOR = 22
BUTTON1 = 23
BUTTON2 = 24


# ============================================================
# CRANE SETTINGS
# ============================================================

SAFE_HEIGHT = 200
PICKUP_Y = 82
APPROACH_Y = 120

POSITION_TOLERANCE = 3
TIMEOUT_SECONDS = 15
POLL_INTERVAL = 0.2
VACUUM_SETTLE_TIME = 0.5

JSON_FILE = Path(__file__).with_name("grade_e_actions.json")


# ============================================================
# MODBUS HELPERS
# ============================================================

def read_register(client, address):
    """Read one holding register and check for errors."""
    result = client.read_holding_registers(
        address=address,
        count=1,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Could not read register {address}."
        )

    return result.registers[0]


def write_register(client, address, value):
    """Write one holding register and check for errors."""
    result = client.write_register(
        address=address,
        value=value,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Could not write value {value} "
            f"to register {address}."
        )


def wait_for_position(client, target_x, target_y):
    """Wait until the crane reaches its target coordinates."""
    deadline = time.monotonic() + TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)

        x_reached = (
            abs(current_x - target_x) <= POSITION_TOLERANCE
        )
        y_reached = (
            abs(current_y - target_y) <= POSITION_TOLERANCE
        )

        if x_reached and y_reached:
            return

        time.sleep(POLL_INTERVAL)

    current_x = read_register(client, AT_X)
    current_y = read_register(client, AT_Y)

    raise TimeoutError(
        f"Crane did not reach X={target_x}, Y={target_y} "
        f"within {TIMEOUT_SECONDS} seconds. "
        f"Actual position: X={current_x}, Y={current_y}."
    )


# ============================================================
# SAFE CRANE MOVEMENT
# ============================================================

def move_to(client, target_x, target_y):
    """
    Move horizontally only at safe height.
    Always command the final Y coordinate when it differs
    from the current Y coordinate, including SAFE_HEIGHT.
    """
    current_x = read_register(client, AT_X)
    current_y = read_register(client, AT_Y)

    # Raise to safe height before horizontal movement.
    if target_x != current_x and current_y != SAFE_HEIGHT:
        write_register(client, SET_Y, SAFE_HEIGHT)
        wait_for_position(client, current_x, SAFE_HEIGHT)
        current_y = SAFE_HEIGHT

    # Move horizontally at safe height.
    if target_x != current_x:
        if current_y != SAFE_HEIGHT:
            raise RuntimeError(
                "Safety check failed: horizontal movement "
                "was requested below safe height."
            )

        write_register(client, SET_X, target_x)
        wait_for_position(client, target_x, SAFE_HEIGHT)

        current_x = target_x
        current_y = SAFE_HEIGHT

    # Move vertically to the requested target.
    # This also handles the final move from Y=82 to Y=200.
    if target_y != current_y:
        write_register(client, SET_Y, target_y)
        wait_for_position(client, target_x, target_y)


# ============================================================
# JSON ACTION VALIDATION
# ============================================================

def validate_actions(actions):
    """Validate the structure and values of the JSON actions."""
    if not isinstance(actions, list) or not actions:
        raise ValueError(
            'JSON must contain a non-empty "actions" list.'
        )

    allowed_keys = {"setX", "setY", "vacuum"}

    for number, action in enumerate(actions, start=1):
        if not isinstance(action, dict) or not action:
            raise ValueError(
                f"Action {number} must be a non-empty object."
            )

        if not set(action).issubset(allowed_keys):
            raise ValueError(
                f"Action {number} contains an unsupported command."
            )

        if "vacuum" in action:
            value = action["vacuum"]

            if type(value) is not int or value not in (0, 1):
                raise ValueError(
                    f"Action {number}: vacuum must be integer 0 or 1."
                )

        for key in ("setX", "setY"):
            if key in action:
                value = action[key]

                if type(value) is not int or value < 0:
                    raise ValueError(
                        f"Action {number}: {key} must be "
                        "a non-negative integer."
                    )

    if actions[0] != {"vacuum": 0}:
        raise ValueError(
            'The first action must be {"vacuum": 0}.'
        )

    if len(actions) < 2 or actions[1] != {
        "setX": 500,
        "setY": SAFE_HEIGHT,
    }:
        raise ValueError(
            "The second action must target X=500, Y=200."
        )

    # Require the final action to raise the crane at the Sink.
    if actions[-1] != {"setX": 945, "setY": SAFE_HEIGHT}:
        raise ValueError(
            "The final JSON action must target X=945, Y=200."
        )

    # Ensure a vacuum-off command exists.
    if not any(action.get("vacuum") == 0 for action in actions):
        raise ValueError(
            "The action sequence must turn the vacuum off."
        )


# ============================================================
# JSON ACTION EXECUTION
# ============================================================

def execute_actions(client, actions):
    """Execute each movement and vacuum command from JSON."""
    for number, action in enumerate(actions, start=1):
        print(f"\nAction {number}/{len(actions)}: {action}")

        if "setX" in action or "setY" in action:
            current_x = read_register(client, AT_X)
            current_y = read_register(client, AT_Y)

            target_x = action.get("setX", current_x)
            target_y = action.get("setY", current_y)

            move_to(client, target_x, target_y)

            print(
                "Position reached: "
                f"X={read_register(client, AT_X)}, "
                f"Y={read_register(client, AT_Y)}"
            )

        if "vacuum" in action:
            vacuum_value = action["vacuum"]

            write_register(client, VACUUM, vacuum_value)
            time.sleep(VACUUM_SETTLE_TIME)

            actual_vacuum = read_register(client, VACUUM)

            if actual_vacuum != vacuum_value:
                raise RuntimeError(
                    f"Vacuum verification failed: expected "
                    f"{vacuum_value}, got {actual_vacuum}."
                )

            print(f"Vacuum set to {actual_vacuum}.")


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():
    if not JSON_FILE.exists():
        raise FileNotFoundError(
            f"JSON file not found: {JSON_FILE}"
        )

    with JSON_FILE.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict) or "actions" not in data:
        raise ValueError(
            'JSON must contain an "actions" property.'
        )

    actions = data["actions"]
    validate_actions(actions)

    client = ModbusTcpClient(HOST, port=PORT)

    try:
        if not client.connect():
            raise ConnectionError(
                f"Could not connect to the simulation at "
                f"{HOST}:{PORT}."
            )

        print("Connected to the crane simulation.")

        # Check startup conditions before any movement.
        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)
        current_vacuum = read_register(client, VACUUM)
        source1 = read_register(client, SOURCE1_SENSOR)
        p1_running = read_register(client, P1_RUNNING)
        p1_sensor = read_register(client, P1_SENSOR)

        print("\nInitial state:")
        print(f"Crane X: {current_x}")
        print(f"Crane Y: {current_y}")
        print(f"Vacuum: {current_vacuum}")
        print(f"Source 1 sensor: {source1}")
        print(f"Process 1 running: {p1_running}")
        print(f"Process 1 sensor: {p1_sensor}")

        if (current_x, current_y) != (500, 150):
            raise RuntimeError(
                "Expected starting position X=500, Y=150. "
                "No actions were executed."
            )

        if current_vacuum != 0:
            raise RuntimeError(
                "Vacuum must be OFF before starting."
            )

        if source1 != 1:
            raise RuntimeError(
                "Source 1 must contain a component before starting."
            )

        if p1_running != 0 or p1_sensor != 0:
            raise RuntimeError(
                "Process 1 must be idle and empty before starting."
            )

        print(f"\nLoaded {len(actions)} JSON actions.")
        print("Starting Grade E action sequence...")

        execute_actions(client, actions)

        # Verify final state.
        final_x = read_register(client, AT_X)
        final_y = read_register(client, AT_Y)
        final_vacuum = read_register(client, VACUUM)
        final_source1 = read_register(client, SOURCE1_SENSOR)
        final_p1_running = read_register(client, P1_RUNNING)
        final_p1_sensor = read_register(client, P1_SENSOR)

        print("\nFinal state:")
        print(f"Crane X: {final_x}")
        print(f"Crane Y: {final_y}")
        print(f"Vacuum: {final_vacuum}")
        print(f"Source 1 sensor: {final_source1}")
        print(f"Process 1 running: {final_p1_running}")
        print(f"Process 1 sensor: {final_p1_sensor}")

        problems = []

        if abs(final_x - 945) > POSITION_TOLERANCE:
            problems.append("Crane did not finish at Sink X=945.")

        if abs(final_y - SAFE_HEIGHT) > POSITION_TOLERANCE:
            problems.append(
                "Crane did not finish at safe height Y=200."
            )

        if final_vacuum != 0:
            problems.append("Vacuum is still ON.")

        if final_source1 != 0:
            problems.append(
                "Source 1 sensor still detects a component."
            )

        if final_p1_running != 0:
            problems.append("Process 1 is still running.")

        if final_p1_sensor != 0:
            problems.append(
                "Process 1 is not empty after the sequence."
            )

        if problems:
            print("\nGRADE E VERIFICATION FAILED:")
            for problem in problems:
                print(f"- {problem}")

            raise RuntimeError(
                "The final state did not satisfy all checks."
            )

        print("\nGRADE E VERIFICATION PASSED.")
        print(
            "The sequence finished at the Sink, "
            "at safe height, with the vacuum OFF."
        )

    except Exception as error:
        print(f"\nController stopped: {error}")
        print(
            "Inspect the simulation before restarting. "
            "The controller will not automatically reset "
            "the simulation or generate a component."
        )

    finally:
        client.close()
        print("Modbus connection closed.")


if __name__ == "__main__":
    main()
