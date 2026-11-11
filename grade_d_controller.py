
import time

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
# CRANE POSITIONS
# ============================================================

SOURCE1_X = 55
PROCESS1_X = 450
SINK_X = 945

PICKUP_Y = 82
APPROACH_Y = 120
SAFE_HEIGHT = 200


# ============================================================
# TIMING SETTINGS
# ============================================================

POSITION_TOLERANCE = 3
MOVE_TIMEOUT = 15
PROCESS_START_TIMEOUT = 5
PROCESS_TIMEOUT = 10
POLL_INTERVAL = 0.2
VACUUM_SETTLE_TIME = 0.5


# ============================================================
# MODBUS HELPERS
# ============================================================

def read_register(client, address):
    """Read one holding register."""
    result = client.read_holding_registers(
        address=address,
        count=1,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Failed to read register {address}."
        )

    return result.registers[0]


def write_register(client, address, value):
    """Write one holding register."""
    result = client.write_register(
        address=address,
        value=value,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Failed to write {value} to register {address}."
        )


# ============================================================
# CRANE MOVEMENT
# ============================================================

def wait_for_position(client, target_x, target_y):
    """Wait until the crane reaches its target position."""
    deadline = time.monotonic() + MOVE_TIMEOUT

    while time.monotonic() < deadline:
        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)

        print(
            f"Crane: X={current_x}, Y={current_y} | "
            f"Target: X={target_x}, Y={target_y}"
        )

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
        f"within {MOVE_TIMEOUT} seconds. "
        f"Actual position: X={current_x}, Y={current_y}."
    )


def move_to(client, target_x, target_y):
    """
    Raise to safe height before horizontal travel.
    Then move vertically to the requested target.

    Important: the final vertical command is also sent when
    the target is SAFE_HEIGHT and the crane is below it.
    """
    current_x = read_register(client, AT_X)
    current_y = read_register(client, AT_Y)

    # Raise before horizontal movement.
    if target_x != current_x and current_y != SAFE_HEIGHT:
        write_register(client, SET_Y, SAFE_HEIGHT)
        wait_for_position(client, current_x, SAFE_HEIGHT)
        current_y = SAFE_HEIGHT

    # Horizontal movement is allowed only at safe height.
    if target_x != current_x:
        if current_y != SAFE_HEIGHT:
            raise RuntimeError(
                "Safety check failed: horizontal movement "
                "requested below safe height."
            )

        write_register(client, SET_X, target_x)
        wait_for_position(client, target_x, SAFE_HEIGHT)

        current_x = target_x
        current_y = SAFE_HEIGHT

    # Corrected logic: move vertically whenever the current
    # Y differs from the requested Y, including Y=200.
    if target_y != current_y:
        write_register(client, SET_Y, target_y)
        wait_for_position(client, target_x, target_y)


# ============================================================
# SOURCE 1 PICKUP
# ============================================================

def pick_from_source1(client):
    """Pick up one component from Source 1."""
    print("\nPicking up a component from Source 1...")

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum must be OFF before picking up a component."
        )

    if read_register(client, SOURCE1_SENSOR) != 1:
        raise RuntimeError(
            "No component detected at Source 1."
        )

    move_to(client, SOURCE1_X, SAFE_HEIGHT)
    move_to(client, SOURCE1_X, APPROACH_Y)
    move_to(client, SOURCE1_X, PICKUP_Y)

    write_register(client, VACUUM, 1)
    time.sleep(VACUUM_SETTLE_TIME)

    # Confirm that the source no longer detects the component.
    if read_register(client, SOURCE1_SENSOR) != 0:
        raise RuntimeError(
            "Source 1 still detects the component after pickup. "
            "Inspect the simulation before continuing."
        )

    move_to(client, SOURCE1_X, APPROACH_Y)
    move_to(client, SOURCE1_X, SAFE_HEIGHT)

    print("Component picked up from Source 1.")


# ============================================================
# PLACE COMPONENT AT PROCESS 1
# ============================================================

def place_at_process1(client):
    """Place the component at Process 1."""
    print("\nMoving component to Process 1...")

    if read_register(client, VACUUM) != 1:
        raise RuntimeError(
            "Vacuum is OFF; the crane may not be holding a component."
        )

    if read_register(client, P1_SENSOR) != 0:
        raise RuntimeError(
            "Process 1 is occupied. Refusing to place another component."
        )

    move_to(client, PROCESS1_X, SAFE_HEIGHT)
    move_to(client, PROCESS1_X, APPROACH_Y)
    move_to(client, PROCESS1_X, PICKUP_Y)

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, P1_SENSOR) != 1:
        raise RuntimeError(
            "Process 1 sensor did not detect the placed component. "
            "Inspect the simulation before continuing."
        )

    move_to(client, PROCESS1_X, APPROACH_Y)
    move_to(client, PROCESS1_X, SAFE_HEIGHT)

    print("Component placed at Process 1.")


# ============================================================
# PROCESS 1 CONTROL
# ============================================================

def start_and_wait_for_process1(client):
    """Start Process 1 and wait for processing to finish."""
    print("\nStarting Process 1...")

    if read_register(client, P1_SENSOR) != 1:
        raise RuntimeError(
            "Process 1 cannot start: no component detected."
        )

    if read_register(client, P1_RUNNING) != 0:
        raise RuntimeError(
            "Process 1 is already running."
        )

    # Issue the process-start command.
    write_register(client, START_P1, 1)

    # Wait for the process to report that it has started.
    start_deadline = (
        time.monotonic() + PROCESS_START_TIMEOUT
    )

    while time.monotonic() < start_deadline:
        if read_register(client, P1_RUNNING) == 1:
            print("Process 1 is running.")
            break

        time.sleep(POLL_INTERVAL)

    else:
        raise TimeoutError(
            "Process 1 did not report that it started."
        )

    # Wait for processing to finish.
    process_deadline = time.monotonic() + PROCESS_TIMEOUT

    while time.monotonic() < process_deadline:
        if read_register(client, P1_RUNNING) == 0:
            print("Process 1 has finished.")
            return

        print("Process 1 is still running...")
        time.sleep(POLL_INTERVAL)

    raise TimeoutError(
        f"Process 1 did not finish within {PROCESS_TIMEOUT} seconds. "
        "Inspect the simulation before restarting."
    )


# ============================================================
# PICK UP PROCESSED COMPONENT
# ============================================================

def pick_from_process1(client):
    """Pick up the processed component from Process 1."""
    print("\nPicking up the processed component...")

    if read_register(client, P1_RUNNING) != 0:
        raise RuntimeError(
            "Cannot pick up the component while Process 1 is running."
        )

    if read_register(client, P1_SENSOR) != 1:
        raise RuntimeError(
            "Cannot pick up: Process 1 sensor detects no component."
        )

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum must be OFF before picking up the processed component."
        )

    move_to(client, PROCESS1_X, SAFE_HEIGHT)
    move_to(client, PROCESS1_X, APPROACH_Y)
    move_to(client, PROCESS1_X, PICKUP_Y)

    write_register(client, VACUUM, 1)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, P1_SENSOR) != 0:
        raise RuntimeError(
            "Process 1 still detects the component after pickup. "
            "Inspect the simulation before continuing."
        )

    move_to(client, PROCESS1_X, APPROACH_Y)
    move_to(client, PROCESS1_X, SAFE_HEIGHT)

    print("Processed component picked up.")


# ============================================================
# DELIVER COMPONENT TO SINK
# ============================================================

def deliver_to_sink(client):
    """Deliver the component to the Sink."""
    print("\nDelivering component to the Sink...")

    if read_register(client, VACUUM) != 1:
        raise RuntimeError(
            "Vacuum is OFF; the crane may not be holding a component."
        )

    move_to(client, SINK_X, SAFE_HEIGHT)
    move_to(client, SINK_X, APPROACH_Y)
    move_to(client, SINK_X, PICKUP_Y)

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum did not turn OFF at the Sink."
        )

    # Raise the crane after releasing the component.
    move_to(client, SINK_X, APPROACH_Y)
    move_to(client, SINK_X, SAFE_HEIGHT)

    print("Component delivered to the Sink.")


# ============================================================
# COMPLETE GRADE D PRODUCTION CYCLE
# ============================================================

def run_one_cycle(client):
    """Process one component from Source 1 to the Sink."""
    pick_from_source1(client)
    place_at_process1(client)
    start_and_wait_for_process1(client)
    pick_from_process1(client)
    deliver_to_sink(client)

    print(
        "\nOne Grade D production cycle completed successfully."
    )


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():
    client = ModbusTcpClient(HOST, port=PORT)

    try:
        print("Connecting to the crane simulation...")

        if not client.connect():
            raise ConnectionError(
                f"Could not connect to {HOST}:{PORT}. "
                "Make sure the simulation is running."
            )

        print("Connected successfully.")

        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)

        print(
            f"Starting crane position: "
            f"X={current_x}, Y={current_y}"
        )

        # Validate initial conditions before moving.
        if read_register(client, VACUUM) != 0:
            raise RuntimeError(
                "Vacuum must be OFF before starting."
            )

        if read_register(client, SOURCE1_SENSOR) != 1:
            raise RuntimeError(
                "No component detected at Source 1. "
                "Generate one component before starting."
            )

        if read_register(client, P1_RUNNING) != 0:
            raise RuntimeError(
                "Process 1 is already running."
            )

        if read_register(client, P1_SENSOR) != 0:
            raise RuntimeError(
                "Process 1 is occupied. Clear the station "
                "before starting."
            )

        print("\nStarting Grade D sequence...")
        run_one_cycle(client)

        # Verify final conditions.
        final_vacuum = read_register(client, VACUUM)
        final_process_running = read_register(client, P1_RUNNING)
        final_process_sensor = read_register(client, P1_SENSOR)
        final_source1 = read_register(client, SOURCE1_SENSOR)
        final_x = read_register(client, AT_X)
        final_y = read_register(client, AT_Y)

        print("\nFinal state:")
        print(f"Crane X: {final_x}")
        print(f"Crane Y: {final_y}")
        print(f"Vacuum: {final_vacuum}")
        print(f"Source 1 sensor: {final_source1}")
        print(f"Process 1 running: {final_process_running}")
        print(f"Process 1 sensor: {final_process_sensor}")

        problems = []

        if abs(final_x - SINK_X) > POSITION_TOLERANCE:
            problems.append(
                f"Crane X should be {SINK_X}, but is {final_x}."
            )

        if abs(final_y - SAFE_HEIGHT) > POSITION_TOLERANCE:
            problems.append(
                f"Crane Y should be {SAFE_HEIGHT}, but is {final_y}."
            )

        if final_vacuum != 0:
            problems.append("Vacuum must be OFF.")

        if final_source1 != 0:
            problems.append(
                "Source 1 should be empty after the cycle."
            )

        if final_process_running != 0:
            problems.append("Process 1 is still running.")

        if final_process_sensor != 0:
            problems.append("Process 1 is not empty.")

        if problems:
            print("\nGRADE D VERIFICATION FAILED:")
            for problem in problems:
                print(f"- {problem}")

            raise RuntimeError(
                "The cycle ran, but the final state "
                "does not match the expected conditions."
            )

        print("\nGRADE D VERIFICATION PASSED.")
        print(
            "The component was processed and delivered to the Sink. "
            "The crane is at safe height with the vacuum OFF."
        )

    except Exception as error:
        print(f"\nController stopped: {error}")
        print(
            "Inspect the simulation before restarting. "
            "Do not reset automatically while a component "
            "may be held by the crane."
        )

    finally:
        client.close()
        print("Modbus connection closed.")


if __name__ == "__main__":
    main()
