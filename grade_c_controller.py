"""
PFA600 - GRADE C CRANE CONTROLLER
Grade D sequence + CSV product-location logging
"""
import csv
import time
from datetime import datetime
from pathlib import Path

from pymodbus.client import ModbusTcpClient


# ================= Connection settings ================================

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

# Input registers
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

# Crane positions
SOURCE1_X = 55
PROCESS1_X = 450
SINK_X = 945

PICKUP_Y = 82
APPROACH_Y = 120
SAFE_HEIGHT = 200

POSITION_TOLERANCE = 3
MOVE_TIMEOUT = 15
PROCESS_START_TIMEOUT = 5
PROCESS_TIMEOUT = 10
POLL_INTERVAL = 0.2
VACUUM_SETTLE_TIME = 0.5

LOG_FILE = Path(__file__).with_name("grade_c_log.csv")
CSV_FIELDS = ["product_id", "timestamp", "x", "y", "vacuum"]


# ------------------------------------------------------------
# Modbus helpers
# ------------------------------------------------------------

def read_register(client, address):
    result = client.read_holding_registers(
        address=address,
        count=1,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Failed to read holding register {address}."
        )

    return result.registers[0]


def write_register(client, address, value):
    result = client.write_register(
        address=address,
        value=value,
        slave=SLAVE_ID,
    )

    if result.isError():
        raise RuntimeError(
            f"Failed to write {value} to register {address}."
        )


def wait_for_position(client, target_x=None, target_y=None):
    start_time = time.monotonic()

    while time.monotonic() - start_time < MOVE_TIMEOUT:
        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)

        x_ok = (
            target_x is None
            or abs(current_x - target_x) <= POSITION_TOLERANCE
        )
        y_ok = (
            target_y is None
            or abs(current_y - target_y) <= POSITION_TOLERANCE
        )

        if x_ok and y_ok:
            return

        time.sleep(POLL_INTERVAL)

    raise TimeoutError(
        f"Crane did not reach target X={target_x}, Y={target_y}."
    )


def wait_for_register(client, address, expected, timeout, description):
    start_time = time.monotonic()

    while time.monotonic() - start_time < timeout:
        value = read_register(client, address)

        if value == expected:
            return

        time.sleep(POLL_INTERVAL)

    raise TimeoutError(
        f"Timed out waiting for {description} to become {expected}."
    )


# ------------------------------------------------------------
# Grade C CSV logging
# ------------------------------------------------------------

def ensure_log_file():
    """Create the CSV file and header if it does not exist."""
    if not LOG_FILE.exists() or LOG_FILE.stat().st_size == 0:
        with LOG_FILE.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=CSV_FIELDS,
            )
            writer.writeheader()


def next_product_id():
    """Continue product numbering when an existing log is present."""
    ensure_log_file()

    latest_id = 0

    with LOG_FILE.open(
        "r",
        newline="",
        encoding="utf-8",
    ) as file:
        for row in csv.DictReader(file):
            try:
                latest_id = max(
                    latest_id,
                    int(row["product_id"]),
                )
            except (KeyError, TypeError, ValueError):
                continue

    return latest_id + 1


def log_location(client, product_id, note):
    """Record the crane's current coordinates and vacuum state."""
    x = read_register(client, AT_X)
    y = read_register(client, AT_Y)
    vacuum = read_register(client, VACUUM)

    timestamp = datetime.now().astimezone().isoformat(
        timespec="seconds"
    )

    ensure_log_file()

    with LOG_FILE.open(
        "a",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=CSV_FIELDS,
        )
        writer.writerow({
            "product_id": product_id,
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "vacuum": vacuum,
        })

    print(
        f"LOG | Product {product_id} | {note} | "
        f"X={x}, Y={y}, Vacuum={vacuum}"
    )


# ------------------------------------------------------------
# Crane movement
# ------------------------------------------------------------

def move_to(client, target_x, target_y, product_id=None):
    """
    Move safely to a target position.

    The crane rises before horizontal travel when necessary.
    While carrying the product, completed movements are logged.
    """
    current_x = read_register(client, AT_X)
    current_y = read_register(client, AT_Y)

    carrying = read_register(client, VACUUM) == 1

    # Raise before horizontal movement if below safe height.
    if current_y != SAFE_HEIGHT and current_y < SAFE_HEIGHT:
        write_register(client, SET_Y, SAFE_HEIGHT)
        wait_for_position(client, target_y=SAFE_HEIGHT)

        if carrying and product_id is not None:
            log_location(
                client,
                product_id,
                "Moved to safe height",
            )

    # Move horizontally at safe height.
    current_x = read_register(client, AT_X)

    if current_x != target_x:
        write_register(client, SET_X, target_x)
        wait_for_position(client, target_x=target_x)

        if carrying and product_id is not None:
            log_location(
                client,
                product_id,
                "Horizontal movement completed",
            )

    # Move vertically to the requested height.
    current_y = read_register(client, AT_Y)

    if current_y != target_y:
        write_register(client, SET_Y, target_y)
        wait_for_position(client, target_y=target_y)

        if carrying and product_id is not None:
            log_location(
                client,
                product_id,
                "Vertical movement completed",
            )


# ------------------------------------------------------------
# Source 1 pickup
# ------------------------------------------------------------

def pick_from_source1(client, product_id):
    print("\nStep 1: Pick up the component from Source 1.")

    if read_register(client, SOURCE1_SENSOR) != 1:
        raise RuntimeError(
            "No component is detected at Source 1."
        )

    move_to(
        client,
        SOURCE1_X,
        SAFE_HEIGHT,
        product_id,
    )

    move_to(
        client,
        SOURCE1_X,
        PICKUP_Y,
        product_id,
    )

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum was already ON before pickup."
        )

    write_register(client, VACUUM, 1)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, SOURCE1_SENSOR) != 0:
        raise RuntimeError(
            "Source 1 sensor did not clear after pickup. "
            "Check the simulation before continuing."
        )

    log_location(
        client,
        product_id,
        "Picked up component from Source 1",
    )

    move_to(
        client,
        SOURCE1_X,
        SAFE_HEIGHT,
        product_id,
    )


# ------------------------------------------------------------
# Process 1 placement and processing
# ------------------------------------------------------------

def place_at_process1(client, product_id):
    print("\nStep 2: Place the component at Process 1.")

    move_to(
        client,
        PROCESS1_X,
        SAFE_HEIGHT,
        product_id,
    )

    if read_register(client, P1_SENSOR) != 0:
        raise RuntimeError(
            "Process 1 is already occupied."
        )

    move_to(
        client,
        PROCESS1_X,
        PICKUP_Y,
        product_id,
    )

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum did not switch OFF at Process 1."
        )

    wait_for_register(
        client,
        P1_SENSOR,
        1,
        timeout=3,
        description="Process 1 component sensor",
    )

    log_location(
        client,
        product_id,
        "Component released at Process 1",
    )

    # The crane is now empty, so its movements are not
    # recorded as movements of the component.
    move_to(
        client,
        PROCESS1_X,
        SAFE_HEIGHT,
    )


def start_and_wait_for_process1(client):
    print("\nStep 3: Run Process 1.")

    if read_register(client, P1_SENSOR) != 1:
        raise RuntimeError(
            "Process 1 has no component to process."
        )

    if read_register(client, P1_RUNNING) != 0:
        raise RuntimeError(
            "Process 1 is already running."
        )

    write_register(client, START_P1, 1)

    wait_for_register(
        client,
        P1_RUNNING,
        1,
        timeout=PROCESS_START_TIMEOUT,
        description="Process 1 running signal",
    )

    print("Process 1 has started.")

    wait_for_register(
        client,
        P1_RUNNING,
        0,
        timeout=PROCESS_TIMEOUT,
        description="Process 1 completion signal",
    )

    print("Process 1 has completed.")


# ------------------------------------------------------------
# Collect the processed component
# ------------------------------------------------------------

def pick_from_process1(client, product_id):
    print("\nStep 4: Collect the component from Process 1.")

    if read_register(client, P1_SENSOR) != 1:
        raise RuntimeError(
            "No component is detected at Process 1."
        )

    move_to(
        client,
        PROCESS1_X,
        SAFE_HEIGHT,
    )

    move_to(
        client,
        PROCESS1_X,
        PICKUP_Y,
    )

    write_register(client, VACUUM, 1)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, P1_SENSOR) != 0:
        raise RuntimeError(
            "Process 1 sensor did not clear after pickup. "
            "Check the simulation before continuing."
        )

    log_location(
        client,
        product_id,
        "Picked up processed component from Process 1",
    )

    move_to(
        client,
        PROCESS1_X,
        SAFE_HEIGHT,
        product_id,
    )


# ------------------------------------------------------------
# Deliver the component to the sink
# ------------------------------------------------------------

def deliver_to_sink(client, product_id):
    print("\nStep 5: Deliver the component to the Sink.")

    move_to(
        client,
        SINK_X,
        SAFE_HEIGHT,
        product_id,
    )

    move_to(
        client,
        SINK_X,
        PICKUP_Y,
        product_id,
    )

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, VACUUM) != 0:
        raise RuntimeError(
            "Vacuum did not switch OFF at the Sink."
        )

    log_location(
        client,
        product_id,
        "Component released at Sink",
    )

    move_to(
        client,
        SINK_X,
        SAFE_HEIGHT,
    )


# ------------------------------------------------------------
# One complete Grade C cycle
# ------------------------------------------------------------

def run_one_cycle(client, product_id):
    log_location(
        client,
        product_id,
        "Component detected at Source 1 before pickup",
    )

    pick_from_source1(client, product_id)
    place_at_process1(client, product_id)
    start_and_wait_for_process1(client)
    pick_from_process1(client, product_id)
    deliver_to_sink(client, product_id)


# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------

def main():
    client = ModbusTcpClient(
        HOST,
        port=PORT,
        timeout=3,
    )

    print("PFA600 Grade C Controller")
    print("=========================")
    print(f"CSV log: {LOG_FILE}")

    try:
        if not client.connect():
            raise ConnectionError(
                f"Could not connect to {HOST}:{PORT}. "
                "Check that the simulation is running."
            )

        print("Modbus connection successful.")

        # Check the initial simulation state before moving.
        current_x = read_register(client, AT_X)
        current_y = read_register(client, AT_Y)
        vacuum = read_register(client, VACUUM)
        source1 = read_register(client, SOURCE1_SENSOR)
        p1_running = read_register(client, P1_RUNNING)
        p1_sensor = read_register(client, P1_SENSOR)

        print("\nInitial simulation state:")
        print(f"Crane X: {current_x}")
        print(f"Crane Y: {current_y}")
        print(f"Vacuum: {vacuum}")
        print(f"Source 1 sensor: {source1}")
        print(f"Process 1 running: {p1_running}")
        print(f"Process 1 sensor: {p1_sensor}")

        if vacuum != 0:
            raise RuntimeError(
                "Initial vacuum must be OFF."
            )

        if source1 != 1:
            raise RuntimeError(
                "Generate exactly one Source 1 component "
                "before running the controller."
            )

        if p1_running != 0:
            raise RuntimeError(
                "Process 1 must be idle before starting."
            )

        if p1_sensor != 0:
            raise RuntimeError(
                "Process 1 must be empty before starting."
            )

        ensure_log_file()
        product_id = next_product_id()

        print(f"\nAssigned product ID: {product_id}")

        run_one_cycle(client, product_id)

        # Final verification
        final_x = read_register(client, AT_X)
        final_y = read_register(client, AT_Y)
        final_vacuum = read_register(client, VACUUM)
        final_source1 = read_register(client, SOURCE1_SENSOR)
        final_p1_running = read_register(client, P1_RUNNING)
        final_p1_sensor = read_register(client, P1_SENSOR)

        print("\nFinal simulation state:")
        print(f"Crane X: {final_x}")
        print(f"Crane Y: {final_y}")
        print(f"Vacuum: {final_vacuum}")
        print(f"Source 1 sensor: {final_source1}")
        print(f"Process 1 running: {final_p1_running}")
        print(f"Process 1 sensor: {final_p1_sensor}")

        if not (
            abs(final_x - SINK_X) <= POSITION_TOLERANCE
            and abs(final_y - SAFE_HEIGHT) <= POSITION_TOLERANCE
            and final_vacuum == 0
            and final_source1 == 0
            and final_p1_running == 0
            and final_p1_sensor == 0
        ):
            raise RuntimeError(
                "Final-state verification failed. "
                "Inspect the simulation and CSV log."
            )

        print("\nGRADE C VERIFICATION PASSED.")
        print("The component was processed and delivered to the Sink.")
        print("The crane is at safe height with the vacuum OFF.")
        print(f"Location history saved to: {LOG_FILE}")

    except Exception as error:
        print(f"\nGRADE C ERROR: {error}")
        print(
            "Inspect the simulation before attempting another run. "
            "Do not reset automatically if the component may be "
            "held by the crane."
        )

    finally:
        client.close()
        print("\nModbus connection closed.")


if __name__ == "__main__":
    main()
