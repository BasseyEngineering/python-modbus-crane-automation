"""
PFA600 Grade B crane controller
Route Type 1: Source 1 -> Process 1 -> Sink
Route Type 2: Source 2 -> Process 2 -> Process 1 -> Sink
"""

import csv
import time
from datetime import datetime
from pathlib import Path

from pymodbus.client import ModbusTcpClient


# ------------------------- CONNECTION SETTINGS -------------------------

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

# Station positions
SOURCE1_X = 55
SOURCE2_X = 158
PROCESS1_X = 450
PROCESS2_X = 650
SINK_X = 945

PICKUP_Y = 82
APPROACH_Y = 120
SAFE_HEIGHT = 200

POSITION_TOLERANCE = 3
MOVE_TIMEOUT = 15
PROCESS_START_TIMEOUT = 5
POLL_INTERVAL = 0.2
VACUUM_SETTLE_TIME = 0.5

# Exit after this much time with no parts anywhere in the cell.
IDLE_EXIT_SECONDS = 30

LOG_FILE = Path(__file__).with_name("grade_b_log.csv")
CSV_FIELDS = ["product_id", "timestamp", "x", "y", "vacuum"]


# ------------------------- MODBUS HELPERS -----------------------------

def read_register(client, address):
    """Read one holding register and return its integer value."""
    result = client.read_holding_registers(
        address=address, count=1, slave=SLAVE_ID
    )
    if result.isError():
        raise RuntimeError(f"Could not read Modbus register {address}.")
    return result.registers[0]


def write_register(client, address, value):
    """Write one holding register and check for a Modbus error."""
    result = client.write_register(
        address=address, value=value, slave=SLAVE_ID
    )
    if result.isError():
        raise RuntimeError(
            f"Could not write value {value} to Modbus register {address}."
        )


def wait_for_position(client, address, target, label):
    """Wait until the crane reaches the requested X or Y position."""
    started = time.monotonic()
    while time.monotonic() - started < MOVE_TIMEOUT:
        current = read_register(client, address)
        if abs(current - target) <= POSITION_TOLERANCE:
            return
        time.sleep(POLL_INTERVAL)
    current = read_register(client, address)
    raise TimeoutError(
        f"Timed out moving to {label}={target}; current value is {current}. "
        "Inspect the simulation for an obstruction or collision."
    )


def log_location(client, product_id):
    """Append the current crane position and vacuum state to the CSV log."""
    new_file = not LOG_FILE.exists()
    with LOG_FILE.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow({
            "product_id": product_id or "",
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "x": read_register(client, AT_X),
            "y": read_register(client, AT_Y),
            "vacuum": read_register(client, VACUUM),
        })


# ------------------------- CRANE MOVEMENT -----------------------------

def move_to(client, target_x, target_y, product_id=None):
    """
    Move horizontally only at safe height, then move vertically.
    This reduces the risk of colliding with station parts and equipment.
    """
    current_x = read_register(client, AT_X)
    current_y = read_register(client, AT_Y)

    # Only raise to safe height before a horizontal move. If X is already
    # correct, vertical approach/retreat moves can happen directly.
    if current_x != target_x:
        if current_y != SAFE_HEIGHT:
            write_register(client, SET_Y, SAFE_HEIGHT)
            wait_for_position(client, AT_Y, SAFE_HEIGHT, "Y")
        write_register(client, SET_X, target_x)
        wait_for_position(client, AT_X, target_x, "X")

    current_y = read_register(client, AT_Y)
    if current_y != target_y:
        write_register(client, SET_Y, target_y)
        wait_for_position(client, AT_Y, target_y, "Y")

    log_location(client, product_id)


def approach_station(client, station_x, product_id=None):
    """Move to a station's pickup height via its approach height."""
    move_to(client, station_x, SAFE_HEIGHT, product_id)
    move_to(client, station_x, APPROACH_Y, product_id)
    move_to(client, station_x, PICKUP_Y, product_id)


def retreat_to_safe_height(client, station_x, product_id=None):
    """Move vertically away from a station after picking or releasing."""
    move_to(client, station_x, APPROACH_Y, product_id)
    move_to(client, station_x, SAFE_HEIGHT, product_id)


# ------------------------- PICK AND PLACE -----------------------------

def pick_from_station(client, station_x, sensor_register, product_id):
    """Pick a part from a source or process station."""
    if read_register(client, VACUUM) != 0:
        raise RuntimeError("Cannot pick a part: vacuum is already ON.")

    if read_register(client, sensor_register) != 1:
        raise RuntimeError(
            f"Cannot pick from X={station_x}: station sensor detects no part."
        )

    approach_station(client, station_x, product_id)
    write_register(client, VACUUM, 1)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, sensor_register) != 0:
        raise RuntimeError(
            f"The sensor at X={station_x} still detects the part after pickup. "
            "Inspect the simulation before continuing."
        )

    log_location(client, product_id)
    retreat_to_safe_height(client, station_x, product_id)
    print(f"Picked up {product_id} at X={station_x}.")


def place_at_station(client, station_x, sensor_register, product_id):
    """Release the carried part at a process station."""
    if read_register(client, VACUUM) != 1:
        raise RuntimeError("Cannot place a part: vacuum is OFF.")

    if read_register(client, sensor_register) != 0:
        raise RuntimeError(
            f"Cannot place at X={station_x}: the station is occupied."
        )

    move_to(client, station_x, SAFE_HEIGHT, product_id)
    move_to(client, station_x, APPROACH_Y, product_id)
    move_to(client, station_x, PICKUP_Y, product_id)

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)

    if read_register(client, sensor_register) != 1:
        raise RuntimeError(
            f"The station sensor at X={station_x} did not detect the placed part. "
            "Inspect the simulation before continuing."
        )

    log_location(client, product_id)
    retreat_to_safe_height(client, station_x, product_id)
    print(f"Placed {product_id} at X={station_x}.")


def deliver_to_sink(client, product_id):
    """Release the carried part at the Sink."""
    if read_register(client, VACUUM) != 1:
        raise RuntimeError("Cannot deliver: vacuum is OFF.")

    move_to(client, SINK_X, SAFE_HEIGHT, product_id)
    move_to(client, SINK_X, APPROACH_Y, product_id)
    move_to(client, SINK_X, PICKUP_Y, product_id)

    write_register(client, VACUUM, 0)
    time.sleep(VACUUM_SETTLE_TIME)
    log_location(client, product_id)

    retreat_to_safe_height(client, SINK_X, product_id)
    print(f"Delivered {product_id} to the Sink.")


# ------------------------- PROCESS MANAGEMENT -------------------------

def start_process_if_needed(
    client, sensor_register, running_register, start_register,
    started_flag, done_flag, process_name
):
    """
    Start a process once for its current part.
    The simulation keeps the start register ON while processing and clears it
    when the process finishes. The controller does not block while it runs.
    """
    has_part = read_register(client, sensor_register) == 1
    is_running = read_register(client, running_register) == 1
    start_signal = read_register(client, start_register) == 1

    if not has_part:
        return False, False

    if not started_flag and not done_flag and not is_running and not start_signal:
        print(f"{process_name}: starting process.")
        write_register(client, start_register, 1)
        started_flag = True

        started = time.monotonic()
        while time.monotonic() - started < PROCESS_START_TIMEOUT:
            if read_register(client, running_register) == 1:
                break
            time.sleep(POLL_INTERVAL)
        else:
            raise TimeoutError(f"{process_name} did not start.")

    # Completion is indicated by both the running status and start signal
    # returning to zero after this controller started the current job.
    is_running = read_register(client, running_register) == 1
    start_signal = read_register(client, start_register) == 1
    if started_flag and not is_running and not start_signal:
        done_flag = True
        print(f"{process_name}: processing complete.")

    return started_flag, done_flag


# ------------------------- MAIN CONTROLLER ----------------------------

def main():
    client = ModbusTcpClient(HOST, port=PORT)

    # Logical IDs assigned by this controller. They are for the CSV log;
    # the simulation's internal part numbers are not exposed by Modbus.
    next_product_number = 1
    source1_product = None
    source2_product = None
    process1_product = None
    process2_product = None
    carried_product = None

    p1_started = False
    p1_done = False
    p2_started = False
    p2_done = False

    last_work_time = time.monotonic()

    try:
        print("Connecting to the PFA600 crane simulation...")
        if not client.connect():
            raise ConnectionError(
                f"Could not connect to {HOST}:{PORT}. "
                "Start plug_and_produce_simulation.py first."
            )
        print("Connected successfully.")

        if read_register(client, VACUUM) != 0:
            raise RuntimeError(
                "Vacuum is ON at startup. Inspect the crane and turn it OFF "
                "only if the gripper is not holding a part."
            )

        if read_register(client, P1_SENSOR) or read_register(client, P2_SENSOR):
            raise RuntimeError(
                "Process 1 or Process 2 is occupied at startup. "
                "This controller expects both process stations to be empty."
            )

        if read_register(client, P1_RUNNING) or read_register(client, P2_RUNNING):
            raise RuntimeError(
                "A process is already running. Let it finish before starting "
                "this controller."
            )

        print("\nGrade B controller is running.")
        print("Type 1 route: Source 1 -> Process 1 -> Sink")
        print("Type 2 route: Source 2 -> Process 2 -> Process 1 -> Sink")
        print(f"New components can be generated while this program runs.")
        print(f"If the whole cell stays idle for {IDLE_EXIT_SECONDS} seconds, "
              "the controller will exit.")
        print("Do not start another grade controller at the same time.\n")

        while True:
            # Read station state at the start of every scheduling cycle.
            source1_present = read_register(client, SOURCE1_SENSOR) == 1
            source2_present = read_register(client, SOURCE2_SENSOR) == 1
            p1_present = read_register(client, P1_SENSOR) == 1
            p2_present = read_register(client, P2_SENSOR) == 1
            p1_running = read_register(client, P1_RUNNING) == 1
            p2_running = read_register(client, P2_RUNNING) == 1
            vacuum_on = read_register(client, VACUUM) == 1

            # Assign controller IDs when a new source component appears.
            # A source can only hold one part at a time.
            if source1_present and source1_product is None:
                source1_product = f"B{next_product_number:04d}"
                next_product_number += 1
                print(f"Detected Type 1 component {source1_product} at Source 1.")
            if not source1_present:
                source1_product = None

            if source2_present and source2_product is None:
                source2_product = f"B{next_product_number:04d}"
                next_product_number += 1
                print(f"Detected Type 2 component {source2_product} at Source 2.")
            if not source2_present:
                source2_product = None

            # Keep track of the part currently occupying each process station.
            if not p1_present:
                process1_product = None
                p1_started = False
                p1_done = False
            if not p2_present:
                process2_product = None
                p2_started = False
                p2_done = False

            # A newly placed part at P1/P2 gets an ID from the source/transfer
            # operation below. Start each process once, without blocking the
            # crane from doing other work.
            if p1_present and process1_product is not None:
                p1_started, p1_done = start_process_if_needed(
                    client, P1_SENSOR, P1_RUNNING, START_P1,
                    p1_started, p1_done, "Process 1"
                )
            if p2_present and process2_product is not None:
                p2_started, p2_done = start_process_if_needed(
                    client, P2_SENSOR, P2_RUNNING, START_P2,
                    p2_started, p2_done, "Process 2"
                )

            # Refresh status after process updates.
            p1_present = read_register(client, P1_SENSOR) == 1
            p2_present = read_register(client, P2_SENSOR) == 1
            p1_running = read_register(client, P1_RUNNING) == 1
            p2_running = read_register(client, P2_RUNNING) == 1
            vacuum_on = read_register(client, VACUUM) == 1

            did_work = False

            # 1. Finished P1 parts always go to the Sink first.
            if (p1_present and process1_product is not None and p1_done
                    and not p1_running and not vacuum_on):
                carried_product = process1_product
                print(f"\nTaking finished {carried_product} from Process 1.")
                pick_from_station(client, PROCESS1_X, P1_SENSOR, carried_product)
                process1_product = None
                p1_started = False
                p1_done = False
                deliver_to_sink(client, carried_product)
                carried_product = None
                did_work = True

            # 2. A finished Type 2 part can move from P2 to P1 only if P1
            #    is empty. P1 must then process it before it reaches the Sink.
            if not did_work:
                p1_present = read_register(client, P1_SENSOR) == 1
                p2_present = read_register(client, P2_SENSOR) == 1
                p1_running = read_register(client, P1_RUNNING) == 1
                p2_running = read_register(client, P2_RUNNING) == 1
                vacuum_on = read_register(client, VACUUM) == 1

                if (p2_present and process2_product is not None and p2_done
                        and not p2_running and not p1_present and not p1_running
                        and not vacuum_on):
                    carried_product = process2_product
                    print(f"\nTransferring {carried_product} from Process 2 "
                          "to Process 1.")
                    pick_from_station(client, PROCESS2_X, P2_SENSOR, carried_product)
                    process2_product = None
                    p2_started = False
                    p2_done = False
                    place_at_station(client, PROCESS1_X, P1_SENSOR, carried_product)
                    process1_product = carried_product
                    carried_product = None
                    did_work = True

            # 3. Type 1 components go directly from Source 1 to P1 if it is free.
            if not did_work:
                p1_present = read_register(client, P1_SENSOR) == 1
                p1_running = read_register(client, P1_RUNNING) == 1
                vacuum_on = read_register(client, VACUUM) == 1
                source1_present = read_register(client, SOURCE1_SENSOR) == 1

                if (source1_present and source1_product is not None
                        and not p1_present and not p1_running and not vacuum_on):
                    carried_product = source1_product
                    print(f"\nMoving Type 1 component {carried_product} "
                          "from Source 1 to Process 1.")
                    pick_from_station(
                        client, SOURCE1_X, SOURCE1_SENSOR, carried_product
                    )
                    source1_product = None
                    place_at_station(client, PROCESS1_X, P1_SENSOR, carried_product)
                    process1_product = carried_product
                    carried_product = None
                    did_work = True

            # 4. Type 2 components go from Source 2 to P2 if it is free.
            if not did_work:
                p2_present = read_register(client, P2_SENSOR) == 1
                p2_running = read_register(client, P2_RUNNING) == 1
                vacuum_on = read_register(client, VACUUM) == 1
                source2_present = read_register(client, SOURCE2_SENSOR) == 1

                if (source2_present and source2_product is not None
                        and not p2_present and not p2_running and not vacuum_on):
                    carried_product = source2_product
                    print(f"\nMoving Type 2 component {carried_product} "
                          "from Source 2 to Process 2.")
                    pick_from_station(
                        client, SOURCE2_X, SOURCE2_SENSOR, carried_product
                    )
                    source2_product = None
                    place_at_station(client, PROCESS2_X, P2_SENSOR, carried_product)
                    process2_product = carried_product
                    carried_product = None
                    did_work = True

            if did_work:
                last_work_time = time.monotonic()
                continue

            # Do not exit while any source, process station, process, or gripper
            # contains a part. This makes the idle timeout safe for normal use.
            source1_present = read_register(client, SOURCE1_SENSOR) == 1
            source2_present = read_register(client, SOURCE2_SENSOR) == 1
            p1_present = read_register(client, P1_SENSOR) == 1
            p2_present = read_register(client, P2_SENSOR) == 1
            p1_running = read_register(client, P1_RUNNING) == 1
            p2_running = read_register(client, P2_RUNNING) == 1
            vacuum_on = read_register(client, VACUUM) == 1

            cell_busy = (
                source1_present or source2_present or p1_present or p2_present
                or p1_running or p2_running or vacuum_on
                or carried_product is not None
            )

            if cell_busy:
                last_work_time = time.monotonic()
            elif time.monotonic() - last_work_time >= IDLE_EXIT_SECONDS:
                print(f"\nNo work detected for {IDLE_EXIT_SECONDS} seconds.")
                print("Grade B controller is exiting normally.")
                break

            time.sleep(POLL_INTERVAL)

        # Final state summary after the idle exit.
        print("\nFinal station state:")
        print(f"Source 1 sensor: {read_register(client, SOURCE1_SENSOR)}")
        print(f"Source 2 sensor: {read_register(client, SOURCE2_SENSOR)}")
        print(f"Process 1 sensor: {read_register(client, P1_SENSOR)}")
        print(f"Process 2 sensor: {read_register(client, P2_SENSOR)}")
        print(f"Process 1 running: {read_register(client, P1_RUNNING)}")
        print(f"Process 2 running: {read_register(client, P2_RUNNING)}")
        print(f"Vacuum: {read_register(client, VACUUM)}")
        print(f"Crane X: {read_register(client, AT_X)}")
        print(f"Crane Y: {read_register(client, AT_Y)}")
        print(f"CSV log: {LOG_FILE}")

    except KeyboardInterrupt:
        print("\nController interrupted by the user.")
        print("Inspect the simulation before restarting, especially if the "
              "vacuum is ON or a part is between stations.")
    except Exception as error:
        print(f"\nController stopped because of an error: {error}")
        print("Inspect the simulation before restarting. Do not reset "
              "automatically while a component may be held by the crane.")
    finally:
        client.close()
        print("Modbus connection closed.")


if __name__ == "__main__":
    main()
