"""
PFA600 Grade A HMI
- Manual crane movement with live X/Y display
- Named positions saved to JSON; update a selected position from current X/Y
- Build, save, load, and execute an action sequence from the HMI
- Supports movement, vacuum ON/OFF, and Process 1 / Process 2 actions
- Uses Modbus TCP at 127.0.0.1:5020 

"""

import json
import subprocess
import sys
import os
import csv
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from pathlib import Path

from pymodbus.client import ModbusTcpClient


# ----------------------------- SETTINGS --------------------------------

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

PICKUP_Y = 82
APPROACH_Y = 120
SAFE_HEIGHT = 200

POSITION_TOLERANCE = 3
MOVE_TIMEOUT = 15
PROCESS_START_TIMEOUT = 5
PROCESS_TIMEOUT = 15
POLL_INTERVAL = 0.15
VACUUM_SETTLE_TIME = 0.5

BASE_DIR = Path(__file__).resolve().parent
POSITIONS_FILE = BASE_DIR / "grade_a_positions.json"
SEQUENCE_FILE = BASE_DIR / "grade_a_sequence.json"

DEFAULT_POSITIONS = {
    "Source 1": {"x": 55, "y": 200},
    "Source 1 Pickup": {"x": 55, "y": 82},
    "Source 2": {"x": 158, "y": 200},
    "Source 2 Pickup": {"x": 158, "y": 82},
    "Process 1": {"x": 450, "y": 200},
    "Process 1 Pickup": {"x": 450, "y": 82},
    "Process 2": {"x": 650, "y": 200},
    "Process 2 Pickup": {"x": 650, "y": 82},
    "Sink": {"x": 945, "y": 200},
    "Sink Release": {"x": 945, "y": 82},
}


# ----------------------------- HMI APP ---------------------------------

class GradeAHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("PFA600 Grade A - Crane HMI")
        self.root.geometry("1050x700")
        self.root.minsize(860, 560)

        self.client = None
        self.client_lock = threading.Lock()
        self.connected = False
        self.busy = False
        self.stop_requested = threading.Event()
        self.worker = None

        self.positions = self._load_positions_startup()
        self.actions = []
        self.auto_sim_proc = None
        self.auto_controller_proc = None
        self.auto_log_handle = None
        self.auto_limit_var = tk.StringVar(value="2")
        self.auto_interval_var = tk.StringVar(value="8")
        self.auto_status_var = tk.StringVar(value="Automatic production: idle")
        self.station_var = tk.StringVar(value="S1: --   S2: --   P1: --   P2: --")
        self.delivered_var = tk.StringVar(value="Delivered this run: 0")
        self.auto_run_start = None
        self.auto_output_file = BASE_DIR / "grade_a_auto_output.log"
        self.auto_controller_log = BASE_DIR / "grade_b_log.csv"

        self.current_x_var = tk.StringVar(value="X: --")
        self.current_y_var = tk.StringVar(value="Y: --")
        self.vacuum_var = tk.StringVar(value="Vacuum: --")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="Connect to the simulation to begin.")
        self.manual_x_var = tk.StringVar(value="500")
        self.manual_y_var = tk.StringVar(value="200")
        self.position_name_var = tk.StringVar()
        self.position_x_var = tk.StringVar(value="945")
        self.position_y_var = tk.StringVar(value="82")
        self.vacuum_action_var = tk.StringVar(value="ON")
        self.process_action_var = tk.StringVar(value="Process 1")

        self._build_ui()
        self._refresh_position_list()
        self._refresh_action_list()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(300, self._poll_status)

    # ----------------------------- UI ----------------------------------

    def _build_ui(self):
        # Use separate tabs rather than one tall window. Each major control
        # group remains accessible even on small laptop displays.
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer)
        header.pack(fill="x", pady=(0, 8))
        ttk.Label(header, text="PFA600 Grade A Crane HMI",
                  font=("TkDefaultFont", 16, "bold")).pack(side="left")
        self.connect_button = ttk.Button(
            header, text="Connect", command=self.connect_simulation
        )
        self.connect_button.pack(side="right", padx=(8, 0))
        ttk.Label(header, textvariable=self.connection_var).pack(side="right")

        notebook = ttk.Notebook(outer)
        notebook.pack(fill="both", expand=True, pady=(6, 0))
        production_tab = ttk.Frame(notebook, padding=8)
        manual_tab = ttk.Frame(notebook, padding=8)
        sequence_tab = ttk.Frame(notebook, padding=8)
        notebook.add(production_tab, text="Automatic production")
        notebook.add(manual_tab, text="Manual & positions")
        notebook.add(sequence_tab, text="Sequence builder")

        auto = ttk.LabelFrame(production_tab, text="Automatic production (Type 1 and Type 2)", padding=8)
        auto.pack(fill="x", pady=(8, 0))
        settings = ttk.Frame(auto)
        settings.pack(fill="x")
        ttk.Label(settings, text="Parts per source:").pack(side="left")
        ttk.Entry(settings, textvariable=self.auto_limit_var, width=5).pack(side="left", padx=5)
        ttk.Label(settings, text="Interval (seconds):").pack(side="left", padx=(8, 0))
        ttk.Entry(settings, textvariable=self.auto_interval_var, width=5).pack(side="left", padx=5)
        auto_buttons = ttk.Frame(auto)
        auto_buttons.pack(fill="x", pady=(8, 2))
        self.auto_sim_button = ttk.Button(auto_buttons, text="Launch auto simulator", command=self.launch_auto_simulator)
        self.auto_sim_button.pack(side="left", padx=5)
        self.auto_start_button = ttk.Button(auto_buttons, text="Start automatic controller", command=self.start_auto_controller)
        self.auto_start_button.pack(side="left", padx=5)
        self.auto_stop_button = ttk.Button(auto_buttons, text="Stop after transfer", command=self.stop_auto_controller)
        self.auto_stop_button.pack(side="left", padx=5)
        ttk.Label(auto, textvariable=self.auto_status_var).pack(anchor="w", pady=(4, 0))
        ttk.Label(auto, textvariable=self.station_var).pack(anchor="w")
        ttk.Label(auto, textvariable=self.delivered_var).pack(anchor="w")
        ttk.Button(auto, text="Open production CSV", command=self.open_auto_csv).pack(anchor="w", pady=(4, 0))
        ttk.Label(auto, text="Stop after transfer is not an emergency stop. Close only after the controller exits.",
                  wraplength=880).pack(anchor="w")


        status = ttk.LabelFrame(outer, text="Live simulation status", padding=8)
        status.pack(fill="x", pady=(0, 8))
        ttk.Label(status, textvariable=self.current_x_var,
                  font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=12)
        ttk.Label(status, textvariable=self.current_y_var,
                  font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=12)
        ttk.Label(status, textvariable=self.vacuum_var,
                  font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=12)

        top = ttk.Frame(manual_tab)
        top.pack(fill="both", expand=True)

        manual = ttk.LabelFrame(top, text="Manual crane control", padding=10)
        manual.pack(side="left", fill="both", expand=True, padx=(0, 8))
        ttk.Label(manual, text="Target X").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(manual, textvariable=self.manual_x_var, width=12).grid(
            row=0, column=1, sticky="w", padx=5, pady=4
        )
        ttk.Label(manual, text="Target Y").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(manual, textvariable=self.manual_y_var, width=12).grid(
            row=1, column=1, sticky="w", padx=5, pady=4
        )
        self.move_button = ttk.Button(
            manual, text="Move to X/Y", command=self.move_manual
        )
        self.move_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=6)

        ttk.Separator(manual).grid(row=3, column=0, columnspan=2, sticky="ew", pady=8)
        ttk.Label(manual, text="Vacuum").grid(row=4, column=0, sticky="w", pady=4)
        vacuum_row = ttk.Frame(manual)
        vacuum_row.grid(row=4, column=1, sticky="w")
        self.vacuum_on_button = ttk.Button(
            vacuum_row, text="ON / Pick", command=lambda: self.set_vacuum(1)
        )
        self.vacuum_on_button.pack(side="left", padx=(0, 4))
        self.vacuum_off_button = ttk.Button(
            vacuum_row, text="OFF / Release", command=lambda: self.set_vacuum(0)
        )
        self.vacuum_off_button.pack(side="left")

        ttk.Label(
            manual,
            text=("Safety note: move horizontally at Y=200. "
                  "Use Y=120 as an approach offset before Y=82."),
            wraplength=350
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(10, 0))

        positions = ttk.LabelFrame(top, text="Saved positions", padding=10)
        positions.pack(side="left", fill="both", expand=True)
        self.position_list = tk.Listbox(positions, height=11, exportselection=False)
        self.position_list.pack(fill="both", expand=True)
        self.position_list.bind("<<ListboxSelect>>", self._on_position_select)

        name_row = ttk.Frame(positions)
        name_row.pack(fill="x", pady=(6, 0))
        ttk.Label(name_row, text="Name").pack(side="left")
        ttk.Entry(name_row, textvariable=self.position_name_var).pack(
            side="left", fill="x", expand=True, padx=5
        )
        coord_row = ttk.Frame(positions)
        coord_row.pack(fill="x", pady=(6, 0))
        ttk.Label(coord_row, text="Edit X").pack(side="left")
        ttk.Entry(coord_row, textvariable=self.position_x_var, width=7).pack(side="left", padx=(4, 10))
        ttk.Label(coord_row, text="Edit Y").pack(side="left")
        ttk.Entry(coord_row, textvariable=self.position_y_var, width=7).pack(side="left", padx=4)
        pos_buttons = ttk.Frame(positions)
        pos_buttons.pack(fill="x", pady=(6, 0))
        ttk.Button(pos_buttons, text="Add / Save current position",
                   command=self.save_current_position).pack(fill="x", pady=2)
        ttk.Button(pos_buttons, text="Save edited X/Y to selected position",
                   command=self.save_edited_position).pack(fill="x", pady=2)
        ttk.Button(pos_buttons, text="Update selected from current crane position",
                   command=self.update_selected_position).pack(fill="x", pady=2)
        ttk.Button(pos_buttons, text="Move to selected position",
                   command=self.move_to_selected_position).pack(fill="x", pady=2)
        ttk.Button(pos_buttons, text="Delete selected position",
                   command=self.delete_selected_position).pack(fill="x", pady=2)
        file_row = ttk.Frame(positions)
        file_row.pack(fill="x", pady=(4, 0))
        ttk.Button(file_row, text="Load Positions JSON",
                   command=self.load_positions_dialog).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(file_row, text="Save Positions JSON",
                   command=self.save_positions_dialog).pack(side="left", fill="x", expand=True, padx=(3, 0))
        ttk.Button(positions, text="Restore default positions",
                   command=self.restore_default_positions).pack(fill="x", pady=(6, 0))

        sequence = ttk.LabelFrame(sequence_tab, text="Sequence builder", padding=10)
        sequence.pack(fill="both", expand=True, pady=(8, 0))

        controls = ttk.Frame(sequence)
        controls.pack(fill="x")
        ttk.Button(controls, text="Add move to selected position",
                   command=self.add_move_action).pack(side="left", padx=(0, 5))
        ttk.Button(controls, text="Add direct X/Y move",
                   command=self.add_direct_move_action).pack(side="left", padx=5)
        ttk.Button(controls, text="Add vacuum action",
                   command=self.add_vacuum_action).pack(side="left", padx=5)
        ttk.Button(controls, text="Add process action",
                   command=self.add_process_action).pack(side="left", padx=5)

        options = ttk.Frame(sequence)
        options.pack(fill="x", pady=(6, 4))
        ttk.Label(options, text="Vacuum:").pack(side="left")
        ttk.Combobox(options, textvariable=self.vacuum_action_var,
                     values=("ON", "OFF"), state="readonly", width=7).pack(
                         side="left", padx=(4, 14)
                     )
        ttk.Label(options, text="Run:").pack(side="left")
        ttk.Combobox(options, textvariable=self.process_action_var,
                     values=("Process 1", "Process 2"), state="readonly",
                     width=12).pack(side="left", padx=4)

        self.action_list = tk.Listbox(sequence, height=8, exportselection=False)
        self.action_list.pack(fill="both", expand=True, pady=4)

        sequence_buttons = ttk.Frame(sequence)
        sequence_buttons.pack(fill="x", pady=(4, 0))
        ttk.Button(sequence_buttons, text="Move action up",
                   command=lambda: self.shift_action(-1)).pack(side="left", padx=(0, 4))
        ttk.Button(sequence_buttons, text="Move action down",
                   command=lambda: self.shift_action(1)).pack(side="left", padx=4)
        ttk.Button(sequence_buttons, text="Remove action",
                   command=self.remove_action).pack(side="left", padx=4)
        ttk.Button(sequence_buttons, text="Clear sequence",
                   command=self.clear_actions).pack(side="left", padx=4)
        ttk.Button(sequence_buttons, text="Load JSON",
                   command=self.load_sequence_dialog).pack(side="right", padx=4)
        ttk.Button(sequence_buttons, text="Save JSON",
                   command=self.save_sequence_dialog).pack(side="right", padx=4)
        self.execute_button = ttk.Button(
            sequence_buttons, text="Execute sequence", command=self.execute_sequence
        )
        self.execute_button.pack(side="right", padx=4)
        self.stop_button = ttk.Button(
            sequence_buttons, text="Stop after current motion", command=self.request_stop
        )
        self.stop_button.pack(side="right", padx=4)
        ttk.Label(outer, textvariable=self.status_var, anchor="w",
                  relief="sunken", padding=5).pack(fill="x", pady=(8, 0))

    # ----------------------------- CONNECTION --------------------------

    def connect_simulation(self):
        if self.connected:
            self.disconnect_simulation()
            return
        try:
            client = ModbusTcpClient(HOST, port=PORT)
            if not client.connect():
                client.close()
                raise ConnectionError(
                    f"Could not connect to {HOST}:{PORT}. "
                    "Start plug_and_produce_simulation.py first."
                )
            # Verify that basic position registers are readable.
            self._read(client, AT_X)
            self._read(client, AT_Y)
            self.client = client
            self.connected = True
            self.connection_var.set("Connected")
            self.connect_button.configure(text="Disconnect")
            self.status_var.set("Connected to the simulation.")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc), parent=self.root)
            self.status_var.set("Connection failed.")

    def disconnect_simulation(self):
        if self._auto_active():
            messagebox.showwarning("Controller active", "Wait until automatic production stops before disconnecting.", parent=self.root)
            return
        if self.busy:
            messagebox.showwarning(
                "Sequence running",
                "Stop the sequence and wait for it to stop before disconnecting.",
                parent=self.root
            )
            return
        if self.client:
            self.client.close()
        self.client = None
        self.client_lock = threading.Lock()
        self.connected = False
        self.connection_var.set("Disconnected")
        self.connect_button.configure(text="Connect")
        self.current_x_var.set("X: --")
        self.current_y_var.set("Y: --")
        self.vacuum_var.set("Vacuum: --")
        self.status_var.set("Disconnected from the simulation.")

    def _read(self, client, address):
        # Serialize requests because the live-status poll and movement worker
        # share one Modbus TCP client.
        with self.client_lock:
            result = client.read_holding_registers(
                address=address, count=1, slave=SLAVE_ID
            )
        if result.isError():
            raise RuntimeError(f"Could not read Modbus register {address}.")
        return result.registers[0]

    def _write(self, client, address, value):
        with self.client_lock:
            result = client.write_register(
                address=address, value=int(value), slave=SLAVE_ID
            )
        if result.isError():
            raise RuntimeError(
                f"Could not write value {value} to register {address}."
            )

    def _require_connection(self):
        if not self.connected or self.client is None:
            raise RuntimeError("Connect to the simulation first.")
        return self.client

    def _poll_status(self):
        if self.connected and self.client:
            try:
                self.station_var.set("S1: {}   S2: {}   P1: {}   P2: {}".format(
                    self._read(self.client, SOURCE1_SENSOR), self._read(self.client, SOURCE2_SENSOR),
                    self._read(self.client, P1_SENSOR), self._read(self.client, P2_SENSOR)))
                x = self._read(self.client, AT_X)
                y = self._read(self.client, AT_Y)
                vacuum = self._read(self.client, VACUUM)
                self.current_x_var.set(f"X: {x}")
                self.current_y_var.set(f"Y: {y}")
                self.vacuum_var.set(f"Vacuum: {'ON' if vacuum else 'OFF'}")
            except Exception as exc:
                self.status_var.set(f"Status read error: {exc}")
        self._poll_auto()
        self.root.after(300, self._poll_status)

    # ----------------------------- SAFE MOVEMENT -----------------------

    def _check_stop(self):
        if self.stop_requested.is_set():
            raise InterruptedError("Stop requested by user.")

    def _wait_position(self, client, address, target, label):
        started = time.monotonic()
        while time.monotonic() - started < MOVE_TIMEOUT:
            self._check_stop()
            current = self._read(client, address)
            if abs(current - target) <= POSITION_TOLERANCE:
                return
            time.sleep(POLL_INTERVAL)
        raise TimeoutError(f"Timed out waiting for {label}={target}.")

    def _move_to(self, client, target_x=None, target_y=None):
        """Move at safe height before changing X, then move to target Y."""
        self._check_stop()
        current_x = self._read(client, AT_X)
        current_y = self._read(client, AT_Y)

        if target_x is not None and not 0 <= int(target_x) <= 1000:
            raise ValueError("X must be between 0 and 1000.")
        if target_y is not None and not 0 <= int(target_y) <= 300:
            raise ValueError("Y must be between 0 and 300.")

        if target_x is not None and int(target_x) != current_x:
            if current_y != SAFE_HEIGHT:
                self._write(client, SET_Y, SAFE_HEIGHT)
                self._wait_position(client, AT_Y, SAFE_HEIGHT, "Y")
            self._write(client, SET_X, int(target_x))
            self._wait_position(client, AT_X, int(target_x), "X")

        if target_y is not None:
            target_y = int(target_y)
            current_y = self._read(client, AT_Y)
            # Use Y=120 as an approach offset before moving down to Y=82.
            # This is required by the project instructions to avoid collisions.
            if target_y < APPROACH_Y and current_y != target_y:
                if current_y != APPROACH_Y:
                    self._write(client, SET_Y, APPROACH_Y)
                    self._wait_position(client, AT_Y, APPROACH_Y, "Y")
            current_y = self._read(client, AT_Y)
            if target_y != current_y:
                self._write(client, SET_Y, target_y)
                self._wait_position(client, AT_Y, target_y, "Y")

    def _hold_current_position(self, client):
        """Ask the crane to hold its current coordinates without releasing a part."""
        try:
            x = self._read(client, AT_X)
            y = self._read(client, AT_Y)
            self._write(client, SET_X, x)
            self._write(client, SET_Y, y)
        except Exception:
            pass

    def move_manual(self):
        if self._auto_active():
            messagebox.showwarning("Controller active", "Automatic controller owns the crane. Stop it first.", parent=self.root)
            return
        try:
            client = self._require_connection()
            x = int(self.manual_x_var.get())
            y = int(self.manual_y_var.get())
            self._start_worker("Manual movement", lambda: self._move_to(client, x, y))
        except Exception as exc:
            messagebox.showerror("Cannot move crane", str(exc), parent=self.root)

    def set_vacuum(self, value):
        if self._auto_active():
            messagebox.showwarning("Controller active", "Automatic controller owns the crane. Stop it first.", parent=self.root)
            return
        try:
            client = self._require_connection()
            if self.busy:
                raise RuntimeError("Wait until the current movement/sequence finishes.")
            self._write(client, VACUUM, value)
            self.status_var.set(f"Vacuum set to {'ON' if value else 'OFF'}.")
        except Exception as exc:
            messagebox.showerror("Vacuum action failed", str(exc), parent=self.root)

    # ----------------------------- POSITIONS ---------------------------

    def _load_positions_startup(self):
        if POSITIONS_FILE.exists():
            try:
                data = json.loads(POSITIONS_FILE.read_text(encoding="utf-8"))
                positions = data.get("positions", data)
                self._validate_positions(positions)
                return positions
            except Exception:
                # Keep the HMI usable; the user can load or save a corrected file.
                pass
        return {name: coords.copy() for name, coords in DEFAULT_POSITIONS.items()}

    @staticmethod
    def _validate_positions(positions):
        if not isinstance(positions, dict):
            raise ValueError("Positions JSON must contain an object of named positions.")
        for name, coords in positions.items():
            if not isinstance(name, str) or not name.strip():
                raise ValueError("Every saved position needs a non-empty name.")
            if not isinstance(coords, dict) or "x" not in coords or "y" not in coords:
                raise ValueError(f"Position '{name}' must contain x and y.")
            x, y = int(coords["x"]), int(coords["y"])
            if not 0 <= x <= 1000 or not 0 <= y <= 300:
                raise ValueError(f"Position '{name}' is outside X=0..1000, Y=0..300.")

    def _refresh_position_list(self, select_name=None):
        self.position_list.delete(0, tk.END)
        for name in sorted(self.positions):
            point = self.positions[name]
            self.position_list.insert(tk.END, f"{name}  (X={point['x']}, Y={point['y']})")
        if select_name is not None:
            names = sorted(self.positions)
            if select_name in names:
                idx = names.index(select_name)
                self.position_list.selection_set(idx)
                self.position_list.see(idx)
                self.position_name_var.set(select_name)

    def _selected_position_name(self):
        selection = self.position_list.curselection()
        if not selection:
            name = self.position_name_var.get().strip()
            if name in self.positions:
                return name
            raise ValueError("Select a saved position first.")
        line = self.position_list.get(selection[0])
        return line.split("  (X=", 1)[0]

    def _on_position_select(self, _event=None):
        try:
            name = self._selected_position_name()
            self.position_name_var.set(name)
            point = self.positions[name]
            self.position_x_var.set(str(point["x"]))
            self.position_y_var.set(str(point["y"]))
        except Exception:
            pass

    def save_edited_position(self):
        """Save typed coordinates to the selected position; this does not move the crane."""
        try:
            name = self._selected_position_name()
            x = int(self.position_x_var.get().strip())
            y = int(self.position_y_var.get().strip())
            if not 0 <= x <= 1000 or not 0 <= y <= 300:
                raise ValueError("X must be 0..1000 and Y must be 0..300.")
            if not messagebox.askyesno(
                "Save position coordinates",
                f"Save '{name}' as X={x}, Y={y}? This changes the saved position only; it does not move the crane.",
                parent=self.root
            ):
                return
            self.positions[name] = {"x": x, "y": y}
            self._refresh_position_list(name)
            self.position_x_var.set(str(x))
            self.position_y_var.set(str(y))
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(f"Saved '{name}' as X={x}, Y={y}; crane not moved.")
        except Exception as exc:
            messagebox.showerror("Save edited position failed", str(exc), parent=self.root)

    def load_positions_dialog(self):
        path = filedialog.askopenfilename(
            title="Load named positions",
            initialdir=str(BASE_DIR),
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            positions = data.get("positions", data) if isinstance(data, dict) else data
            self._validate_positions(positions)
            if not messagebox.askyesno(
                "Load positions",
                f"Load {len(positions)} positions from this file and replace the current HMI position list?",
                parent=self.root
            ):
                return
            self.positions = {
                name: {"x": int(point["x"]), "y": int(point["y"])}
                for name, point in positions.items()
            }
            self.position_name_var.set("")
            self.position_x_var.set("")
            self.position_y_var.set("")
            self._refresh_position_list()
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(f"Loaded {len(self.positions)} positions from {path}.")
        except Exception as exc:
            messagebox.showerror("Load positions failed", str(exc), parent=self.root)

    def save_current_position(self):
        try:
            client = self._require_connection()
            if self.busy:
                raise RuntimeError("Wait until the current movement/sequence finishes.")
            name = self.position_name_var.get().strip()
            if not name:
                raise ValueError("Enter a name for the position.")
            x = self._read(client, AT_X)
            y = self._read(client, AT_Y)
            self.positions[name] = {"x": x, "y": y}
            self._refresh_position_list(name)
            self.position_x_var.set(str(x))
            self.position_y_var.set(str(y))
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(f"Saved '{name}' at X={x}, Y={y}.")
        except Exception as exc:
            messagebox.showerror("Save position failed", str(exc), parent=self.root)

    def update_selected_position(self):
        try:
            client = self._require_connection()
            if self.busy:
                raise RuntimeError("Wait until the current movement/sequence finishes.")
            name = self._selected_position_name()
            x = self._read(client, AT_X)
            y = self._read(client, AT_Y)
            self.positions[name] = {"x": x, "y": y}
            self._refresh_position_list(name)
            self.position_x_var.set(str(x))
            self.position_y_var.set(str(y))
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(f"Updated '{name}' to current X={x}, Y={y}.")
        except Exception as exc:
            messagebox.showerror("Update position failed", str(exc), parent=self.root)

    def move_to_selected_position(self):
        if self._auto_active():
            messagebox.showwarning("Controller active", "Automatic controller owns the crane. Stop it first.", parent=self.root)
            return
        try:
            client = self._require_connection()
            name = self._selected_position_name()
            point = self.positions[name]
            self._start_worker(
                f"Move to {name}",
                lambda: self._move_to(client, point["x"], point["y"])
            )
        except Exception as exc:
            messagebox.showerror("Move failed", str(exc), parent=self.root)

    def delete_selected_position(self):
        try:
            name = self._selected_position_name()
            if not messagebox.askyesno(
                "Delete position", f"Delete saved position '{name}'?", parent=self.root
            ):
                return
            del self.positions[name]
            self.position_name_var.set("")
            self.position_x_var.set("")
            self.position_y_var.set("")
            self._refresh_position_list()
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(f"Deleted position '{name}'.")
        except Exception as exc:
            messagebox.showerror("Delete position failed", str(exc), parent=self.root)

    def _write_positions(self, path):
        self._validate_positions(self.positions)
        path.write_text(
            json.dumps({"positions": self.positions}, indent=2) + "\n",
            encoding="utf-8"
        )

    def restore_default_positions(self):
        """Restore standard Grade A positions without moving the physical crane."""
        if not messagebox.askyesno(
            "Restore default positions",
            "Replace the current saved-position list with the standard Grade A positions? "
            "Custom positions will be removed from the list. The crane will NOT move.",
            parent=self.root,
        ):
            return

        try:
            self.positions = {
                name: coords.copy() for name, coords in DEFAULT_POSITIONS.items()
            }
            self.position_name_var.set("")
            self.position_x_var.set("")
            self.position_y_var.set("")
            self._refresh_position_list()
            self._write_positions(POSITIONS_FILE)
            self.status_var.set(
                "Default positions restored and saved. Sink Release is X=945, Y=82. Crane not moved."
            )
        except Exception as exc:
            messagebox.showerror(
                "Restore defaults failed", str(exc), parent=self.root
            )

    def save_positions_dialog(self):
        path = filedialog.asksaveasfilename(
            title="Save positions JSON",
            initialfile=POSITIONS_FILE.name,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        if path:
            try:
                self._write_positions(Path(path))
                self.status_var.set(f"Positions saved to {path}.")
            except Exception as exc:
                messagebox.showerror("Save positions failed", str(exc), parent=self.root)

    # ----------------------------- SEQUENCE BUILDER --------------------

    def _refresh_action_list(self):
        self.action_list.delete(0, tk.END)
        for i, action in enumerate(self.actions, start=1):
            self.action_list.insert(tk.END, f"{i:02d}. {self._describe_action(action)}")

    @staticmethod
    def _describe_action(action):
        if "position" in action:
            return f"Move to position: {action['position']}"
        if "setX" in action or "setY" in action:
            return f"Move to X={action.get('setX', 'current')}, Y={action.get('setY', 'current')}"
        if "vacuum" in action:
            return f"Vacuum {'ON' if int(action['vacuum']) else 'OFF'}"
        if "startp1" in action:
            return "Run Process 1 and wait for completion"
        if "startp2" in action:
            return "Run Process 2 and wait for completion"
        return f"Unknown action: {action}"

    def add_move_action(self):
        try:
            name = self._selected_position_name()
            self.actions.append({"position": name})
            self._refresh_action_list()
            self.status_var.set(f"Added movement to '{name}' to the sequence.")
        except Exception as exc:
            messagebox.showerror("Add movement failed", str(exc), parent=self.root)

    def add_direct_move_action(self):
        try:
            x = int(self.manual_x_var.get())
            y = int(self.manual_y_var.get())
            if not 0 <= x <= 1000 or not 0 <= y <= 300:
                raise ValueError("X must be 0..1000 and Y must be 0..300.")
            self.actions.append({"setX": x, "setY": y})
            self._refresh_action_list()
        except Exception as exc:
            messagebox.showerror("Add direct move failed", str(exc), parent=self.root)

    def add_vacuum_action(self):
        value = 1 if self.vacuum_action_var.get() == "ON" else 0
        self.actions.append({"vacuum": value})
        self._refresh_action_list()

    def add_process_action(self):
        if self.process_action_var.get() == "Process 1":
            self.actions.append({"startp1": 1, "wait": True})
        else:
            self.actions.append({"startp2": 1, "wait": True})
        self._refresh_action_list()

    def remove_action(self):
        selection = self.action_list.curselection()
        if not selection:
            messagebox.showinfo("Remove action", "Select an action first.", parent=self.root)
            return
        del self.actions[selection[0]]
        self._refresh_action_list()

    def shift_action(self, direction):
        selection = self.action_list.curselection()
        if not selection:
            return
        old = selection[0]
        new = old + direction
        if not 0 <= new < len(self.actions):
            return
        self.actions[old], self.actions[new] = self.actions[new], self.actions[old]
        self._refresh_action_list()
        self.action_list.selection_set(new)

    def clear_actions(self):
        if self.actions and not messagebox.askyesno(
            "Clear sequence", "Remove every action from the sequence?", parent=self.root
        ):
            return
        self.actions.clear()
        self._refresh_action_list()

    def save_sequence_dialog(self):
        path = filedialog.asksaveasfilename(
            title="Save complete sequence JSON",
            initialfile=SEQUENCE_FILE.name,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        if not path:
            return
        try:
            self._validate_actions(self.actions)
            data = {
                "positions": self.positions,
                "actions": self.actions
            }
            Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            self.status_var.set(f"Complete sequence saved to {path}.")
        except Exception as exc:
            messagebox.showerror("Save sequence failed", str(exc), parent=self.root)

    def load_sequence_dialog(self):
        path = filedialog.askopenfilename(
            title="Load sequence JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            actions = data.get("actions")
            if not isinstance(actions, list):
                raise ValueError("The JSON file must contain an 'actions' list.")
            self._validate_actions(actions)
            # Merge named positions from the file, retaining positions already
            # available in the HMI if the sequence file has none.
            loaded_positions = data.get("positions")
            if loaded_positions is not None:
                self._validate_positions(loaded_positions)
                self.positions.update(loaded_positions)
                self._refresh_position_list()
                self._write_positions(POSITIONS_FILE)
            self.actions = actions
            self._refresh_action_list()
            self.status_var.set(f"Loaded {len(actions)} actions from {path}.")
        except Exception as exc:
            messagebox.showerror("Load sequence failed", str(exc), parent=self.root)

    @staticmethod
    def _validate_actions(actions):
        if not isinstance(actions, list):
            raise ValueError("'actions' must be a list.")
        allowed = {"position", "setX", "setY", "vacuum", "startp1", "startp2", "wait"}
        for index, action in enumerate(actions, start=1):
            if not isinstance(action, dict) or not action:
                raise ValueError(f"Action {index} must be a non-empty object.")
            if not set(action).issubset(allowed):
                raise ValueError(f"Action {index} has unknown keys: {action}.")
            if "vacuum" in action and int(action["vacuum"]) not in (0, 1):
                raise ValueError(f"Action {index}: vacuum must be 0 or 1.")
            for key, max_value in (("setX", 1000), ("setY", 300)):
                if key in action and not 0 <= int(action[key]) <= max_value:
                    raise ValueError(f"Action {index}: {key} is outside allowed range.")
            if "position" in action and not isinstance(action["position"], str):
                raise ValueError(f"Action {index}: position must be a name.")
            if "startp1" in action and int(action["startp1"]) not in (0, 1):
                raise ValueError(f"Action {index}: startp1 must be 0 or 1.")
            if "startp2" in action and int(action["startp2"]) not in (0, 1):
                raise ValueError(f"Action {index}: startp2 must be 0 or 1.")

    # ----------------------------- EXECUTION ---------------------------

    def execute_sequence(self):
        if self._auto_active():
            messagebox.showwarning("Controller active", "Automatic controller owns the crane. Stop it first.", parent=self.root)
            return
        try:
            client = self._require_connection()
            if self.busy or self._auto_active():
                raise RuntimeError("A movement or automatic controller is already running.")
            if not self.actions:
                raise RuntimeError("The sequence is empty. Add actions first.")
            self._validate_actions(self.actions)
            for action in self.actions:
                if "position" in action and action["position"] not in self.positions:
                    raise ValueError(
                        f"Action refers to missing saved position '{action['position']}'."
                    )
            if not messagebox.askyesno(
                "Execute sequence",
                f"Execute {len(self.actions)} actions? Ensure the crane path is clear.",
                parent=self.root
            ):
                return
            actions_snapshot = [dict(action) for action in self.actions]
            positions_snapshot = {k: dict(v) for k, v in self.positions.items()}
            self.stop_requested.clear()
            self._start_worker(
                "Sequence",
                lambda: self._execute_actions(client, actions_snapshot, positions_snapshot)
            )
        except Exception as exc:
            messagebox.showerror("Cannot execute sequence", str(exc), parent=self.root)

    def _execute_actions(self, client, actions, positions):
        for index, action in enumerate(actions, start=1):
            self._check_stop()
            self._set_status_threadsafe(f"Executing action {index}/{len(actions)}: {self._describe_action(action)}")

            if "position" in action:
                point = positions[action["position"]]
                self._move_to(client, point["x"], point["y"])
            else:
                # Compatible with the Grade E JSON shape: setX/setY/vacuum.
                if "setX" in action or "setY" in action:
                    self._move_to(client, action.get("setX"), action.get("setY"))
                if "vacuum" in action:
                    self._check_stop()
                    self._write(client, VACUUM, int(action["vacuum"]))
                    time.sleep(VACUUM_SETTLE_TIME)
                if "startp1" in action and int(action["startp1"]) == 1:
                    self._run_process(client, START_P1, P1_RUNNING, P1_SENSOR, "Process 1")
                if "startp2" in action and int(action["startp2"]) == 1:
                    self._run_process(client, START_P2, P2_RUNNING, P2_SENSOR, "Process 2")

        self._set_status_threadsafe("Sequence completed successfully.")

    def _run_process(self, client, start_register, running_register, sensor_register, name):
        self._check_stop()
        if self._read(client, sensor_register) != 1:
            raise RuntimeError(f"{name} cannot start: no part is detected at that station.")
        if self._read(client, running_register) != 0:
            raise RuntimeError(f"{name} is already running.")
        self._write(client, start_register, 1)

        started = time.monotonic()
        while time.monotonic() - started < PROCESS_START_TIMEOUT:
            self._check_stop()
            if self._read(client, running_register) == 1:
                break
            time.sleep(POLL_INTERVAL)
        else:
            raise TimeoutError(f"{name} did not start.")

        while time.monotonic() - started < PROCESS_TIMEOUT + PROCESS_START_TIMEOUT:
            self._check_stop()
            running = self._read(client, running_register)
            start_signal = self._read(client, start_register)
            if running == 0 and start_signal == 0:
                return
            time.sleep(POLL_INTERVAL)
        raise TimeoutError(f"{name} did not finish within the expected time.")

    def _start_worker(self, description, function):
        if self.busy or self._auto_active():
            raise RuntimeError("A movement or automatic controller is already running.")
        self.busy = True
        self._set_controls_enabled(False)
        self.stop_requested.clear()

        def run():
            try:
                function()
                self.root.after(0, lambda: self.status_var.set(f"{description}: completed."))
            except InterruptedError as exc:
                if self.client:
                    self._hold_current_position(self.client)
                self.root.after(0, lambda: self.status_var.set(f"{description}: stopped. {exc}"))
            except Exception as exc:
                if self.client:
                    self._hold_current_position(self.client)
                self.root.after(0, lambda e=exc: messagebox.showerror(
                    "HMI action failed", str(e), parent=self.root
                ))
                self.root.after(0, lambda e=exc: self.status_var.set(f"{description}: failed — {e}"))
            finally:
                self.busy = False
                self.root.after(0, lambda: self._set_controls_enabled(True))

        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        self.status_var.set(f"{description}: running...")

    def _set_status_threadsafe(self, text):
        self.root.after(0, lambda: self.status_var.set(text))

    def request_stop(self):
        if not self.busy:
            self.status_var.set("No movement or sequence is running.")
            return
        self.stop_requested.set()
        self.status_var.set(
            "Stop requested. The controller will stop at the next safe checkpoint; "
            "vacuum will not be switched off automatically."
        )

    def _set_controls_enabled(self, enabled):
        state = "normal" if enabled and not self._auto_active() else "disabled"
        # Avoid simultaneous manual and automatic control.
        for widget in (
            self.move_button, self.vacuum_on_button, self.vacuum_off_button,
            self.execute_button
        ):
            widget.configure(state=state)
        self.connect_button.configure(state="disabled" if self.busy or self._auto_active() else "normal")

    # ----------------------------- AUTOMATIC PRODUCTION ------------------

    def _auto_active(self):
        return self.auto_controller_proc is not None and self.auto_controller_proc.poll() is None

    def launch_auto_simulator(self):
        if self.connected or self.busy or self._auto_active():
            messagebox.showwarning("Simulator", "Disconnect the HMI and stop other controllers first.", parent=self.root)
            return
        if self.auto_sim_proc and self.auto_sim_proc.poll() is None:
            messagebox.showinfo("Simulator", "The simulator is already running.", parent=self.root)
            return
        try:
            limit = int(self.auto_limit_var.get())
            interval = float(self.auto_interval_var.get())
            if not 1 <= limit <= 100 or not 1 <= interval <= 3600:
                raise ValueError("Parts must be 1..100 and interval 1..3600 seconds.")
            path = BASE_DIR / "plug_and_produce_simulation_auto.py"
            if not path.is_file():
                raise FileNotFoundError(str(path))
            self.auto_sim_proc = subprocess.Popen(
                [sys.executable, str(path), "--port", str(PORT), "--auto-generate", str(limit),
                 "--auto-interval", str(interval)], cwd=str(BASE_DIR))
            self.auto_status_var.set("Simulator launched. Wait for it to open, then click Connect.")
        except Exception as exc:
            messagebox.showerror("Simulator launch failed", str(exc), parent=self.root)

    def start_auto_controller(self):
        if self.busy or self._auto_active():
            messagebox.showwarning("Controller", "Another operation is already running.", parent=self.root)
            return
        if not self.connected:
            messagebox.showwarning("Controller", "Connect to the simulator first.", parent=self.root)
            return
        path = BASE_DIR / "grade_b_auto_controller.py"
        if not path.is_file():
            messagebox.showerror("Controller missing", f"Missing {path}", parent=self.root)
            return
        if not messagebox.askyesno("Start automatic production",
            "Make sure no other crane controller is running, processes are empty, and vacuum is OFF. "
            "Start the Type 1 / Type 2 controller?", parent=self.root):
            return
        try:
            if any(self._read(self.client, a) for a in (P1_SENSOR, P2_SENSOR, P1_RUNNING, P2_RUNNING, VACUUM)):
                raise RuntimeError("Process stations or vacuum are occupied. Inspect the cell before starting.")
            stop_path = BASE_DIR / "grade_b_stop.request"
            stop_path.unlink(missing_ok=True)
            self.auto_log_handle = self.auto_output_file.open("w", encoding="utf-8")
            self.auto_run_start = __import__('datetime').datetime.now().isoformat(timespec="seconds")
            self.auto_controller_proc = subprocess.Popen(
                [sys.executable, "-u", str(path)], cwd=str(BASE_DIR),
                stdout=self.auto_log_handle, stderr=subprocess.STDOUT)
            self.auto_status_var.set("Automatic controller running")
            self._set_controls_enabled(False)
        except Exception as exc:
            messagebox.showerror("Controller start failed", str(exc), parent=self.root)

    def stop_auto_controller(self):
        if not self._auto_active():
            self.auto_status_var.set("No automatic controller running")
            return
        (BASE_DIR / "grade_b_stop.request").write_text("stop after transfer\n", encoding="utf-8")
        self.auto_status_var.set("Stop requested; waiting for current transfer to finish")

    def _poll_auto(self):
        if self.auto_controller_proc is not None:
            if self.auto_controller_proc.poll() is None:
                self._set_controls_enabled(False)
            else:
                code = self.auto_controller_proc.returncode
                self.auto_controller_proc = None
                if self.auto_log_handle:
                    self.auto_log_handle.close()
                    self.auto_log_handle = None
                self.auto_status_var.set(f"Automatic controller exited (code {code}). See production log.")
                self._set_controls_enabled(not self.busy)
        if self.auto_output_file.exists():
            try:
                lines = self.auto_output_file.read_text(encoding="utf-8", errors="replace").splitlines()
                meaningful = [line for line in lines if line.strip()]
                if meaningful and self._auto_active():
                    self.auto_status_var.set("Automatic: " + meaningful[-1][:130])
                if self.auto_run_start:
                    deliveries = sum("Delivered " in line and "to the Sink" in line for line in meaningful)
                    self.delivered_var.set(f"Delivered this run: {deliveries}")
            except OSError:
                pass

    def open_auto_csv(self):
        if not self.auto_controller_log.exists():
            messagebox.showinfo("CSV", "No production CSV exists yet.", parent=self.root)
            return
        try:
            if sys.platform == "darwin":
                subprocess.Popen(["open", str(self.auto_controller_log)])
            elif sys.platform.startswith("win"):
                os.startfile(str(self.auto_controller_log))
            else:
                subprocess.Popen(["xdg-open", str(self.auto_controller_log)])
        except Exception as exc:
            messagebox.showerror("Open CSV", str(exc), parent=self.root)

    def _on_close(self):
        if self._auto_active():
            messagebox.showwarning("Automatic production active",
                "Request Stop after transfer and wait until the controller exits before closing. "
                "Do not close the simulator while parts are moving.", parent=self.root)
            return
        if self.busy:
            if not messagebox.askyesno(
                "Close HMI",
                "A movement or sequence is active. Request a stop and close the HMI?",
                parent=self.root
            ):
                return
            self.stop_requested.set()
            self.root.after(500, self._finish_close)
        else:
            self._finish_close()

    def _finish_close(self):
        if self.client:
            self.client.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = GradeAHMI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
