"""
OCR A-Level Computer Science NEA
Robotic Arm Control System - Stage 1 Implementation
Candidate: Troy Okoji
Candidate Number: 4088
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
from vpython import canvas, cylinder, box, vector, color, rate
from datetime import datetime
import time

DEFAULT_BAUD_RATE = 9600
SERVO_MIN = 0
SERVO_MAX = 180


class RoboticControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control")
        self.root.geometry("1200x800")

        # Serial connection
        self.serial_connection = None
        self.connected_port = None

        # Motor control variables
        self.motor_angles = [tk.DoubleVar(value=90) for _ in range(4)]
        self.motor_angles[0].set(90)  # Base rotation (stepper) - starts at 90°
        self.motor_angles[1].set(90)  # Servo 1
        self.motor_angles[2].set(90)  # Servo 2
        self.motor_angles[3].set(0)  # Claw (now 0=closed, 90=open)

        # Stepper settings (NEMA 17 on CNC shield)
        self.stepper_steps_per_rev = 200
        self.stepper_microsteps = 16
        self.total_steps_per_rev = self.stepper_steps_per_rev * self.stepper_microsteps
        self.max_stepper_angle = 180  # 90° in either direction from center
        self.stepper_min_limit = 0  # Default limits (0-180)
        self.stepper_max_limit = 180
        self.stepper_step_size = 1  # Default step size

        # Playback settings
        self.playback_speed = tk.DoubleVar(value=1.0)
        self.positions_file = "positions.robotpos"
        self.temporary_log = []

        # Initialize UI
        self.init_3d_visualization()
        self.create_main_layout()
        self.create_connection_controls()
        self.create_stepper_limit_controls()
        self.scan_serial_ports()

        # Add credits
        self.add_credits()

    def add_credits(self):
        credit_label = ttk.Label(self.root,
                                 text="Troy Okoji H446 Candidate Number:4088 NEA",
                                 foreground="gray70",
                                 font=('Arial', 10))
        credit_label.pack(side=tk.BOTTOM, pady=5)

    def init_3d_visualization(self):
        self.vp_canvas = canvas(width=600, height=600, background=color.gray(0.2))
        self.vp_canvas.embedded = True
        self.vp_canvas.align = 'left'

        # Base
        self.ground_plane = box(pos=vector(0, -50, 0), size=vector(200, 1, 200), color=color.gray(0.5))
        self.joint1 = cylinder(pos=vector(0, 0, 0), axis=vector(0, 50, 0), radius=5, color=color.blue)

        # Arm segments
        self.joint2 = cylinder(pos=vector(0, 50, 0), axis=vector(50, 0, 0), radius=5, color=color.green)
        self.joint3 = cylinder(pos=vector(50, 50, 0), axis=vector(0, -50, 0), radius=5, color=color.red)

        # Claw
        self.claw_left = box(pos=vector(50, 0, 15), size=vector(10, 10, 40), color=color.orange)
        self.claw_right = box(pos=vector(50, 0, -15), size=vector(10, 10, 40), color=color.orange)

    def create_stepper_limit_controls(self):
        limit_frame = ttk.LabelFrame(self.root, text="Stepper Limits", padding=10)
        limit_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(limit_frame, text="Min Angle:").grid(row=0, column=0, sticky="w")
        self.min_limit_var = tk.IntVar(value=0)
        ttk.Entry(limit_frame, textvariable=self.min_limit_var, width=5).grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(limit_frame, text="Max Angle:").grid(row=0, column=2, sticky="w")
        self.max_limit_var = tk.IntVar(value=180)
        ttk.Entry(limit_frame, textvariable=self.max_limit_var, width=5).grid(row=0, column=3, sticky="w", padx=5)

        ttk.Label(limit_frame, text="Step Size:").grid(row=0, column=4, sticky="w")
        self.step_size_var = tk.DoubleVar(value=1.0)
        ttk.Entry(limit_frame, textvariable=self.step_size_var, width=5).grid(row=0, column=5, sticky="w", padx=5)

        ttk.Button(limit_frame, text="Set Limits", command=self.set_stepper_limits).grid(row=0, column=6, padx=5)

    def set_stepper_limits(self):
        try:
            min_limit = int(self.min_limit_var.get())
            max_limit = int(self.max_limit_var.get())
            step_size = float(self.step_size_var.get())

            if min_limit < 0 or max_limit > 180 or min_limit >= max_limit:
                raise ValueError("Invalid limits")

            if step_size <= 0 or step_size > 10:
                raise ValueError("Step size must be between 0 and 10")

            self.stepper_min_limit = min_limit
            self.stepper_max_limit = max_limit
            self.stepper_step_size = step_size

            # Update current position if it's outside new limits
            current = float(self.motor_angles[0].get())
            if current < min_limit:
                self.motor_angles[0].set(min_limit)
                self.update_motors()
            elif current > max_limit:
                self.motor_angles[0].set(max_limit)
                self.update_motors()

            self.log(f"Stepper limits set: {min_limit}° to {max_limit}° with step size {step_size}°")
        except ValueError as e:
            self.log(f"Error setting limits: {str(e)}")
            messagebox.showerror("Error", f"Invalid limits: {str(e)}")

    def create_main_layout(self):
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 3D Visualization
        vis_frame = tk.Frame(main_frame)
        vis_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Control Panel
        control_frame = tk.Frame(main_frame, padx=10, pady=10)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        self.create_motor_controls(control_frame)
        self.create_action_buttons(control_frame)
        self.create_log_area(control_frame)
        self.create_temporary_log_display(control_frame)

    def create_motor_controls(self, parent):
        motor_frame = ttk.LabelFrame(parent, text="Motor Controls", padding=10)
        motor_frame.pack(fill=tk.X, pady=5)

        labels = ["Base Rotation", "Servo #1 (Pin 9)", "Servo #2 (Pin 10)", "Claw (Pin 11)"]
        ranges = [(0, 180), (0, 180), (0, 180), (0, 90)]  # Claw now 0-90

        for i, (label, (min_val, max_val)) in enumerate(zip(labels, ranges)):
            ttk.Label(motor_frame, text=label).grid(row=i, column=0, sticky="w", pady=2)
            slider = ttk.Scale(motor_frame, from_=min_val, to=max_val, variable=self.motor_angles[i],
                               orient=tk.HORIZONTAL, length=200, command=self.update_motors)
            slider.grid(row=i, column=1, pady=2)
            ttk.Label(motor_frame, textvariable=self.motor_angles[i]).grid(row=i, column=2, padx=5)

            # Add step buttons for base rotation
            if i == 0:
                step_frame = ttk.Frame(motor_frame)
                step_frame.grid(row=i, column=3, padx=5)
                ttk.Button(step_frame, text="+", width=3,
                           command=lambda: self.step_stepper(1)).pack(side=tk.LEFT)
                ttk.Button(step_frame, text="-", width=3,
                           command=lambda: self.step_stepper(-1)).pack(side=tk.LEFT)

    def create_action_buttons(self, parent):
        action_frame = ttk.LabelFrame(parent, text="Actions", padding=10)
        action_frame.pack(fill=tk.X, pady=5)

        buttons = [
            ("Add Position", self.add_to_temporary_log),
            ("Simulate", self.simulate_positions),
            ("Push to Arduino", self.push_positions),
            ("Save Positions", self.save_log_to_file),
            ("Load Positions", self.open_positions_file),
            ("Emergency Stop", self.emergency_stop),
            ("Clear Log", self.clear_temporary_log),
            ("Settings", self.show_help_and_settings)
        ]

        for i, (text, cmd) in enumerate(buttons):
            ttk.Button(action_frame, text=text, command=cmd).grid(row=i // 2, column=i % 2, padx=5, pady=5, sticky="ew")

    def create_log_area(self, parent):
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = tk.Text(log_frame, height=10, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)

        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(fill=tk.Y, side=tk.RIGHT)
        self.log_text.config(yscrollcommand=scrollbar.set)

    def create_temporary_log_display(self, parent):
        temp_frame = ttk.LabelFrame(parent, text="Position Log", padding=10)
        temp_frame.pack(fill=tk.BOTH, pady=5)

        self.temp_log_text = tk.Text(temp_frame, height=5, wrap=tk.WORD)
        self.temp_log_text.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)

        scrollbar = ttk.Scrollbar(temp_frame, command=self.temp_log_text.yview)
        scrollbar.pack(fill=tk.Y, side=tk.RIGHT)
        self.temp_log_text.config(yscrollcommand=scrollbar.set)

    def create_connection_controls(self):
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combobox = ttk.Combobox(conn_frame, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.scan_serial_ports).grid(row=0, column=2)

        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0, sticky="w")
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        ttk.Entry(conn_frame, textvariable=self.baud_var, width=10).grid(row=1, column=1, sticky="w", padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2)

        self.conn_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.conn_status.grid(row=2, column=0, columnspan=3)

    def scan_serial_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)
        self.log(f"Found {len(ports)} serial ports")

    def toggle_connection(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combobox.get()
        if not port:
            self.log("No port selected!")
            return

        try:
            baud_rate = int(self.baud_var.get())
        except ValueError:
            self.log("Invalid baud rate! Using default.")
            baud_rate = DEFAULT_BAUD_RATE
            self.baud_var.set(str(baud_rate))

        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.connected_port = port
            self.connect_btn.config(text="Disconnect")
            self.conn_status.config(text=f"Connected to {port}", foreground="green")
            self.log(f"Connected to {port} at {baud_rate} baud")

            # Send initial positions
            self.update_motors()
        except Exception as e:
            self.log(f"Error connecting to {port}: {str(e)}")
            self.conn_status.config(text="Connection failed", foreground="red")

    def disconnect_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.close()
                self.log(f"Disconnected from {self.connected_port}")
            except Exception as e:
                self.log(f"Error disconnecting: {str(e)}")

        self.serial_connection = None
        self.connected_port = None
        self.connect_btn.config(text="Connect")
        self.conn_status.config(text="Disconnected", foreground="red")

    def update_robot_visual(self):
        try:
            base_angle = float(self.motor_angles[0].get())
            servo1_angle = float(self.motor_angles[1].get())
            servo2_angle = float(self.motor_angles[2].get())
            claw_angle = float(self.motor_angles[3].get())

            base_rad = base_angle * 0.0174533  # deg to rad

            # Base rotation
            self.joint1.axis = vector(0, 50, 0).rotate(angle=base_rad, axis=vector(0, 1, 0))

            # First joint (green arm) - rotates with servo1_angle
            self.joint2.pos = self.joint1.pos + self.joint1.axis
            self.joint2.axis = vector(50, 0, 0).rotate(
                angle=servo1_angle * 0.0174533,
                axis=vector(0, 0, 1)
            ).rotate(angle=base_rad, axis=vector(0, 1, 0))

            # Second joint (red arm) - rotates OPPOSITE to servo2_angle to match real hardware
            self.joint3.pos = self.joint2.pos + self.joint2.axis
            self.joint3.axis = vector(0, -50, 0).rotate(
                angle=(180 - servo2_angle) * 0.0174533,  # Invert the angle to match real movement
                axis=vector(0, 0, 1)
            ).rotate(angle=base_rad, axis=vector(0, 1, 0))

            # Claw - non-linear mapping (0=closed, 90=open)
            claw_pos = self.joint3.pos + self.joint3.axis
            claw_angle_normalized = claw_angle / 90.0  # Normalize to 0-1
            claw_open = 15 * (1 - min(1.0, max(0.0, claw_angle_normalized)))
            self.claw_left.pos = claw_pos + vector(0, 0, claw_open).rotate(
                angle=base_rad, axis=vector(0, 1, 0))
            self.claw_right.pos = claw_pos + vector(0, 0, -claw_open).rotate(
                angle=base_rad, axis=vector(0, 1, 0))
        except Exception as e:
            self.log(f"Visualization error: {str(e)}")

    def update_motors(self, _=None):
        if not (self.serial_connection and self.serial_connection.is_open):
            return

        # Base stepper - apply limits before sending
        base_angle = float(self.motor_angles[0].get())
        base_angle = max(self.stepper_min_limit, min(self.stepper_max_limit, base_angle))
        steps = int((base_angle / 360) * self.total_steps_per_rev)
        self.send_command(f"STEPPER {steps}")

        # Servo 1
        servo1_angle = int(self.motor_angles[1].get())
        servo1_angle = max(SERVO_MIN, min(SERVO_MAX, servo1_angle))

        # Servo 2
        servo2_angle = int(self.motor_angles[2].get())
        servo2_angle = max(SERVO_MIN, min(SERVO_MAX, servo2_angle))

        self.send_command(f"SERVO {servo1_angle} {servo2_angle}")

        # Claw (0-90 range)
        claw_angle = int(self.motor_angles[3].get())
        claw_angle = max(0, min(90, claw_angle))  # Limit to 0-90
        self.send_command(f"CLAW {claw_angle}")

        self.update_robot_visual()

    def step_stepper(self, direction):
        current = float(self.motor_angles[0].get())
        new_val = current + (direction * self.stepper_step_size)

        # Apply limits
        new_val = max(self.stepper_min_limit, min(self.stepper_max_limit, new_val))

        if new_val != current:
            self.motor_angles[0].set(new_val)
            self.update_motors()

    def push_positions(self):
        if not self.temporary_log:
            self.log("No positions to push")
            return

        if not (self.serial_connection and self.serial_connection.is_open):
            self.log("Not connected to Arduino!")
            return

        speed_factor = 1.0 / max(0.1, float(self.playback_speed.get()))
        self.log(f"Starting playback at {1 / speed_factor:.1f}x speed")

        current_angles = [var.get() for var in self.motor_angles]

        for position in self.temporary_log:
            target_angles = position[:]

            steps = 20
            for step in range(steps + 1):
                interp_angles = [
                    current + (target - current) * (step / steps)
                    for current, target in zip(current_angles, target_angles)
                ]

                # Update all motors
                for i, angle in enumerate(interp_angles):
                    self.motor_angles[i].set(angle)

                self.update_motors()
                self.root.update()
                time.sleep(0.05 * speed_factor)

            current_angles = target_angles
            self.log(f"Reached position: {target_angles}")

    def simulate_positions(self):
        if not self.temporary_log:
            self.log("No positions to simulate")
            return

        speed_factor = max(0.1, float(self.playback_speed.get()))
        self.log(f"Starting simulation at {speed_factor:.1f}x speed")

        current_angles = [var.get() for var in self.motor_angles]
        steps = int(30 * speed_factor)

        for position in self.temporary_log:
            for intermediate in self.interpolate_angles(current_angles, position, steps):
                for i, angle in enumerate(intermediate):
                    self.motor_angles[i].set(angle)
                self.update_robot_visual()
                self.root.update()
                rate(60 * speed_factor)
            current_angles = position
            self.log(f"Reached position: {position}")

    def interpolate_angles(self, current, target, steps):
        for step in range(steps + 1):
            interpolated = [
                c + (t - c) * (step / steps)
                for c, t in zip(current, target)
            ]
            yield interpolated

    def add_to_temporary_log(self):
        angles = [var.get() for var in self.motor_angles]
        self.temporary_log.append(angles)
        self.log(f"Added position: {angles}")
        self.update_temporary_log_display()

    def save_log_to_file(self):
        if not self.temporary_log:
            self.log("No positions to save")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".robotpos",
            filetypes=[("Robot Position Files", "*.robotpos"), ("All Files", "*.*")],
            title="Save Positions"
        )
        if file_path:
            try:
                with open(file_path, "w") as f:
                    for pos in self.temporary_log:
                        f.write(f"{pos}\n")
                self.log(f"Saved positions to {file_path}")
            except Exception as e:
                self.log(f"Error saving file: {str(e)}")

    def open_positions_file(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("Robot Position Files", "*.robotpos"), ("All Files", "*.*")],
            title="Open Positions File"
        )
        if file_path:
            try:
                with open(file_path, "r") as f:
                    self.temporary_log = [eval(line.strip()) for line in f.readlines()]
                self.log(f"Loaded {len(self.temporary_log)} positions from {file_path}")
                self.update_temporary_log_display()
            except Exception as e:
                self.log(f"Error loading file: {str(e)}")

    def clear_temporary_log(self):
        self.temporary_log.clear()
        self.log("Cleared temporary log")
        self.update_temporary_log_display()

    def emergency_stop(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.send_command("STOP")
        self.log("EMERGENCY STOP ACTIVATED")

    def show_help_and_settings(self):
        settings = tk.Toplevel(self.root)
        settings.title("Settings and Help")
        settings.geometry("600x500")

        # Help section
        help_frame = ttk.LabelFrame(settings, text="Help", padding=10)
        help_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        help_text = """
        Robotic Arm Control Software

        Controls:
        - Base: Stepper motor (0-180°) Use slider to control motor connected to pin 9
        - Servo 1/2: Arm joints (0-180°) Use slider to control motor connected to pin 10  
        - Claw: Gripper (0=closed, 90=open) Use slider to control motor connected to pin 11

        Playback:
        - Adjust speed below
        - Add positions to log
        - Simulate or push to Arduino
        """
        ttk.Label(help_frame, text=help_text, justify=tk.LEFT).pack(fill=tk.BOTH, expand=True)

        # Settings section
        settings_frame = ttk.LabelFrame(settings, text="Settings", padding=10)
        settings_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        ttk.Label(settings_frame, text="Playback Speed:").grid(row=0, column=0, sticky="w")
        ttk.Scale(settings_frame, from_=0.1, to=5.0, variable=self.playback_speed,
                  orient=tk.HORIZONTAL, length=300).grid(row=0, column=1)
        ttk.Label(settings_frame, textvariable=self.playback_speed).grid(row=0, column=2)

        ttk.Button(settings, text="Close", command=settings.destroy).pack(pady=10)

    def update_temporary_log_display(self):
        self.temp_log_text.delete(1.0, tk.END)
        for pos in self.temporary_log:
            self.temp_log_text.insert(tk.END, f"{pos}\n")
        self.temp_log_text.see(tk.END)

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def send_command(self, command):
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.write(f"{command}\n".encode())
                self.log(f"Sent: {command}")
        except Exception as e:
            self.log(f"Error sending command: {str(e)}")


if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticControlApp(root)
    root.mainloop()
    if app.serial_connection and app.serial_connection.is_open:
        app.serial_connection.close()