import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time

class MatrixCalculatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("FPGA Matrix Calculator UI (EGO1)")
        self.root.geometry("700x600")

        # --- Connection Frame ---
        self.port_frame = ttk.LabelFrame(root, text="Connection")
        self.port_frame.pack(fill="x", padx=5, pady=5)
        
        self.port_label = ttk.Label(self.port_frame, text="Port:")
        self.port_label.pack(side="left", padx=5)
        
        self.port_combo = ttk.Combobox(self.port_frame, width=10)
        self.port_combo.pack(side="left", padx=5)
        self.refresh_ports()
        
        self.connect_btn = ttk.Button(self.port_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)
        
        self.refresh_btn = ttk.Button(self.port_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", padx=5)

        self.ser = None
        self.is_connected = False

        # --- Settings Frame ---
        self.settings_frame = ttk.LabelFrame(root, text="Display Settings")
        self.settings_frame.pack(fill="x", padx=5, pady=5)

        # Data Width Selection
        ttk.Label(self.settings_frame, text="Data Width:").pack(side="left", padx=5)
        self.data_width_var = tk.StringVar(value="8-bit")
        self.width_combo = ttk.Combobox(self.settings_frame, textvariable=self.data_width_var, values=["8-bit", "16-bit"], state="readonly", width=8)
        self.width_combo.pack(side="left", padx=5)
        
        # Column Wrapping
        ttk.Label(self.settings_frame, text="Matrix Columns (for wrap):").pack(side="left", padx=5)
        self.cols_var = tk.StringVar(value="0")
        self.cols_entry = ttk.Entry(self.settings_frame, textvariable=self.cols_var, width=5)
        self.cols_entry.pack(side="left", padx=5)
        ttk.Label(self.settings_frame, text="(0 = no wrap)").pack(side="left", padx=0)

        # View Mode
        ttk.Label(self.settings_frame, text="View Mode:").pack(side="left", padx=5)
        self.view_mode_var = tk.StringVar(value="ASCII")
        self.view_mode_combo = ttk.Combobox(self.settings_frame, textvariable=self.view_mode_var, values=["ASCII", "Raw Hex"], state="readonly", width=10)
        self.view_mode_combo.pack(side="left", padx=5)

        # --- Matrix Input Area ---
        self.input_frame = ttk.LabelFrame(root, text="Matrix Input")
        self.input_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.input_label = ttk.Label(self.input_frame, text="Enter Matrix (Dim M, Dim N, Elements...):")
        self.input_label.pack(anchor="w", padx=5)
        
        self.input_text = scrolledtext.ScrolledText(self.input_frame, height=5)
        self.input_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.send_btn = ttk.Button(self.input_frame, text="Send Matrix Data", command=self.send_data)
        self.send_btn.pack(pady=5)

        # --- Output Area ---
        self.output_frame = ttk.LabelFrame(root, text="System Output")
        self.output_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.output_text = scrolledtext.ScrolledText(self.output_frame, height=15, state='disabled', font=("Consolas", 10))
        self.output_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.clear_btn = ttk.Button(self.output_frame, text="Clear Output", command=self.clear_output)
        self.clear_btn.pack(pady=5)

        # Buffer for 16-bit reconstruction
        self.rx_buffer = []
        self.display_counter = 0
        self.parse_state = 0
        self.current_matrix = {}

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combo['values'] = [port.device for port in ports]
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_combo.get()
                if not port: return
                self.ser = serial.Serial(port, 115200, timeout=0.1) # Low timeout for responsive UI
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.start_reading()
                self.log(f"Connected to {port}")
                self.rx_buffer = []
                self.display_counter = 0
            except Exception as e:
                self.log(f"Error: {e}")
        else:
            if self.ser:
                self.ser.close()
            self.is_connected = False
            self.connect_btn.config(text="Connect")
            self.log("Disconnected")

    def send_data(self):
        if not self.is_connected:
            self.log("Error: Not connected")
            return
        
        data_str = self.input_text.get("1.0", tk.END).strip()
        if not data_str:
            return
            
        # Parse input (space or newline separated)
        try:
            values = [int(x) for x in data_str.replace('\n', ' ').split()]
            # Send bytes
            self.ser.write(bytes(values))
            self.log(f"Sent: {values}")
        except ValueError:
            self.log("Error: Invalid input (must be integers)")

    def start_reading(self):
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        while self.is_connected and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    raw_data = self.ser.read(self.ser.in_waiting)
                    self.process_data(raw_data)
                time.sleep(0.05)
            except Exception as e:
                print(f"Read error: {e}")
                break

    def process_data(self, data):
        mode = self.view_mode_var.get()
        
        if mode == "ASCII":
            # Just decode and print
            try:
                # Replace errors to avoid crashing on binary data
                text = data.decode('utf-8', errors='replace')
                # Handle potential newline issues if needed, but usually \n is fine
                self.root.after(0, self.log_output, text)
            except Exception as e:
                print(f"Decode error: {e}")
        else:
            # Raw Hex Dump
            msg = ""
            for byte in data:
                msg += f"{byte:02X} "
            self.root.after(0, self.log_output, msg)

    def log(self, msg):
        self.output_text.config(state='normal')
        self.output_text.insert(tk.END, msg + "\n")
        self.output_text.see(tk.END)
        self.output_text.config(state='disabled')

    def log_output(self, msg):
        self.output_text.config(state='normal')
        self.output_text.insert(tk.END, msg)
        self.output_text.see(tk.END)
        self.output_text.config(state='disabled')

    def clear_output(self):
        self.output_text.config(state='normal')
        self.output_text.delete("1.0", tk.END)
        self.output_text.config(state='disabled')
        self.display_counter = 0
        self.rx_buffer = []

if __name__ == "__main__":
    root = tk.Tk()
    app = MatrixCalculatorGUI(root)
    root.mainloop()
