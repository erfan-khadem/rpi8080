import sys
import serial
import argparse
import os
import threading
import time
from PyQt6 import QtWidgets, QtCore
import serial.tools.list_ports

MEMORY_SIZE = 65536

def get_device_version(ser_port, callback=None, error_callback=None):
    try:
        ser = serial.Serial(ser_port, 921600, timeout=0.5)
        end_val = (1 << 32) - 1
        ser.write(f"{1:08x} {end_val:08x}\r\n".encode())
        res = ser.readline().decode().strip()
        ser.close()
        if callback:
            callback(res)
    except serial.SerialException as e:
        error_msg = f"Error communicating with serial port {ser_port}: {e}"
        print(error_msg, file=sys.stderr)
        if error_callback:
            error_callback(error_msg)
    except Exception as e:
        error_msg = f"Unexpected error: {e}"
        print(error_msg, file=sys.stderr)
        if error_callback:
            error_callback(error_msg)

def program_device(ser_port, file_path, progress_callback=None, error_callback=None, done_callback=None):
    try:
        ser = serial.Serial(ser_port, 921600, timeout=0.1)
    except serial.SerialException as e:
        error_msg = f"Error opening serial port {ser_port}: {e}"
        print(error_msg, file=sys.stderr)
        if error_callback:
            error_callback(error_msg)
        return

    try:
        with open(file_path, 'rb') as hfile:
            bf = hfile.read()
            if len(bf) != MEMORY_SIZE:
                error_msg = f"Input file size does not match storage size: given {len(bf)} bytes, wanted {MEMORY_SIZE} bytes"
                raise ValueError(error_msg)
    except FileNotFoundError:
        error_msg = f"File not found: {file_path}"
        print(error_msg, file=sys.stderr)
        ser.close()
        if error_callback:
            error_callback(error_msg)
        return
    except ValueError as e:
        print(e, file=sys.stderr)
        ser.close()
        if error_callback:
            error_callback(str(e))
        return

    total_chunks = MEMORY_SIZE // 4
    chunks_written = 0

    for i in range(0, MEMORY_SIZE, 4):
        data = bf[i]
        data += bf[i+1] << 8
        data += bf[i+2] << 16
        data += bf[i+3] << 24

        if data == 0:
            continue

        ser.write(f"{data:08x} {i:08x}\r\n".encode())
        chk = data ^ i
        res = ser.read(10)
        if res != (f"{chk:08x}\r\n").encode():
            error_msg = f"Error at address {i:08x}: expected {chk:08x}, got {res.decode().strip()}"
            print(error_msg, file=sys.stderr)
            ser.close()
            if error_callback:
                error_callback(error_msg)
            return

        chunks_written += 1
        if progress_callback:
            progress_percent = (chunks_written / total_chunks) * 100
            progress_callback(progress_percent)

    end_val = (1 << 32) - 1
    ser.write(f"{end_val:08x} {end_val:08x}\r\n".encode())
    ser.close()
    print("Programming Done!", file=sys.stderr)
    if done_callback:
        done_callback()

def launch_gui():
    class ProgrammerGUI(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.init_ui()

        def init_ui(self):
            self.setWindowTitle('Device Programmer')

            layout = QtWidgets.QVBoxLayout()

            # Serial port selection
            port_group = QtWidgets.QGroupBox('Serial Port')
            port_layout = QtWidgets.QHBoxLayout()
            self.port_combo = QtWidgets.QComboBox()
            self.port_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
            self.refresh_ports()
            port_layout.addWidget(self.port_combo)
            refresh_button = QtWidgets.QPushButton('Refresh')
            refresh_button.clicked.connect(self.refresh_ports)
            port_layout.addWidget(refresh_button)
            port_group.setLayout(port_layout)
            layout.addWidget(port_group)

            # File selection
            file_group = QtWidgets.QGroupBox('Binary File')
            file_layout = QtWidgets.QHBoxLayout()
            self.file_edit = QtWidgets.QLineEdit()
            file_layout.addWidget(self.file_edit)
            browse_button = QtWidgets.QPushButton('Browse')
            browse_button.clicked.connect(self.browse_file)
            file_layout.addWidget(browse_button)
            file_group.setLayout(file_layout)
            layout.addWidget(file_group)

            # Buttons layout
            buttons_layout = QtWidgets.QHBoxLayout()

            # Get Version button
            self.version_button = QtWidgets.QPushButton('Get Device Version')
            self.version_button.clicked.connect(self.get_version)
            buttons_layout.addWidget(self.version_button)

            # Start Programming button
            self.start_button = QtWidgets.QPushButton('Start Programming')
            self.start_button.clicked.connect(self.start_programming)
            buttons_layout.addWidget(self.start_button)

            layout.addLayout(buttons_layout)

            # Progress bar
            self.progress_bar = QtWidgets.QProgressBar()
            layout.addWidget(self.progress_bar)

            # Status label
            self.status_label = QtWidgets.QLabel('')
            layout.addWidget(self.status_label)

            self.setLayout(layout)
            self.resize(400, 250)

        def refresh_ports(self):
            self.port_combo.clear()
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo.addItems(ports)

        def browse_file(self):
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Select binary file', '', 'Binary files (*.bin);;All files (*.*)')
            if file_path:
                self.file_edit.setText(file_path)

        def start_programming(self):
            ser_port = self.port_combo.currentText()
            file_path = self.file_edit.text()

            if not ser_port:
                QtWidgets.QMessageBox.critical(self, 'Error', 'Please select a serial port.')
                return

            if not file_path:
                QtWidgets.QMessageBox.critical(self, 'Error', 'Please select a binary file.')
                return

            self.progress_bar.setValue(0)
            self.status_label.setText('Programming started...')
            self.start_button.setEnabled(False)
            self.version_button.setEnabled(False)
            threading.Thread(target=program_device, args=(ser_port, file_path, self.update_progress, self.show_error, self.program_done)).start()

        def update_progress(self, value):
            QtCore.QMetaObject.invokeMethod(self.progress_bar, "setValue", QtCore.Q_ARG(int, int(value)))

        def show_error(self, message):
            def inner():
                self.start_button.setEnabled(True)
                self.version_button.setEnabled(True)
                QtWidgets.QMessageBox.critical(self, 'Error', message)
                self.status_label.setText('Error occurred.')
            QtCore.QMetaObject.invokeMethod(self, "invoke", QtCore.Q_ARG('PyQt_PyObject', inner))

        def program_done(self):
            def inner():
                self.start_button.setEnabled(True)
                self.version_button.setEnabled(True)
                self.status_label.setText('Programming Done!')
                QtWidgets.QMessageBox.information(self, 'Success', 'Programming Done!')
            QtCore.QMetaObject.invokeMethod(self, "invoke", QtCore.Q_ARG('PyQt_PyObject', inner))

        def get_version(self):
            ser_port = self.port_combo.currentText()
            if not ser_port:
                QtWidgets.QMessageBox.critical(self, 'Error', 'Please select a serial port.')
                return
            self.status_label.setText('Getting device version...')
            self.version_button.setEnabled(False)
            self.start_button.setEnabled(False)
            threading.Thread(target=get_device_version, args=(ser_port, self.show_version, self.show_error)).start()

        def show_version(self, version):
            def inner():
                self.version_button.setEnabled(True)
                self.start_button.setEnabled(True)
                self.status_label.setText(f'Device Version: {version}')
                QtWidgets.QMessageBox.information(self, 'Device Version', f'Device Version: {version}')
            QtCore.QMetaObject.invokeMethod(self, "invoke", QtCore.Q_ARG('PyQt_PyObject', inner))

        @QtCore.pyqtSlot('PyQt_PyObject')
        def invoke(self, func):
            func()

    app = QtWidgets.QApplication(sys.argv)
    gui = ProgrammerGUI()
    gui.show()
    sys.exit(app.exec())

def main():
    parser = argparse.ArgumentParser(description='Program a device via serial port.')
    parser.add_argument('serial_port', nargs='?', help='Serial port to use.')
    parser.add_argument('file', nargs='?', help='Binary file to program.')
    parser.add_argument('--cli', action='store_true', help='Use command-line interface instead of GUI.')
    parser.add_argument('--get-version', action='store_true', help='Get device version.')

    args = parser.parse_args()

    if not args.cli:
        launch_gui()
    else:
        if args.get_version:
            if args.serial_port:
                ser_port = args.serial_port
            else:
                ser_port = 'COM1' if os.name == 'nt' else '/dev/ttyACM0'
            get_device_version(ser_port, callback=lambda v: print(f"Device Version: {v}"), error_callback=lambda e: print(f"Error: {e}", file=sys.stderr))
            return

        if args.file:
            file_path = args.file
            if args.serial_port:
                ser_port = args.serial_port
            else:
                ser_port = 'COM1' if os.name == 'nt' else '/dev/ttyACM0'
        elif args.serial_port:
            print("Binary file not specified.", file=sys.stderr)
            parser.print_help()
            return
        else:
            print("Usage:", file=sys.stderr)
            parser.print_help()
            return

        program_device(ser_port, file_path)

if __name__ == "__main__":
    main()

