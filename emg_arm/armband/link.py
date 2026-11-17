"""Armband (BioWave ESP32-S3) link: Wi-Fi discovery/control, USB Wi-Fi
provisioning, EMG stream readers and the Random Forest inference worker.

Copied from ArmBand/code/mouse.py (ArmBand repo, 2026-10-25) so this project runs
on its own. Only the parts the arm app uses are kept; the mouse-control and
Fitts' law code is left out. If the armband firmware protocol changes, update
this file from mouse.py.
"""

import hashlib
import hmac
import socket
import struct
import threading
import time
from dataclasses import dataclass
from urllib.parse import quote

import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox, QDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QVBoxLayout,
)

try:
    from rf_features import extract_window_features
    HAS_RF = True
except ImportError:
    HAS_RF = False

# Wireless & Communication Constants
WIFI_STREAM_PORT = 5000
WIFI_CONTROL_PORT = 5001
DEFAULT_ACCESS_KEY = "CHANGE_THIS_TO_A_LONG_RANDOM_KEY"
DISCOVERY_ADDRESS = "255.255.255.255"
DISCOVERY_TIMEOUT = 1.2
CONTROL_TIMEOUT = 2.0
USB_SERIAL_BAUD = 921600  # Matches the ESP32 USB stream and provisioning port.
SERIAL_BOOT_WAIT_S = 3.5
SERIAL_RESPONSE_TIMEOUT_S = 15.0

WIRELESS_EMG_CHANNELS = 8
WIRELESS_IMU_CHANNELS = 3
WIRELESS_TOTAL_CHANNELS = WIRELESS_EMG_CHANNELS + WIRELESS_IMU_CHANNELS
WIFI_PACKET_HEADER_FORMAT = "<4sBBHI"
WIFI_PACKET_HEADER_SIZE = struct.calcsize(WIFI_PACKET_HEADER_FORMAT)
WIRELESS_FRAME_FORMAT = "<IIII8HfffB3x"
WIRELESS_FRAME_SIZE = struct.calcsize(WIRELESS_FRAME_FORMAT)
WIRELESS_FRAMES_PER_PACKET = 5
WIRELESS_PACKET_SIZE = WIFI_PACKET_HEADER_SIZE + (WIRELESS_FRAME_SIZE * WIRELESS_FRAMES_PER_PACKET)

# --- NETWORK DATA STRUCTURES & PROTOCOLS ---

@dataclass
class DeviceInfo:
    ip: str
    device_id: str
    device_name: str
    wifi_mode: str
    reported_ip: str
    imu_ready: bool
    streaming: bool
    firmware: str

    @property
    def summary(self):
        return f"{self.device_name} @ {self.ip} ({self.wifi_mode})"


@dataclass
class SerialDeviceInfo:
    port_name: str
    device_id: str
    device_name: str
    imu_ready: bool
    wifi_saved: bool
    firmware: str


def sign_message(secret, *parts):
    message = "|".join(str(part) for part in parts)
    return hmac.new(secret.encode("utf-8"), message.encode("utf-8"), hashlib.sha256).hexdigest()


def get_broadcast_addresses():
    addresses = ["255.255.255.255"]
    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = item[4][0]
            if ip and not ip.startswith("127."):
                parts = ip.split(".")
                if len(parts) == 4:
                    subnet_bc = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
                    if subnet_bc not in addresses:
                        addresses.append(subnet_bc)
    except Exception:
        pass
    for prefix in ["192.168.1", "192.168.0", "192.168.4", "10.0.0"]:
        bc = f"{prefix}.255"
        if bc not in addresses:
            addresses.append(bc)
    return addresses


def get_local_ip_for_target(target_ip):
    # 1. Try UDP probe to target_ip directly
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect((target_ip, 1))
        ip = probe.getsockname()[0]
        probe.close()
        if ip and ip != "0.0.0.0" and not ip.startswith("127."):
            return ip
    except Exception:
        pass

    # 2. Try UDP probe to common DNS/gateway
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        ip = probe.getsockname()[0]
        probe.close()
        if ip and ip != "0.0.0.0" and not ip.startswith("127."):
            return ip
    except Exception:
        pass

    # 3. Hostname resolution
    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = item[4][0]
            if ip and ip != "0.0.0.0" and not ip.startswith("127."):
                return ip
    except Exception:
        pass

    return "127.0.0.1"


class ControlProtocol:
    @staticmethod
    def parse_device_info(message, source_ip):
        parts = message.strip().split("|")
        if len(parts) != 8 or parts[0] != "HELLO":
            raise ValueError("Unexpected device response.")

        return DeviceInfo(
            ip=source_ip,
            device_id=parts[1],
            device_name=parts[2],
            wifi_mode=parts[3],
            reported_ip=parts[4],
            imu_ready=parts[5] == "1",
            streaming=parts[6] == "1",
            firmware=parts[7],
        )

    @staticmethod
    def send_and_receive(message, target_ip, expect_multiple=False, timeout=CONTROL_TIMEOUT, broadcast=False):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2 if expect_multiple else timeout)
        if broadcast:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        responses = []
        deadline = time.monotonic() + timeout
        try:
            if broadcast and target_ip == "255.255.255.255":
                for bc_ip in get_broadcast_addresses():
                    try:
                        sock.sendto(message.encode("utf-8"), (bc_ip, WIFI_CONTROL_PORT))
                    except OSError:
                        pass
            else:
                try:
                    sock.sendto(message.encode("utf-8"), (target_ip, WIFI_CONTROL_PORT))
                except OSError as exc:
                    if exc.errno == 65 or getattr(exc, "errno", None) == 65:
                        raise RuntimeError(f"No route to host ({target_ip}). Check network connection / Wi-Fi subnet.")
                    raise

            if expect_multiple:
                while time.monotonic() < deadline:
                    try:
                        data, addr = sock.recvfrom(2048)
                        responses.append((data.decode("utf-8", errors="replace"), addr[0]))
                    except socket.timeout:
                        continue
                    except OSError:
                        pass
                return responses

            data, addr = sock.recvfrom(2048)
            return data.decode("utf-8", errors="replace"), addr[0]
        except socket.timeout:
            raise RuntimeError(
                f"Connection timed out ({target_ip}).\n\n"
                "1. Ensure the ESP32 is powered on and connected to Wi-Fi.\n"
                "2. Verify the Access Key matches the device access key.\n"
                "3. Ensure PC and ESP32 are on the same Wi-Fi subnet (disable VPN if active)."
            )
        finally:
            sock.close()

    @staticmethod
    def discover():
        devices = {}
        responses = ControlProtocol.send_and_receive(
            "DISCOVER",
            DISCOVERY_ADDRESS,
            expect_multiple=True,
            timeout=DISCOVERY_TIMEOUT,
            broadcast=True,
        )
        for response, source_ip in responses:
            try:
                device = ControlProtocol.parse_device_info(response, source_ip)
                devices[device.ip] = device
            except ValueError:
                continue
        return list(devices.values())

    @staticmethod
    def get_challenge(target_ip):
        response, _ = ControlProtocol.send_and_receive("CHALLENGE", target_ip)
        parts = response.strip().split("|")
        if len(parts) != 2 or parts[0] != "CHALLENGE":
            raise RuntimeError("Device did not return a valid challenge.")
        return parts[1]

    @staticmethod
    def authenticated_command(target_ip, secret, command, *payload):
        if not secret:
            raise RuntimeError("Device access key is required.")

        challenge = ControlProtocol.get_challenge(target_ip)
        auth = sign_message(secret, command, challenge, *payload)
        message = "|".join([command, challenge, *payload, auth])
        response, _ = ControlProtocol.send_and_receive(message, target_ip)

        parts = response.strip().split("|")
        if not parts:
            raise RuntimeError("Device returned an empty response.")
        if parts[0] == "ERR":
            detail = parts[1] if len(parts) > 1 else "UNKNOWN"
            raise RuntimeError(f"Device rejected command: {detail}")
        if parts[0] != "ACK":
            raise RuntimeError("Unexpected device acknowledgement.")
        return parts[1:]

    @staticmethod
    def start_stream(target_ip, secret, client_ip, client_port):
        return ControlProtocol.authenticated_command(
            target_ip,
            secret,
            "START",
            client_ip,
            str(client_port),
        )

    @staticmethod
    def stop_stream(target_ip, secret):
        return ControlProtocol.authenticated_command(target_ip, secret, "STOP")

    @staticmethod
    def ping(target_ip, secret):
        return ControlProtocol.authenticated_command(target_ip, secret, "PING")


class WiFiSerialProvisionProtocol:
    @staticmethod
    def available_ports():
        return list(serial.tools.list_ports.comports())

    @staticmethod
    def _exchange_line(port_name, command, expected_prefixes, timeout=SERIAL_RESPONSE_TIMEOUT_S):
        try:
            with serial.Serial(port_name, USB_SERIAL_BAUD, timeout=0.3, write_timeout=1) as ser:
                ser.setDTR(False)
                ser.setRTS(False)
                time.sleep(0.15)
                time.sleep(SERIAL_BOOT_WAIT_S)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write((command + "\n").encode("utf-8"))
                ser.flush()

                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    raw_line = ser.readline()
                    if not raw_line:
                        continue
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    if any(line.startswith(prefix) for prefix in expected_prefixes):
                        return line
        except serial.SerialException as exc:
            raise RuntimeError(f"Serial communication failed on {port_name}: {exc}") from exc

        raise RuntimeError("The ESP32 did not return a serial response in time.")

    @staticmethod
    def query_info(port_name):
        response = WiFiSerialProvisionProtocol._exchange_line(port_name, "INFO", expected_prefixes=("INFO|", "ERR|"))
        parts = response.split("|")
        if len(parts) >= 2 and parts[0] == "ERR":
            raise RuntimeError(f"ESP32 returned an error: {parts[1]}")
        if len(parts) != 6 or parts[0] != "INFO":
            raise RuntimeError(f"Unexpected serial response: {response}")

        return SerialDeviceInfo(
            port_name=port_name,
            device_id=parts[1],
            device_name=parts[2],
            imu_ready=parts[3] == "1",
            wifi_saved=parts[4] == "1",
            firmware=parts[5],
        )

    @staticmethod
    def provision(port_name, ssid, password):
        encoded_ssid = quote(ssid, safe="")
        encoded_password = quote(password, safe="")
        response = WiFiSerialProvisionProtocol._exchange_line(
            port_name,
            f"PROVISION|{encoded_ssid}|{encoded_password}",
            expected_prefixes=("ACK|", "ERR|"),
            timeout=8.0,
        )
        parts = response.split("|")
        if len(parts) >= 2 and parts[0] == "ACK" and parts[1] == "PROVISIONED":
            return
        if len(parts) >= 2 and parts[0] == "ERR":
            raise RuntimeError(f"ESP32 rejected provisioning: {parts[1]}")
        raise RuntimeError(f"Unexpected serial response: {response}")


class USBProvisionDialog(QDialog):
    """Dialog for provisioning Wi-Fi credentials to ESP32 over USB serial."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("USB Wi-Fi Provisioning")
        self.setModal(True)
        self.resize(500, 320)
        self.selected_port = ""

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        intro = QLabel("Connect ESP32-S3 over USB, select the serial port, probe, and send Wi-Fi credentials.")
        intro.setWordWrap(True)
        layout.addWidget(intro)

        row_port = QHBoxLayout()
        row_port.addWidget(QLabel("Serial Port:"))
        self.port_selector = QComboBox()
        row_port.addWidget(self.port_selector, stretch=1)
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ports)
        row_port.addWidget(btn_refresh)
        layout.addLayout(row_port)

        form = QFormLayout()
        self.ssid_input = QLineEdit()
        self.pass_input = QLineEdit()
        self.pass_input.setEchoMode(QLineEdit.Password)
        form.addRow("Wi-Fi SSID:", self.ssid_input)
        form.addRow("Wi-Fi Password:", self.pass_input)
        layout.addLayout(form)

        btn_row = QHBoxLayout()
        btn_probe = QPushButton("Probe Selected Port")
        btn_probe.clicked.connect(self.probe_selected_port)
        btn_row.addWidget(btn_probe)

        btn_send = QPushButton("Send Credentials")
        btn_send.setStyleSheet("background-color: #2e7d32; color: white;")
        btn_send.clicked.connect(self.send_credentials)
        btn_row.addWidget(btn_send)

        btn_cancel = QPushButton("Close")
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(btn_cancel)
        layout.addLayout(btn_row)

        self.lbl_status = QLabel("Ready.")
        self.lbl_status.setStyleSheet("color: #A9C2CF;")
        layout.addWidget(self.lbl_status)

        self.refresh_ports()

    def refresh_ports(self):
        self.port_selector.clear()
        ports = WiFiSerialProvisionProtocol.available_ports()
        for p in ports:
            self.port_selector.addItem(f"{p.device} - {p.description}", p.device)

    def probe_selected_port(self):
        port_name = self.port_selector.currentData() or self.port_selector.currentText().split()[0]
        if not port_name:
            QMessageBox.warning(self, "Warning", "Select a serial port first.")
            return
        try:
            info = WiFiSerialProvisionProtocol.query_info(port_name)
            self.lbl_status.setText(f"Detected {info.device_name} (ID: {info.device_id}, FW: {info.firmware})")
            self.lbl_status.setStyleSheet("color: #3B9797;")
        except Exception as e:
            QMessageBox.warning(self, "Probe Failed", str(e))
            self.lbl_status.setText("Probe failed.")

    def send_credentials(self):
        port_name = self.port_selector.currentData() or self.port_selector.currentText().split()[0]
        ssid = self.ssid_input.text().strip()
        password = self.pass_input.text()

        if not port_name or not ssid:
            QMessageBox.warning(self, "Warning", "Port and SSID are required.")
            return

        try:
            WiFiSerialProvisionProtocol.provision(port_name, ssid, password)
            QMessageBox.information(self, "Success", "Wi-Fi credentials sent to ESP32! It will now restart and join Wi-Fi.")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Provision Error", str(e))


# --- BACKGROUND WORKERS ---


class WirelessStreamWorker(QThread):
    """Receives live UDP EMG+IMU datagrams from ESP32 hardware."""
    batch_received = pyqtSignal(object)
    error_occurred = pyqtSignal(str)

    def __init__(self, port=WIFI_STREAM_PORT):
        super().__init__()
        self.port = int(port)
        self._running = True
        self._sock = None

    def _parse_datagram(self, data):
        # 1. Standard 252-byte binary packet with BWIM header
        if len(data) == WIRELESS_PACKET_SIZE:
            magic, version, frame_count, frame_size, packet_sequence = struct.unpack(
                WIFI_PACKET_HEADER_FORMAT, data[:WIFI_PACKET_HEADER_SIZE]
            )
            if magic != b"BWIM" or version != 1 or frame_size != WIRELESS_FRAME_SIZE:
                return None
            payload = data[WIFI_PACKET_HEADER_SIZE:]
            expected_payload = frame_count * frame_size
            if len(payload) != expected_payload:
                return None

            rows = []
            for offset in range(0, expected_payload, WIRELESS_FRAME_SIZE):
                frame = payload[offset : offset + WIRELESS_FRAME_SIZE]
                _frame_id, _frame_ts, _imu_id, _imu_ts, *frame_fields = struct.unpack(WIRELESS_FRAME_FORMAT, frame)
                # 8 EMG channels + roll, pitch, yaw
                row = [float(v) for v in frame_fields[:WIRELESS_EMG_CHANNELS]]
                row.extend([float(frame_fields[8]), float(frame_fields[9]), float(frame_fields[10])])
                rows.append(row)
            batch = np.asarray(rows, dtype=np.float32)
            return {"batch": batch, "source": "wireless"}

        # 2. Raw 240-byte multi-frame binary payload (without header)
        if len(data) == (WIRELESS_FRAME_SIZE * WIRELESS_FRAMES_PER_PACKET):
            rows = []
            for offset in range(0, len(data), WIRELESS_FRAME_SIZE):
                frame = data[offset : offset + WIRELESS_FRAME_SIZE]
                _frame_id, _frame_ts, _imu_id, _imu_ts, *frame_fields = struct.unpack(WIRELESS_FRAME_FORMAT, frame)
                row = [float(v) for v in frame_fields[:WIRELESS_EMG_CHANNELS]]
                row.extend([float(frame_fields[8]), float(frame_fields[9]), float(frame_fields[10])])
                rows.append(row)
            batch = np.asarray(rows, dtype=np.float32)
            return {"batch": batch, "source": "wireless_legacy"}

        # 3. Single 48-byte binary frame
        if len(data) == WIRELESS_FRAME_SIZE:
            _frame_id, _frame_ts, _imu_id, _imu_ts, *frame_fields = struct.unpack(WIRELESS_FRAME_FORMAT, data)
            row = [float(v) for v in frame_fields[:WIRELESS_EMG_CHANNELS]]
            row.extend([float(frame_fields[8]), float(frame_fields[9]), float(frame_fields[10])])
            batch = np.asarray([row], dtype=np.float32)
            return {"batch": batch, "source": "wireless_single"}

        # 4. Text CSV over UDP fallback
        try:
            line = data.decode("utf-8", errors="ignore").strip()
            if line:
                parts = line.replace(",", " ").split()
                if len(parts) >= 1:
                    vals = [float(p) for p in parts]
                    return {"batch": np.asarray([vals], dtype=np.float32), "source": "wireless_text"}
        except Exception:
            pass

        return None

    def run(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            self.error_occurred.emit(f"Failed to bind UDP stream port {self.port}: {e}")
            return
        self._sock.settimeout(1.0)

        try:
            while self._running:
                try:
                    data, _addr = self._sock.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    break

                payload = self._parse_datagram(data)
                if payload is not None:
                    batch = payload.get("batch")
                    if batch is not None and batch.size > 0:
                        self.batch_received.emit(payload)
        except Exception as exc:
            if self._running:
                self.error_occurred.emit(f"Wireless stream error: {exc}")
        finally:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self.wait()


class SerialWorker(QThread):
    """Reads live EMG data from serial port or TCP simulator."""
    batch_received = pyqtSignal(object)
    error_occurred = pyqtSignal(str)

    def __init__(self, port_name, baud_rate, num_channels, batch_size=25):
        super().__init__()
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.num_channels = num_channels
        self.batch_size = batch_size
        self._running = True
        self._serial = None

    def run(self):
        partial_line = ""
        batch = []
        try:
            self._serial = serial.serial_for_url(self.port_name, self.baud_rate, timeout=0.05)
            self._serial.reset_input_buffer()

            while self._running:
                waiting = self._serial.in_waiting
                chunk = self._serial.read(waiting if waiting else 1)
                if not chunk:
                    continue

                partial_line += chunk.decode("utf-8", errors="ignore")
                lines = partial_line.split("\n")
                partial_line = lines.pop()

                for raw_line in lines:
                    line = raw_line.strip()
                    if not line:
                        continue
                    
                    parts = line.replace(",", " ").split()
                    if len(parts) < self.num_channels:
                        continue
                    
                    try:
                        vals = [float(parts[i]) for i in range(self.num_channels)]
                        batch.append(vals)
                    except ValueError:
                        continue

                    if len(batch) >= self.batch_size:
                        self.batch_received.emit(np.asarray(batch, dtype=np.float32))
                        batch = []
        except Exception as e:
            if self._running:
                self.error_occurred.emit(str(e))
        finally:
            if self._serial and self._serial.is_open:
                self._serial.close()

    def stop(self):
        self._running = False
        self.wait()


class InferenceWorker(QThread):
    """Extracts features and runs Random Forest prediction."""
    prediction_ready = pyqtSignal(str, float)

    def __init__(self, sample_rate):
        super().__init__()
        self.sample_rate = sample_rate
        self.model = None
        self.class_names = []
        self._window = None
        self._running = True
        self._lock = threading.Lock()
        self._event = threading.Event()

    def load_model(self, model, class_names):
        with self._lock:
            self.model = model
            self.class_names = class_names

    def submit_window(self, window):
        with self._lock:
            self._window = window
        self._event.set()

    def run(self):
        while self._running:
            self._event.wait(0.1)
            if not self._running:
                break
            if not self._event.is_set():
                continue
            self._event.clear()

            with self._lock:
                win = self._window
                model = self.model
                classes = self.class_names
                self._window = None

            if win is None or model is None or not HAS_RF:
                continue

            try:
                feats = extract_window_features(win, sample_rate=self.sample_rate).reshape(1, -1)
                
                conf = 0.0
                if hasattr(model, "predict_proba"):
                    proba = model.predict_proba(feats)[0]
                    pred_idx = int(np.argmax(proba))
                    conf = float(proba[pred_idx])
                    model_classes = list(getattr(model, "classes_", []))
                    best_cls = model_classes[pred_idx]
                    
                    if isinstance(best_cls, (int, np.integer)) and 0 <= best_cls < len(classes):
                        pred_label = classes[best_cls]
                    else:
                        pred_label = str(best_cls)
                else:
                    pred_raw = model.predict(feats)[0]
                    if isinstance(pred_raw, (int, np.integer)) and 0 <= pred_raw < len(classes):
                        pred_label = classes[pred_raw]
                    else:
                        pred_label = str(pred_raw)
                        conf = 1.0

                self.prediction_ready.emit(pred_label, conf)

            except Exception as e:
                print(f"Inference error: {e}")

    def stop(self):
        self._running = False
        self._event.set()
        self.wait()
