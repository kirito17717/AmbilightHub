import sys
import time
import json
import os
import numpy as np
import mss
import serial
import serial.tools.list_ports

from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal, QObject, QRect, QTimer
)
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QSlider, QComboBox, QCheckBox, QSpinBox, 
    QDoubleSpinBox, QGroupBox, QTabWidget, QSystemTrayIcon, QMenu, 
    QWidgetAction, QPushButton, QStyle, QMessageBox
)
from PyQt6.QtGui import QIcon, QPainter, QColor, QPen, QBrush, QAction, QKeyEvent
from PyQt6.QtNetwork import QLocalServer, QLocalSocket

CONFIG_FILE = "ambilight_config.json"
SOCKET_NAME = "AmbilightAdvancedHub_SingleInstance_Socket"

DEFAULT_CONFIG = {
    "port": "COM12",
    "baudrate": 921600,
    "target_fps": 60,
    "sync_with_screen": True,
    "master_brightness": 75,
    "gamma": 1.9,
    "black_threshold": 35,
    "r_correction": 1.0,
    "g_correction": 1.0,
    "b_correction": 1.0,
    "smoothing": 0.1,
    "enable_dithering": True,
    
    "connection_timeout": 30,
    "exit_on_timeout": True,
    "layout_mode": "Полный периметр (4 стороны)",
    
    'bottom_count': 30, 'top_count': 30, 'left_count': 20, 'right_count': 20,
    'bottom_height': 40, 'bottom_offset': 5, 'bottom_margin_side': 0,
    'top_height': 40, 'top_offset': 5, 'top_margin_side': 0,
    'left_width': 40, 'left_offset': 5, 'left_margin_side': 0,
    'right_width': 40, 'right_offset': 5, 'right_margin_side': 0,
    
    'start_corner': "Нижний правый", 'direction': 0,
    "test_mode": "Disabled"
}

SYNC_MAX_FPS = 60


def disable_captureblt():
    for name in ("mss.windows.gdi", "mss.windows"):
        mod = sys.modules.get(name)
        if mod is not None and hasattr(mod, "CAPTUREBLT"):
            mod.CAPTUREBLT = 0


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
                merged = DEFAULT_CONFIG.copy()
                merged.update(cfg)
                return merged
        except Exception:
            pass
    return DEFAULT_CONFIG.copy()

def save_config(config):
    try:
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=4, ensure_ascii=False)
    except Exception as e:
        print(f"Error saving config: {e}")


class AmbilightWorker(QThread):
    colors_updated = pyqtSignal(list, list)
    status_signal = pyqtSignal(str)
    timeout_occurred = pyqtSignal()

    def __init__(self, config):
        super().__init__()
        self.config = config.copy()
        self.running = True
        self.serial_conn = None
        self.dither_error = None
        self.configured_sent = False
        self.last_colors = []
        self.lights_enabled = True

    def update_config(self, new_config):
        old_total = self.get_total_leds(self.config)
        self.config.update(new_config)
        new_total = self.get_total_leds(self.config)
        if old_total != new_total:
            self.configured_sent = False

    def set_lights_enabled(self, enabled):
        self.lights_enabled = enabled
        if not enabled and self.serial_conn and self.serial_conn.is_open:
            try:
                num_leds = self.get_total_leds(self.config)
                if num_leds > 0:
                    hi = (num_leds >> 8) & 0xFF
                    lo = num_leds & 0xFF
                    payload = bytearray([0] * (num_leds * 3))
                    checksum = hi ^ lo ^ 0 ^ 0x55
                    packet = bytes([ord('A'), ord('d'), ord('a'), hi, lo, 0, checksum]) + payload
                    self.serial_conn.write(packet)
            except Exception:
                pass

    def get_total_leds(self, c):
        mode = c.get("layout_mode", "Полный периметр (4 стороны)")
        if mode == "Только нижняя линия (1 сторона)":
            return c.get('bottom_count', 0)
        elif mode == "П-образный (Низ + Лево + Право)":
            return c.get('bottom_count', 0) + c.get('left_count', 0) + c.get('right_count', 0)
        else:
            return c.get('bottom_count', 0) + c.get('top_count', 0) + c.get('left_count', 0) + c.get('right_count', 0)

    def run(self):
        timeout_sec = self.config.get("connection_timeout", 30)
        start_init_time = time.time()

        while self.running and not (self.serial_conn and self.serial_conn.is_open):
            self.init_serial()
            if self.serial_conn and self.serial_conn.is_open:
                break

            elapsed_init = time.time() - start_init_time
            if timeout_sec > 0 and elapsed_init > timeout_sec:
                self.status_signal.emit(f"Таймаут: нет связи с ESP ({timeout_sec} сек). Автоотключение.")
                self.timeout_occurred.emit()
                return
            time.sleep(1.0)

        with mss.mss() as sct:
            disable_captureblt()
            disconnected_since = None

            while self.running:
                loop_start = time.perf_counter()

                if not self.serial_conn or not self.serial_conn.is_open:
                    if disconnected_since is None:
                        disconnected_since = time.time()
                    if timeout_sec > 0 and (time.time() - disconnected_since) > timeout_sec:
                        self.status_signal.emit(
                            f"Таймаут: нет связи с ESP ({timeout_sec} сек). Автоотключение."
                        )
                        self.timeout_occurred.emit()
                        return
                    self.init_serial()
                    time.sleep(1.0)
                    continue
                else:
                    disconnected_since = None

                monitor = sct.monitors[1]
                zones_rects = self.generate_zones_rects(monitor["width"], monitor["height"])
                if len(zones_rects) == 0:
                    time.sleep(0.05)
                    continue

                if self.config.get("test_mode", "Disabled") == "Disabled":
                    raw = self.zone_means(sct, monitor, zones_rects)
                else:
                    raw = np.zeros((len(zones_rects), 3), dtype=np.float32)

                colors = self.process_colors(raw)
                self.last_colors = colors

                # Стабильная постоянная отправка каждого актуального кадра
                if self.lights_enabled:
                    self.send_adalight_data(self.reorder_leds(colors))

                if self.config.get("sync_with_screen", True):
                    fps = SYNC_MAX_FPS
                else:
                    fps = max(5, self.config.get("target_fps", 30))
                remaining = 1.0 / fps - (time.perf_counter() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    @staticmethod
    def group_strips(zones):
        groups = []
        for i, (x, y, w, h) in enumerate(zones):
            if groups:
                horiz, idxs = groups[-1]
                x0, y0, w0, h0 = zones[idxs[0]]
                if len(idxs) == 1:
                    if (y, h) == (y0, h0):
                        groups[-1] = (True, idxs + [i])
                        continue
                    if (x, w) == (x0, w0):
                        groups[-1] = (False, idxs + [i])
                        continue
                elif horiz and (y, h) == (y0, h0):
                    idxs.append(i)
                    continue
                elif not horiz and (x, w) == (x0, w0):
                    idxs.append(i)
                    continue
            groups.append((True, [i]))
        return groups

    def zone_means(self, sct, monitor, zones):
        out = np.zeros((len(zones), 3), dtype=np.float32)
        sw, sh = monitor["width"], monitor["height"]

        for horiz, idxs in self.group_strips(zones):
            rects = [zones[i] for i in idxs]
            left = max(0, min(r[0] for r in rects))
            top = max(0, min(r[1] for r in rects))
            right = min(sw, max(r[0] + r[2] for r in rects))
            bottom = min(sh, max(r[1] + r[3] for r in rects))
            if right <= left or bottom <= top:
                continue

            shot = sct.grab({
                "left": monitor["left"] + left, "top": monitor["top"] + top,
                "width": right - left, "height": bottom - top,
            })
            bgr = np.frombuffer(shot.raw, dtype=np.uint8).reshape(shot.height, shot.width, 4)[:, :, :3]

            if horiz:
                prof = bgr.sum(axis=0, dtype=np.uint32)
                depth = bgr.shape[0]
                starts = np.array([r[0] - left for r in rects])
            else:
                prof = bgr.sum(axis=1, dtype=np.uint32)
                depth = bgr.shape[1]
                starts = np.array([r[1] - top for r in rects])

            starts = np.clip(starts, 0, len(prof) - 1)
            order = np.argsort(starts, kind="stable")
            s = starts[order]
            sums = np.add.reduceat(prof, s, axis=0).astype(np.float32)
            lens = np.maximum(np.diff(np.append(s, len(prof))), 1)
            means = sums / (lens[:, None] * depth)
            out[np.array(idxs)[order]] = means[:, ::-1]

        return out

    def process_colors(self, raw_colors):
        test_mode = self.config.get("test_mode", "Disabled")
        if test_mode == "Red":
            return [(255, 0, 0)] * len(raw_colors)
        elif test_mode == "Green":
            return [(0, 255, 0)] * len(raw_colors)
        elif test_mode == "Blue":
            return [(0, 0, 255)] * len(raw_colors)

        black_thresh = self.config.get("black_threshold", 15)
        max_channels = np.max(raw_colors, axis=1)
        black_mask = max_channels < black_thresh

        gamma = max(0.1, self.config.get("gamma", 2.2))
        r_corr = self.config.get("r_correction", 1.0)
        g_corr = self.config.get("g_correction", 1.0)
        b_corr = self.config.get("b_correction", 1.0)

        norm_colors = raw_colors / 255.0
        corrected = np.power(norm_colors, gamma)
        corrected[:, 0] *= r_corr
        corrected[:, 1] *= g_corr
        corrected[:, 2] *= b_corr
        corrected = np.clip(corrected, 0.0, 1.0)

        corrected[black_mask] = 0.0

        smoothing = self.config.get("smoothing", 0.3)
        if smoothing > 0 and len(corrected) > 1:
            rolled_prev = np.roll(corrected, 1, axis=0)
            rolled_next = np.roll(corrected, -1, axis=0)
            corrected = (1 - smoothing) * corrected + (smoothing / 2.0) * (rolled_prev + rolled_next)

        if self.config.get("enable_dithering", True):
            if self.dither_error is None or len(self.dither_error) != len(corrected):
                self.dither_error = np.zeros_like(corrected)
            corrected += self.dither_error
            self.dither_error = corrected - np.clip(corrected, 0.0, 1.0)

        mb = self.config.get("master_brightness", 100) / 100.0
        final_colors = (corrected * mb * 255.0).astype(np.uint8)
        final_colors[black_mask] = 0

        return [tuple(c) for c in final_colors]

    def init_serial(self):
        port = self.config.get("port", "COM12")
        baudrate = self.config.get("baudrate", 921600)
        if not port:
            self.status_signal.emit("Порт не выбран")
            return

        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=0.1, write_timeout=0.5)
            self.configured_sent = False
            self.status_signal.emit(f"Подключено: {port}")
        except Exception as e:
            self.serial_conn = None
            self.status_signal.emit(f"Ожидание ESP ({port}): {e}")

    def generate_zones_rects(self, sw, sh):
        zones = []
        c = self.config
        mode = c.get("layout_mode", "Полный периметр (4 стороны)")

        if mode == "Только нижняя линия (1 сторона)":
            if c.get('bottom_count', 0) > 0:
                b_cnt = c['bottom_count']
                margin = c.get('bottom_margin_side', 0)
                bw = sw - (2 * margin)
                bh = c['bottom_height']
                by = sh - bh - c['bottom_offset']
                bx_start = margin
                step = bw / b_cnt if b_cnt > 0 else bw
                for i in range(b_cnt):
                    zones.append((int(bx_start + i * step), by, int(step), bh))
            return zones

        elif mode == "П-образный (Низ + Лево + Право)":
            l_cnt = c.get('left_count', 0)
            b_cnt = c.get('bottom_count', 0)
            r_cnt = c.get('right_count', 0)

            if l_cnt > 0:
                margin = c.get('left_margin_side', 0)
                lh = sh - (2 * margin)
                lw = c['left_width']
                lx = c['left_offset']
                ly_start = sh - margin - lh
                step = lh / l_cnt if l_cnt > 0 else lh
                for i in range(l_cnt):
                    zones.append((lx, int(ly_start + (l_cnt - 1 - i) * step), lw, int(step)))

            if b_cnt > 0:
                margin = c.get('bottom_margin_side', 0)
                bw = sw - (2 * margin)
                bh = c['bottom_height']
                by = sh - bh - c['bottom_offset']
                bx_start = margin
                step = bw / b_cnt if b_cnt > 0 else bw
                for i in range(b_cnt):
                    zones.append((int(bx_start + i * step), by, int(step), bh))

            if r_cnt > 0:
                margin = c.get('right_margin_side', 0)
                rh = sh - (2 * margin)
                rw = c['right_width']
                rx = sw - rw - c['right_offset']
                ry_start = margin
                step = rh / r_cnt if r_cnt > 0 else rh
                for i in range(r_cnt):
                    zones.append((rx, int(ry_start + i * step), rw, int(step)))

            return zones

        else:
            b_cnt = c.get('bottom_count', 0)
            r_cnt = c.get('right_count', 0)
            t_cnt = c.get('top_count', 0)
            l_cnt = c.get('left_count', 0)

            if b_cnt > 0:
                margin = c.get('bottom_margin_side', 0)
                bw = sw - (2 * margin)
                bh = c['bottom_height']
                by = sh - bh - c['bottom_offset']
                bx_start = margin
                step = bw / b_cnt if b_cnt > 0 else bw
                for i in range(b_cnt):
                    zones.append((int(bx_start + i * step), by, int(step), bh))

            if r_cnt > 0:
                margin = c.get('right_margin_side', 0)
                rh = sh - (2 * margin)
                rw = c['right_width']
                rx = sw - rw - c['right_offset']
                ry_start = margin
                step = rh / r_cnt if r_cnt > 0 else rh
                for i in range(r_cnt):
                    zones.append((rx, int(ry_start + i * step), rw, int(step)))

            if t_cnt > 0:
                margin = c.get('top_margin_side', 0)
                tw = sw - (2 * margin)
                th = c['top_height']
                ty = c['top_offset']
                tx_start = sw - margin - tw
                step = tw / t_cnt if t_cnt > 0 else tw
                for i in range(t_cnt):
                    zones.append((int(tx_start + (t_cnt - 1 - i) * step), ty, int(step), th))

            if l_cnt > 0:
                margin = c.get('left_margin_side', 0)
                lh = sh - (2 * margin)
                lw = c['left_width']
                lx = c['left_offset']
                ly_start = sh - margin - lh
                step = lh / l_cnt if l_cnt > 0 else lh
                for i in range(l_cnt):
                    zones.append((lx, int(ly_start + (l_cnt - 1 - i) * step), lw, int(step)))

            return zones

    def reorder_leds(self, colors):
        c = self.config
        mode = c.get("layout_mode", "Полный периметр (4 стороны)")
        
        if mode == "Только нижняя линия (1 сторона)" or mode == "П-образный (Низ + Лево + Право)":
            return colors
        else:
            b_cnt = c.get('bottom_count', 0)
            r_cnt = c.get('right_count', 0)
            t_cnt = c.get('top_count', 0)
            l_cnt = c.get('left_count', 0)

            bottom = colors[:b_cnt]
            right = colors[b_cnt:b_cnt+r_cnt]
            top = colors[b_cnt+r_cnt:b_cnt+r_cnt+t_cnt]
            left = colors[b_cnt+r_cnt+t_cnt:]

            full_chain = bottom + right + top + left
            corner = self.config.get('start_corner', "Нижний правый")
            direction = self.config.get('direction', 0)

            if len(full_chain) > 1:
                shift = 0
                if corner == "Нижний правый":
                    shift = b_cnt
                elif corner == "Верхний правый":
                    shift = b_cnt + r_cnt
                elif corner == "Верхний левый":
                    shift = b_cnt + r_cnt + t_cnt
                
                if shift > 0 and shift < len(full_chain):
                    full_chain = full_chain[shift:] + full_chain[:shift]

                if direction == 1:
                    first = full_chain[0]
                    rest = full_chain[1:][::-1]
                    full_chain = [first] + rest

            return full_chain

    def send_adalight_data(self, colors):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        num_leds = len(colors)
        if num_leds == 0:
            return

        hi = (num_leds >> 8) & 0xFF
        lo = num_leds & 0xFF
        brightness = 255

        if not self.configured_sent:
            cfg_checksum = hi ^ lo ^ brightness ^ 0x33
            cfg_packet = bytes([ord('C'), ord('f'), ord('g'), hi, lo, brightness, cfg_checksum])
            try:
                self.serial_conn.write(cfg_packet)
                self.serial_conn.flush()
                time.sleep(0.05)
                self.configured_sent = True
            except Exception as e:
                self.status_signal.emit(f"Ошибка конфига UART: {e}")
                self.serial_conn = None
                return

        ada_checksum = hi ^ lo ^ brightness ^ 0x55
        payload = bytearray()
        for r, g, b in colors:
            payload.extend([r, g, b])

        packet = bytes([ord('A'), ord('d'), ord('a'), hi, lo, brightness, ada_checksum]) + payload
        try:
            self.serial_conn.write(packet)
        except Exception as e:
            self.status_signal.emit(f"Ошибка UART потока: {e}")
            self.serial_conn = None

    def stop(self):
        self.running = False
        self.wait()


class ConfigOverlay(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool |
            Qt.WindowType.WindowTransparentForInput
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        screen = QApplication.primaryScreen().geometry()
        self.setGeometry(screen)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        sw = self.width()
        sh = self.height()

        zones_rects = self.main_window.worker.generate_zones_rects(sw, sh)
        curr_colors = getattr(self.main_window.worker, 'last_colors', [])

        for i, (x, y, w, h) in enumerate(zones_rects):
            if i < len(curr_colors):
                r, g, b = curr_colors[i]
                fill_color = QColor(r, g, b, 120)
                pen_color = QColor(r, g, b, 255)
            else:
                fill_color = QColor(30, 30, 30, 100)
                pen_color = QColor(200, 200, 200, 255)

            painter.setBrush(QBrush(fill_color))
            painter.setPen(QPen(pen_color, 2))
            painter.drawRect(x, y, w, h)


class MainWindow(QMainWindow):
    def __init__(self, config, worker):
        super().__init__()
        self.config = config
        self.worker = worker

        self.setWindowTitle("Ambilight Advanced Hub")
        self.resize(650, 550)
        




        self.overlay = ConfigOverlay(self)
        self.overlay_timer = QTimer(self)
        self.overlay_timer.timeout.connect(self.overlay.update)

        self.init_ui()
        self.worker.colors_updated.connect(self.on_colors_updated)
        self.worker.status_signal.connect(self.update_status)
        self.worker.finished.connect(self.on_worker_finished)
        self.worker.timeout_occurred.connect(self.on_timeout_triggered)

        self.is_running = True

    def changeEvent(self, event):
        if event.type() == event.Type.WindowStateChange:
            if self.isMinimized():
                self.hide()
                event.accept()
                return
        super().changeEvent(event)

    def on_colors_updated(self, rects, colors):
        if self.overlay.isVisible():
            self.overlay.update()

    def on_worker_finished(self):
        if not self.is_running:
            return
        self.is_running = False
        self.btn_start.setText("СТАРТ ПОДСВЕТКИ")
        self.btn_start.setStyleSheet(
            "background-color: #2ecc71; color: white; font-weight: bold; "
            "font-size: 14px; padding: 10px;"
        )

    def on_timeout_triggered(self):
        if self.config.get("exit_on_timeout", True):
            QApplication.quit()

    def get_config_dict(self):
        self.on_config_changed()
        return self.config

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_layout = QHBoxLayout()
        top_layout.addWidget(QLabel("COM-порт:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        top_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("Обновить")
        refresh_btn.clicked.connect(self.refresh_ports)
        top_layout.addWidget(refresh_btn)
        main_layout.addLayout(top_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_general_tab(), "Общие параметры")
        self.tabs.addTab(self.create_color_tab(), "Цвет и Гамма")
        
        self.tab_bottom = self.create_side_tab("bottom")
        self.tab_top = self.create_side_tab("top")
        self.tab_left = self.create_side_tab("left")
        self.tab_right = self.create_side_tab("right")

        self.tabs.addTab(self.tab_bottom, "Нижняя зона")
        self.tabs.addTab(self.tab_top, "Верхняя зона")
        self.tabs.addTab(self.tab_left, "Левая зона")
        self.tabs.addTab(self.tab_right, "Правая зона")

        main_layout.addWidget(self.tabs)

        btn_layout = QHBoxLayout()
        self.btn_preview = QPushButton("Показать сетку на экране")
        self.btn_preview.clicked.connect(self.toggle_overlay_manual)
        btn_layout.addWidget(self.btn_preview)
        main_layout.addLayout(btn_layout)

        self.btn_start = QPushButton("СТОП ПОДСВЕТКИ")
        self.btn_start.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px; padding: 10px;")
        self.btn_start.clicked.connect(self.toggle_ambilight)
        main_layout.addWidget(self.btn_start)

        self.status_label = QLabel("Статус: Запуск...")
        main_layout.addWidget(self.status_label)
        
        self.update_tabs_visibility()

    def update_tabs_visibility(self):
        mode = self.layout_combo.currentText()
        if mode == "Только нижняя линия (1 сторона)":
            self.tabs.setTabEnabled(3, False)
            self.tabs.setTabEnabled(4, False)
            self.tabs.setTabEnabled(5, False)
        elif mode == "П-образный (Низ + Лево + Право)":
            self.tabs.setTabEnabled(3, False)
            self.tabs.setTabEnabled(4, True)
            self.tabs.setTabEnabled(5, True)
        else:
            self.tabs.setTabEnabled(3, True)
            self.tabs.setTabEnabled(4, True)
            self.tabs.setTabEnabled(5, True)

    def create_general_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        c = self.config

        row = 0
        layout.addWidget(QLabel("<b>Режим использования периметра:</b>"), row, 0, 1, 3)
        row += 1

        self.layout_combo = QComboBox()
        self.layout_combo.addItems([
            "Полный периметр (4 стороны)", 
            "Только нижняя линия (1 сторона)", 
            "П-образный (Низ + Лево + Право)"
        ])
        self.layout_combo.setCurrentText(c.get("layout_mode", "Полный периметр (4 стороны)"))
        self.layout_combo.currentTextChanged.connect(self.update_tabs_visibility)
        layout.addWidget(self.layout_combo, row, 0, 1, 3)
        row += 1

        self.spin_b_cnt = QSpinBox()
        self.spin_b_cnt.setRange(0, 300)
        self.spin_b_cnt.setValue(c['bottom_count'])

        self.spin_t_cnt = QSpinBox()
        self.spin_t_cnt.setRange(0, 300)
        self.spin_t_cnt.setValue(c['top_count'])

        self.spin_l_cnt = QSpinBox()
        self.spin_l_cnt.setRange(0, 300)
        self.spin_l_cnt.setValue(c['left_count'])

        self.spin_r_cnt = QSpinBox()
        self.spin_r_cnt.setRange(0, 300)
        self.spin_r_cnt.setValue(c['right_count'])

        layout.addWidget(QLabel("Диодов (Низ):"), row, 0)
        layout.addWidget(self.spin_b_cnt, row, 2)
        row += 1

        layout.addWidget(QLabel("Диодов (Верх):"), row, 0)
        layout.addWidget(self.spin_t_cnt, row, 2)
        row += 1

        layout.addWidget(QLabel("Диодов (Лево):"), row, 0)
        layout.addWidget(self.spin_l_cnt, row, 2)
        row += 1

        layout.addWidget(QLabel("Диодов (Право):"), row, 0)
        layout.addWidget(self.spin_r_cnt, row, 2)
        row += 1

        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 2000000)
        self.baud_spin.setValue(c.get("baudrate", 921600))

        self.fps_spin = QSpinBox()
        self.fps_spin.setRange(10, 240)
        self.fps_spin.setValue(c.get("target_fps", 30))
        
        self.sync_screen_cb = QCheckBox("Синхронизировать с частотой экрана (динамический FPS)")
        self.sync_screen_cb.setChecked(c.get("sync_with_screen", True))
        self.sync_screen_cb.toggled.connect(self.on_sync_toggled)

        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(5, 300)
        self.timeout_spin.setValue(c.get("connection_timeout", 50))

        self.exit_on_timeout_cb = QCheckBox("Закрывать приложение при таймауте связи")
        self.exit_on_timeout_cb.setChecked(c.get("exit_on_timeout", True))

        layout.addWidget(QLabel("Baud Rate:"), row, 0, 1, 2)
        layout.addWidget(self.baud_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Лимит FPS (если выкл. синхр.):"), row, 0, 1, 2)
        layout.addWidget(self.fps_spin, row, 2)
        row += 1

        layout.addWidget(self.sync_screen_cb, row, 0, 1, 3)
        row += 1

        layout.addWidget(QLabel("Таймаут подключения (сек):"), row, 0, 1, 2)
        layout.addWidget(self.timeout_spin, row, 2)
        row += 1

        layout.addWidget(self.exit_on_timeout_cb, row, 0, 1, 3)
        row += 1

        self.combo_corner = QComboBox()
        self.combo_corner.addItems(["Нижний левый", "Нижний правый", "Верхний правый", "Верхний левый"])
        self.combo_corner.setCurrentText(c.get('start_corner', "Нижний правый"))

        self.combo_dir = QComboBox()
        self.combo_dir.addItems(["По часовой", "Против часовой"])
        self.combo_dir.setCurrentIndex(c.get('direction', 0))

        layout.addWidget(QLabel("Угол подключения ленты:"), row, 0, 1, 2)
        layout.addWidget(self.combo_corner, row, 2)
        row += 1

        layout.addWidget(QLabel("Направление обхода:"), row, 0, 1, 2)
        layout.addWidget(self.combo_dir, row, 2)
        row += 1

        test_layout = QHBoxLayout()
        test_layout.addWidget(QLabel("Тестовый режим:"))
        self.test_combo = QComboBox()
        self.test_combo.addItems(["Disabled", "Red", "Green", "Blue"])
        self.test_combo.setCurrentText(c.get("test_mode", "Disabled"))
        test_layout.addWidget(self.test_combo)
        layout.addLayout(test_layout, row, 0, 1, 3)

        for widget_item in [
            self.layout_combo, self.spin_b_cnt, self.spin_t_cnt, self.spin_l_cnt, self.spin_r_cnt,
            self.baud_spin, self.fps_spin, self.sync_screen_cb, self.timeout_spin, self.exit_on_timeout_cb,
            self.combo_corner, self.combo_dir, self.test_combo
        ]:
            if isinstance(widget_item, QComboBox):
                widget_item.currentTextChanged.connect(self.on_config_changed)
            elif isinstance(widget_item, QSpinBox):
                widget_item.valueChanged.connect(self.on_config_changed)
            elif isinstance(widget_item, QCheckBox):
                widget_item.toggled.connect(self.on_config_changed)

        self.fps_spin.setEnabled(not self.sync_screen_cb.isChecked())

        return widget

    def on_sync_toggled(self, checked):
        self.fps_spin.setEnabled(not checked)
        self.on_config_changed()

    def create_color_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        c = self.config

        self.mb_slider = QSlider(Qt.Orientation.Horizontal)
        self.mb_slider.setRange(0, 100)
        self.mb_slider.setValue(c.get("master_brightness", 100))
        self.mb_label = QLabel(f"{self.mb_slider.value()}%")
        self.mb_slider.valueChanged.connect(self.on_mb_changed)

        self.gamma_spin = QDoubleSpinBox()
        self.gamma_spin.setRange(0.5, 4.0)
        self.gamma_spin.setSingleStep(0.1)
        self.gamma_spin.setValue(c.get("gamma", 2.2))

        self.black_spin = QSpinBox()
        self.black_spin.setRange(0, 100)
        self.black_spin.setValue(c.get("black_threshold", 15))

        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(0.0, 2.0)
        self.r_spin.setSingleStep(0.05)
        self.r_spin.setValue(c.get("r_correction", 1.0))

        self.g_spin = QDoubleSpinBox()
        self.g_spin.setRange(0.0, 2.0)
        self.g_spin.setSingleStep(0.05)
        self.g_spin.setValue(c.get("g_correction", 1.0))

        self.b_spin = QDoubleSpinBox()
        self.b_spin.setRange(0.0, 2.0)
        self.b_spin.setSingleStep(0.05)
        self.b_spin.setValue(c.get("b_correction", 1.0))

        self.smooth_spin = QDoubleSpinBox()
        self.smooth_spin.setRange(0.0, 1.0)
        self.smooth_spin.setSingleStep(0.05)
        self.smooth_spin.setValue(c.get("smoothing", 0.3))

        self.dither_cb = QCheckBox("Включить временной дизеринг")
        self.dither_cb.setChecked(c.get("enable_dithering", True))

        row = 0
        layout.addWidget(QLabel("Мастер Яркость:"), row, 0)
        layout.addWidget(self.mb_slider, row, 1)
        layout.addWidget(self.mb_label, row, 2)
        row += 1

        layout.addWidget(QLabel("Гамма:"), row, 0, 1, 2)
        layout.addWidget(self.gamma_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Отсечка чёрного (0..100):"), row, 0, 1, 2)
        layout.addWidget(self.black_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Красный (R):"), row, 0, 1, 2)
        layout.addWidget(self.r_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Зеленый (G):"), row, 0, 1, 2)
        layout.addWidget(self.g_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Синий (B):"), row, 0, 1, 2)
        layout.addWidget(self.b_spin, row, 2)
        row += 1

        layout.addWidget(QLabel("Сглаживание по кольцу:"), row, 0, 1, 2)
        layout.addWidget(self.smooth_spin, row, 2)
        row += 1

        layout.addWidget(self.dither_cb, row, 0, 1, 3)

        for w_item in [self.gamma_spin, self.black_spin, self.r_spin, self.g_spin, self.b_spin, self.smooth_spin]:
            w_item.valueChanged.connect(self.on_config_changed)
        self.dither_cb.toggled.connect(self.on_config_changed)

        return widget

    def create_side_tab(self, side):
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        c = self.config

        if side == "bottom":
            self.spin_b_h = QSpinBox()
            self.spin_b_h.setRange(5, 300)
            self.spin_b_h.setValue(c['bottom_height'])
            self.spin_b_off = QSpinBox()
            self.spin_b_off.setRange(0, 300)
            self.spin_b_off.setValue(c['bottom_offset'])
            self.spin_b_ms = QSpinBox()
            self.spin_b_ms.setRange(0, 1000)
            self.spin_b_ms.setValue(c['bottom_margin_side'])

            layout.addWidget(QLabel("Высота зоны захвата (px):"), 0, 0)
            layout.addWidget(self.spin_b_h, 0, 1)
            layout.addWidget(QLabel("Отступ от низа экрана (px):"), 1, 0)
            layout.addWidget(self.spin_b_off, 1, 1)
            layout.addWidget(QLabel("Отступ по бокам (px):"), 2, 0)
            layout.addWidget(self.spin_b_ms, 2, 1)

            for w_item in [self.spin_b_h, self.spin_b_off, self.spin_b_ms]:
                w_item.valueChanged.connect(self.on_config_changed)

        elif side == "top":
            self.spin_t_h = QSpinBox()
            self.spin_t_h.setRange(5, 300)
            self.spin_t_h.setValue(c['top_height'])
            self.spin_t_off = QSpinBox()
            self.spin_t_off.setRange(0, 300)
            self.spin_t_off.setValue(c['top_offset'])
            self.spin_t_ms = QSpinBox()
            self.spin_t_ms.setRange(0, 1000)
            self.spin_t_ms.setValue(c['top_margin_side'])

            layout.addWidget(QLabel("Высота зоны захвата (px):"), 0, 0)
            layout.addWidget(self.spin_t_h, 0, 1)
            layout.addWidget(QLabel("Отступ от верха экрана (px):"), 1, 0)
            layout.addWidget(self.spin_t_off, 1, 1)
            layout.addWidget(QLabel("Отступ по бокам (px):"), 2, 0)
            layout.addWidget(self.spin_t_ms, 2, 1)

            for w_item in [self.spin_t_h, self.spin_t_off, self.spin_t_ms]:
                w_item.valueChanged.connect(self.on_config_changed)

        elif side == "left":
            self.spin_l_w = QSpinBox()
            self.spin_l_w.setRange(5, 300)
            self.spin_l_w.setValue(c['left_width'])
            self.spin_l_off = QSpinBox()
            self.spin_l_off.setRange(0, 300)
            self.spin_l_off.setValue(c['left_offset'])
            self.spin_l_ms = QSpinBox()
            self.spin_l_ms.setRange(0, 1000)
            self.spin_l_ms.setValue(c['left_margin_side'])

            layout.addWidget(QLabel("Ширина зоны захвата (px):"), 0, 0)
            layout.addWidget(self.spin_l_w, 0, 1)
            layout.addWidget(QLabel("Отступ слева от края экрана (px):"), 1, 0)
            layout.addWidget(self.spin_l_off, 1, 1)
            layout.addWidget(QLabel("Отступ по бокам (px):"), 2, 0)
            layout.addWidget(self.spin_l_ms, 2, 1)

            for w_item in [self.spin_l_w, self.spin_l_off, self.spin_l_ms]:
                w_item.valueChanged.connect(self.on_config_changed)

        elif side == "right":
            self.spin_r_w = QSpinBox()
            self.spin_r_w.setRange(5, 300)
            self.spin_r_w.setValue(c['right_width'])
            self.spin_r_off = QSpinBox()
            self.spin_r_off.setRange(0, 300)
            self.spin_r_off.setValue(c['right_offset'])
            self.spin_r_ms = QSpinBox()
            self.spin_r_ms.setRange(0, 1000)
            self.spin_r_ms.setValue(c['right_margin_side'])

            layout.addWidget(QLabel("Ширина зоны захвата (px):"), 0, 0)
            layout.addWidget(self.spin_r_w, 0, 1)
            layout.addWidget(QLabel("Отступ справа от края экрана (px):"), 1, 0)
            layout.addWidget(self.spin_r_off, 1, 1)
            layout.addWidget(QLabel("Отступ по бокам (px):"), 2, 0)
            layout.addWidget(self.spin_r_ms, 2, 1)

            for w_item in [self.spin_r_w, self.spin_r_off, self.spin_r_ms]:
                w_item.valueChanged.connect(self.on_config_changed)

        return widget

    def refresh_ports(self):
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports and "COM12" not in ports:
            ports = ["COM12"]
        elif "COM12" not in ports:
            ports.append("COM12")
            
        self.port_combo.addItems(ports)
        saved_port = self.config.get("port", "COM12")
        if saved_port in ports:
            self.port_combo.setCurrentText(saved_port)
        else:
            self.port_combo.setCurrentText("COM12")
        self.port_combo.blockSignals(False)

    def on_mb_changed(self, val):
        self.mb_label.setText(f"{val}%")
        self.on_config_changed()

    def update_mb_from_tray(self, val):
        self.mb_slider.blockSignals(True)
        self.mb_slider.setValue(val)
        self.mb_label.setText(f"{val}%")
        self.mb_slider.blockSignals(False)
        self.on_config_changed()

    def on_config_changed(self):
        self.config["port"] = self.port_combo.currentText()
        self.config["baudrate"] = self.baud_spin.value()
        self.config["target_fps"] = self.fps_spin.value()
        self.config["sync_with_screen"] = self.sync_screen_cb.isChecked()
        self.config["connection_timeout"] = self.timeout_spin.value()
        self.config["exit_on_timeout"] = self.exit_on_timeout_cb.isChecked()
        self.config["master_brightness"] = self.mb_slider.value()
        self.config["gamma"] = self.gamma_spin.value()
        self.config["black_threshold"] = self.black_spin.value()
        self.config["r_correction"] = self.r_spin.value()
        self.config["g_correction"] = self.g_spin.value()
        self.config["b_correction"] = self.b_spin.value()
        self.config["smoothing"] = self.smooth_spin.value()
        self.config["enable_dithering"] = self.dither_cb.isChecked()

        self.config["layout_mode"] = self.layout_combo.currentText()
        self.config["bottom_count"] = self.spin_b_cnt.value()
        self.config["top_count"] = self.spin_t_cnt.value()
        self.config["left_count"] = self.spin_l_cnt.value()
        self.config["right_count"] = self.spin_r_cnt.value()

        self.config['bottom_height'] = self.spin_b_h.value()
        self.config['bottom_offset'] = self.spin_b_off.value()
        self.config['bottom_margin_side'] = self.spin_b_ms.value()

        self.config['top_height'] = self.spin_t_h.value()
        self.config['top_offset'] = self.spin_t_off.value()
        self.config['top_margin_side'] = self.spin_t_ms.value()

        self.config['left_width'] = self.spin_l_w.value()
        self.config['left_offset'] = self.spin_l_off.value()
        self.config['left_margin_side'] = self.spin_l_ms.value()

        self.config['right_width'] = self.spin_r_w.value()
        self.config['right_offset'] = self.spin_r_off.value()
        self.config['right_margin_side'] = self.spin_r_ms.value()

        self.config['start_corner'] = self.combo_corner.currentText()
        self.config['direction'] = self.combo_dir.currentIndex()
        self.config["test_mode"] = self.test_combo.currentText()

        save_config(self.config)
        self.worker.update_config(self.config)

    def toggle_overlay_manual(self):
        if self.overlay.isVisible():
            self.overlay.hide()
            self.overlay_timer.stop()
            self.btn_preview.setText("Показать сетку на экране")
        else:
            self.overlay.show()
            self.overlay_timer.start(66)
            self.btn_preview.setText("Скрыть сетку с экрана")

    def toggle_ambilight(self):
        if self.is_running:
            self.worker.stop()
            self.is_running = False
            self.btn_start.setText("СТАРТ ПОДСВЕТКИ")
            self.btn_start.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; font-size: 14px; padding: 10px;")
            self.status_label.setText("Статус: Остановлено")
        else:
            self.worker = AmbilightWorker(self.config)
            self.worker.colors_updated.connect(self.on_colors_updated)
            self.worker.status_signal.connect(self.update_status)
            self.worker.finished.connect(self.on_worker_finished)
            self.worker.timeout_occurred.connect(self.on_timeout_triggered)
            self.worker.start()
            self.is_running = True
            self.btn_start.setText("СТОП ПОДСВЕТКИ")
            self.btn_start.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px; padding: 10px;")

    def update_status(self, text):
        self.status_label.setText(f"Статус: {text}")

    def closeEvent(self, event):
        self.worker.stop()
        self.overlay.close()
        event.accept()
        QApplication.quit()


class TrayApp:
    def __init__(self, app, config, worker):
        self.app = app
        self.config = config
        self.worker = worker
        self.window = MainWindow(config, worker)
        self.setup_tray()

    def setup_tray(self):
        self.tray_icon = QSystemTrayIcon()
        icon = self.app.style().standardIcon(QStyle.StandardPixmap.SP_ComputerIcon)
        self.tray_icon.setIcon(icon)
        self.tray_icon.setToolTip("Ambilight Advanced Hub")

        tray_menu = QMenu()

        self.toggle_lights_action = QAction("Выключить подсветку", tray_menu)
        def toggle_lights_state():
            current_state = self.worker.lights_enabled
            new_state = not current_state
            self.worker.set_lights_enabled(new_state)
            if new_state:
                self.toggle_lights_action.setText("Выключить подсветку")
            else:
                self.toggle_lights_action.setText("Включить подсветку")

        self.toggle_lights_action.triggered.connect(toggle_lights_state)
        tray_menu.addAction(self.toggle_lights_action)

        tray_menu.addSeparator()

        brightness_menu = tray_menu.addMenu("Яркость подсветки")
        slider_widget = QWidget()
        s_layout = QHBoxLayout(slider_widget)
        s_layout.setContentsMargins(10, 5, 10, 5)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(self.config.get("master_brightness", 100))
        lbl = QLabel(f"{slider.value()}%")

        def on_slider(v):
            lbl.setText(f"{v}%")
            self.window.update_mb_from_tray(v)

        slider.valueChanged.connect(on_slider)
        s_layout.addWidget(slider)
        s_layout.addWidget(lbl)

        wa = QWidgetAction(brightness_menu)
        wa.setDefaultWidget(slider_widget)
        brightness_menu.addAction(wa)

        for pct in [0, 25, 50, 75, 100]:
            act = QAction(f"{pct}%", brightness_menu)
            act.triggered.connect(lambda ch, p=pct: slider.setValue(p)) # Исправлено
            brightness_menu.addAction(act)

        tray_menu.addSeparator()

        show_action = QAction("Открыть настройки", tray_menu)
        show_action.triggered.connect(self.show_window)
        tray_menu.addAction(show_action)

        exit_action = QAction("Выход", tray_menu)
        exit_action.triggered.connect(self.quit_app)
        tray_menu.addAction(exit_action)

        self.tray_icon.setContextMenu(tray_menu)
        self.tray_icon.activated.connect(self.on_tray_icon_activated)
        self.tray_icon.show()

    def on_tray_icon_activated(self, reason):
        if reason == QSystemTrayIcon.ActivationReason.Trigger:
            self.show_window()

    def show_window(self):
        self.window.show()
        self.window.activateWindow()
        self.window.raise_()

    def quit_app(self):
        self.worker.stop()
        self.window.overlay.close()
        self.app.quit()


def main():
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)

    socket = QLocalSocket()
    socket.connectToServer(SOCKET_NAME)
    if socket.waitForConnected(500):
        socket.write(b"SHOW")
        socket.waitForBytesWritten(1000)
        socket.disconnectFromServer()
        sys.exit(0)

    server = QLocalServer()
    server.listen(SOCKET_NAME)

    config = load_config()
    worker = AmbilightWorker(config)
    tray_app = TrayApp(app, config, worker)

    def handle_ipc():
        client = server.nextPendingConnection()
        if client:
            client.waitForReadyRead(1000)
            msg = client.readAll().data().decode('utf-8')
            if msg == "SHOW":
                tray_app.show_window()
            client.disconnectFromServer()

    server.newConnection.connect(handle_ipc)
    worker.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
