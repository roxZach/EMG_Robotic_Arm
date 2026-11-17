import os
import sys


THEME_COLORS = {
    "special": "#EF476F",
    "bg": "#09111F",
    "title_bar": "#0B1526",
    "panel": "#111F35",
    "accent": "#38BDF8",
    "success": "#2DD4BF",
    "graph_bg": "#07101D",
    "text": "#F1F5F9",
    "muted": "#94A3B8",
    "disabled": "#64748B",
}


def apply_dark_title_bar(window):
    """Request a dark native title bar on Windows where supported."""
    try:
        import ctypes
        import sys
        from ctypes import wintypes
    except Exception:
        return False

    if sys.platform != "win32":
        return False

    try:
        hwnd = int(window.winId())
    except Exception:
        return False

    def _hex_to_colorref(hex_color):
        val = (hex_color or "").strip().lstrip("#")
        if len(val) != 6:
            return None
        try:
            r = int(val[0:2], 16)
            g = int(val[2:4], 16)
            b = int(val[4:6], 16)
        except ValueError:
            return None
        return (b << 16) | (g << 8) | r

    use_dark = ctypes.c_int(1)
    use_dark_size = ctypes.sizeof(use_dark)
    attrs = (20, 19)  # Win10 20H1+, then legacy fallback
    enabled_dark = False
    for attr in attrs:
        try:
            result = ctypes.windll.dwmapi.DwmSetWindowAttribute(
                wintypes.HWND(hwnd),
                wintypes.DWORD(attr),
                ctypes.byref(use_dark),
                wintypes.DWORD(use_dark_size),
            )
            if result == 0:
                enabled_dark = True
                break
        except Exception:
            continue

    caption_color = _hex_to_colorref(THEME_COLORS.get("title_bar", THEME_COLORS["bg"]))
    text_color = _hex_to_colorref(THEME_COLORS["text"])
    if caption_color is not None:
        try:
            caption_val = ctypes.c_int(caption_color)
            ctypes.windll.dwmapi.DwmSetWindowAttribute(
                wintypes.HWND(hwnd),
                wintypes.DWORD(35),  # DWMWA_CAPTION_COLOR
                ctypes.byref(caption_val),
                wintypes.DWORD(ctypes.sizeof(caption_val)),
            )
        except Exception:
            pass
    if text_color is not None:
        try:
            text_val = ctypes.c_int(text_color)
            ctypes.windll.dwmapi.DwmSetWindowAttribute(
                wintypes.HWND(hwnd),
                wintypes.DWORD(36),  # DWMWA_TEXT_COLOR
                ctypes.byref(text_val),
                wintypes.DWORD(ctypes.sizeof(text_val)),
            )
        except Exception:
            pass
    return enabled_dark


def app_stylesheet(font_size=16):
    c = THEME_COLORS
    if os.name == "nt":
        default_font = "Bahnschrift"
    elif sys.platform == "darwin":
        default_font = "Helvetica Neue"
    else:
        default_font = "Sans Serif"
    return f"""
QWidget {{
    background-color: {c['bg']};
    color: {c['text']};
    font-family: "{default_font}";
    font-size: {int(font_size)}px;
}}
QMainWindow, QDialog {{
    background-color: {c['bg']};
}}
QToolTip {{
    background-color: #10182B;
    color: #67E8F9;
    border: 1px solid #A855F7;
    border-radius: 5px;
    padding: 5px;
}}
QLabel {{
    color: {c['text']};
}}
QGroupBox {{
    border: 1px solid #29415F;
    border-radius: 10px;
    margin-top: 12px;
    padding: 9px;
    font-weight: 700;
    color: #67E8F9;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 7px;
    color: #C084FC;
}}
QPushButton {{
    background-color: #162A45;
    color: {c['text']};
    border: 1px solid #27466B;
    border-radius: 8px;
    padding: 7px 13px;
    font-weight: 600;
}}
QPushButton:hover:!disabled {{
    background-color: #1E3A5F;
    border-color: {c['accent']};
}}
QPushButton:pressed {{
    background-color: #6D28D9;
    border-color: #C084FC;
}}
QPushButton:disabled {{
    background-color: {c['panel']};
    color: {c['disabled']};
    border-color: {c['accent']};
}}
QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background-color: {c['panel']};
    color: {c['text']};
    border: 1px solid #29415F;
    border-radius: 8px;
    padding: 5px 8px;
    selection-background-color: #0E7490;
}}
QComboBox QAbstractItemView {{
    background-color: {c['panel']};
    color: {c['text']};
    selection-background-color: #0E7490;
    border: 1px solid #29415F;
}}
QCheckBox {{
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid #3B5B80;
    border-radius: 4px;
    background: #0B1526;
}}
QCheckBox::indicator:checked {{
    background: #A855F7;
    border-color: #E879F9;
}}
QRadioButton::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid #3B5B80;
    border-radius: 7px;
    background: #0B1526;
}}
QRadioButton::indicator:checked {{
    background: #22D3EE;
    border: 3px solid #0B1526;
    outline: 1px solid #67E8F9;
}}
QTabWidget::pane {{
    border: 1px solid #29415F;
    background: {c['panel']};
    border-radius: 8px;
}}
QTabBar::tab {{
    background: #0D1A2D;
    color: {c['muted']};
    border: 1px solid transparent;
    border-radius: 7px;
    margin-right: 4px;
    padding: 7px 13px;
}}
QTabBar::tab:selected {{
    background: #173A5E;
    color: {c['text']};
    border-color: #255D8F;
}}
QHeaderView::section {{
    background-color: #152642;
    color: #67E8F9;
    border: 1px solid #29415F;
    padding: 6px;
    font-weight: 700;
}}
QTableWidget {{
    background-color: {c['panel']};
    color: {c['text']};
    alternate-background-color: #0D1A2D;
    gridline-color: #29415F;
    border: 1px solid #29415F;
    selection-background-color: #163F63;
    selection-color: #F0FDFA;
}}
QProgressBar {{
    border: 1px solid {c['accent']};
    border-radius: 5px;
    text-align: center;
    background-color: {c['panel']};
    color: {c['text']};
}}
QProgressBar::chunk {{
    background-color: #22D3EE;
}}
QScrollArea {{
    border: 1px solid #29415F;
    background-color: {c['panel']};
}}
QScrollBar:vertical {{
    border: none;
    background: {c['bg']};
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: #7C3AED;
    min-height: 24px;
    border-radius: 5px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
    background: transparent;
    border: none;
}}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    background: {c['bg']};
}}
QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {{
    background: transparent;
    width: 0px;
    height: 0px;
}}
QScrollBar:horizontal {{
    border: none;
    background: {c['bg']};
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: #7C3AED;
    min-width: 24px;
    border-radius: 5px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
    background: transparent;
    border: none;
}}
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
    background: {c['bg']};
}}
QScrollBar::left-arrow:horizontal, QScrollBar::right-arrow:horizontal {{
    background: transparent;
    width: 0px;
    height: 0px;
}}
"""


def apply_dark_theme(app, font_size=16):
    font = app.font()
    if os.name == "nt":
        from PyQt5.QtGui import QFontInfo

        font.setFamily("Bahnschrift")
        # Fallback for Windows systems where Bahnschrift is unavailable.
        if QFontInfo(font).family().lower() != "bahnschrift":
            font.setFamily("Segoe UI")
    else:
        font.setFamily("Helvetica Neue" if sys.platform == "darwin" else "Sans Serif")
    app.setFont(font)
    app.setStyleSheet(app_stylesheet(font_size=font_size))


def themed_button_style(kind="accent"):
    c = THEME_COLORS
    if kind == "danger":
        bg = c["special"]
        text = c["text"]
        border = c["special"]
        hover_bg = "#BE123C"
        hover_border = "#FB7185"
    elif kind == "success":
        bg = c["success"]
        text = "#042F2E"
        border = c["success"]
        hover_bg = "#5EEAD4"
        hover_border = "#99F6E4"
    elif kind == "muted":
        bg = c["panel"]
        text = c["text"]
        border = c["accent"]
        hover_bg = c["bg"]
        hover_border = c["accent"]
    else:
        bg = c["accent"]
        text = "#082F49"
        border = c["accent"]
        hover_bg = "#7DD3FC"
        hover_border = "#BAE6FD"
    return (
        f"QPushButton {{ "
        f"background-color: {bg}; color: {text}; font-weight: bold; "
        f"border: 1px solid {border}; border-radius: 8px; padding: 6px 13px; "
        f"}} "
        f"QPushButton:hover:!disabled {{ "
        f"background-color: {hover_bg}; border-color: {hover_border}; "
        f"}} "
        f"QPushButton:pressed:!disabled {{ "
        f"background-color: {c['special']}; border-color: {c['special']}; color: {c['text']}; "
        f"}} "
        f"QPushButton:disabled {{ "
        f"background-color: {c['panel']}; color: {c['disabled']}; border-color: {c['panel']}; "
        f"}}"
    )


def themed_label_style(kind="muted"):
    c = THEME_COLORS
    if kind == "danger":
        color = c["special"]
    elif kind == "success":
        color = c["success"]
    else:
        color = c["muted"]
    return f"color: {color}; font-weight: bold;"


def themed_status_color(color_hint=None):
    hint = (color_hint or "").strip().lower()
    success_hints = {"#2e7d32", "#00c853", "#4caf50", "#78a083", "#3b9797"}
    warning_hints = {"#ff9800", "#f57c00"}
    error_hints = {"#f44336", "#c62828", "#bf092f"}
    if hint in error_hints:
        return THEME_COLORS["special"]
    if hint in warning_hints:
        return THEME_COLORS["special"]
    if hint in success_hints:
        return THEME_COLORS["success"]
    return THEME_COLORS["muted"]
