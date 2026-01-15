import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QStackedWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QMessageBox, QFrame
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
import snap7
from snap7.util import get_bool, get_int, get_real, set_bool
import time
from miHikCamera import HikCamera
from clase_yolov5 import YoloModelV5
from CommPLC import *
import numpy as np
import cv2
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QSizePolicy, QSpacerItem, QTextEdit
import subprocess
import platform
import threading, math
from ctypes import byref
import select_pixel_v4
import json
import os
try:
    from snap7.types import Areas
except:
    from snap7.snap7types import Areas

CONFIG_PATH = "config_camara.json"
mtx = np.loadtxt('C:\\Users\\Seiki\\PycharmProjects\\HexagonLocator\\recursos\\mtx.txt')
distMtx = np.loadtxt('C:\\Users\\Seiki\\PycharmProjects\\HexagonLocator\\recursos\\dist.txt')
camMxt = np.loadtxt('C:\\Users\\Seiki\\PycharmProjects\\HexagonLocator\\recursos\\cameraMAtrix.txt')
guardar = True
ancho = 850
alto = 300


# CONSTANTES
AIMEN_CHECK_ALTURA_MAX_TRAMO0 = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."CHECK_ALTURA_MAX"."TRAMO_0"'
AIMEN_CHECK_ALTURA_MAX_TRAMO1 = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."CHECK_ALTURA_MAX"."TRAMO_1"'
AIMEN_CHECK_ALTURA_MAX_TRAMO2 = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."CHECK_ALTURA_MAX"."TRAMO_2"'
AIMEN_CHECK_ALTURA_MAX_TRAMO3 = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."CHECK_ALTURA_MAX"."TRAMO_3"'
AIMEN_PANEL_ANCHO = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."DIMENSIONES_PANEL"."ANCHO"'
AIMEN_PANEL_LARGO = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."DIMENSIONES_PANEL"."LARGO"'
AIMEN_FLANCO_BOTON = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."FLANCO_BOTON"'
AIMEN_PANEL_SALIDA = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."PANEL_SALIDA"'
AIMEN_PULSADOR_SEIKI = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."PULSACION_SEIKI"'
AIMEN_SECTOR0_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[0]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR0_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[0]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR1_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[1]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR1_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[1]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR2_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[2]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR2_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[2]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR3_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[3]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR3_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[3]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR4_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[4]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR4_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[4]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR5_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[5]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR5_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[5]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR6_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[6]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR6_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[6]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR7_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[7]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR7_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[7]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR8_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[8]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR8_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[8]"."PUNTO_COGIDA"."Y"'
AIMEN_SECTOR9_X = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[9]"."PUNTO_COGIDA"."X"'
AIMEN_SECTOR9_Y = 'ns=3;s="COMUNICACIÓN_SEIKI_DB"."SECTORES[9]"."PUNTO_COGIDA"."Y"'


SEIKI_BASTIDOR_READY = 'ns=5;i=2'
SEIKI_LANZAR_BASTIDOR = 'ns=5;i=5'
SEIKI_PANEL_ANCHO = 'ns=5;i=3'
SEIKI_PANEL_LARGO = 'ns=5;i=4'
SEIKI_ALTURA_MAX_TRAMO_0 = 'ns=5;i=26'
SEIKI_ALTURA_MAX_TRAMO_1 = 'ns=5;i=27'
SEIKI_ALTURA_MAX_TRAMO_2 = 'ns=5;i=28'
SEIKI_ALTURA_MAX_TRAMO_3 = 'ns=5;i=29'
SEIKI_SECTOR0_X = 'ns=5;i=6'
SEIKI_SECTOR0_Y = 'ns=5;i=7'
SEIKI_SECTOR1_X = 'ns=5;i=8'
SEIKI_SECTOR1_Y = 'ns=5;i=9'
SEIKI_SECTOR2_X = 'ns=5;i=10'
SEIKI_SECTOR2_Y = 'ns=5;i=11'
SEIKI_SECTOR3_X = 'ns=5;i=12'
SEIKI_SECTOR3_Y = 'ns=5;i=13'
SEIKI_SECTOR4_X = 'ns=5;i=14'
SEIKI_SECTOR4_Y = 'ns=5;i=15'
SEIKI_SECTOR5_X = 'ns=5;i=16'
SEIKI_SECTOR5_Y = 'ns=5;i=17'
SEIKI_SECTOR6_X = 'ns=5;i=18'
SEIKI_SECTOR6_Y = 'ns=5;i=19'
SEIKI_SECTOR7_X = 'ns=5;i=20'
SEIKI_SECTOR7_Y = 'ns=5;i=21'
SEIKI_SECTOR8_X = 'ns=5;i=22'
SEIKI_SECTOR8_Y = 'ns=5;i=23'
SEIKI_SECTOR9_X = 'ns=5;i=24'
SEIKI_SECTOR9_Y = 'ns=5;i=25'
SEIKI_PLC_ALIVE = 'ns=5;i=30'

def cargar_config():
    if not os.path.exists(CONFIG_PATH):
        return {}
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"Error leyendo {CONFIG_PATH}: {e}")
        return {}

def guardar_config(data: dict):
    """Guarda el diccionario de configuración en config_camara.json"""
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
        print("💾 Configuración guardada en", CONFIG_PATH)
    except Exception as e:
        print(f"Error guardando configuración: {e}")

# ---------------- Constantes ----------------
NOMBRES_RECUADROS = [
    "clin Altura Tramo 0",
    "clin Altura Tramo 1",
    "clin Altura Tramo 2",
    "clin Altura Tramo 3",
    "clin Ancho",
    "clin Largo",
    "clin Sector 0 X",
    "clin Sector 0 Y",
    "clin Sector 1 X",
    "clin Sector 1 Y"
]

class EmisorSalida:
    def __init__(self, imprimir_signal):
        self.imprimir_signal = imprimir_signal

    def write(self, mensaje):
        if mensaje.strip():  
            self.imprimir_signal.emit(mensaje)

    def flush(self):
        pass


# ---------------- Pantalla Principal ----------------
class PantallaPrincipal(QWidget):
    signal_reset_db = pyqtSignal(bool)
    imprimir_signal = pyqtSignal(str)

    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        layout = QVBoxLayout()

        # --- RECUADRO CLINCHADORA ---
        clinchadora_group = QGroupBox("🔧 CLINCHADORA")
        clinchadora_layout = QGridLayout()

        led_clinchadora = QLabel()
        led_clinchadora.setFixedSize(24, 24)
        led_clinchadora.setStyleSheet("background-color: #28A745; border-radius: 12px;")
        clinchadora_layout.addWidget(led_clinchadora, 0, 0)
        clinchadora_layout.addWidget(QLabel("COMUNICACION OK"), 0, 1)


        boton_pantalla = QPushButton("PANTALLA CLINCHADORA")
        boton_pantalla.setObjectName("roundButton")
        boton_pantalla.setFixedHeight(40)
        boton_pantalla.setMinimumWidth(200)
        boton_pantalla.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        clinchadora_layout.addWidget(boton_pantalla, 3, 0, 1, 2)

        clinchadora_group.setLayout(clinchadora_layout)

                # --- RECUADRO CAMARA HIK ---
        camara_group = QGroupBox("📷 CÁMARA HIK")
        camara_layout = QGridLayout()

        # LED de cámara como atributo de la clase
        self.led_camara = QLabel()
        self.led_camara.setFixedSize(24, 24)
        self.led_camara.setStyleSheet("background-color: #DC3545; border-radius: 12px;")  # 🔴 Rojo por defecto

        camara_layout.addWidget(self.led_camara, 0, 0)
        camara_layout.addWidget(QLabel("COMUNICACION OK"), 0, 1)

        # Botón configuración
        boton_conf = QPushButton("P.CONF CAMARA")
        boton_conf.setObjectName("roundButton")
        boton_conf.setFixedHeight(40)
        boton_conf.setMinimumWidth(200)
        boton_conf.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))
        camara_layout.addWidget(boton_conf, 1, 0, 1, 2)

        camara_group.setLayout(camara_layout)

        # --- RECUADRO VARIOS ---
        varios_group = QGroupBox("🛠️ VARIOS")
        varios_layout = QVBoxLayout()
        varios_layout.setContentsMargins(10, 10, 10, 10)
        varios_layout.setSpacing(10)

        # --- Parte superior: FPS y coordenadas ---
        top_layout = QVBoxLayout()
        top_layout.setSpacing(5)

        # FPS
        self.lbl_fps = QLabel("FPS: ---")
        self.lbl_fps.setAlignment(Qt.AlignCenter)
        self.lbl_fps.setFixedHeight(30)
        self.lbl_fps.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #00FF7F;      /* verde brillante */
            background-color: #222;
            padding: 3px 6px;
            border-radius: 6px;
        """)
        top_layout.addWidget(self.lbl_fps)

        # Hexágono tratado
        hex_layout = QHBoxLayout()
        hex_layout.setSpacing(5)
        lbl_hex_title = QLabel("Hexágono tratado:")
        #lbl_hex_title.setStyleSheet("font-size: 24px; font-weight: bold; color: white;")
        lbl_hex_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #222;
            padding: 2px 6px;
            border-radius: 6px;
        """)

        self.lbl_hex_x = QLabel("X: ---")
        self.lbl_hex_x.setFixedHeight(48)
        self.lbl_hex_x.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #87CEFA;
            background-color: #222;
            padding: 2px 6px;
            border-radius: 6px;
        """)

        self.lbl_hex_y = QLabel("Y: ---")
        self.lbl_hex_y.setFixedHeight(48)
        self.lbl_hex_y.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #87CEFA;
            background-color: #222;
            padding: 2px 6px;
            border-radius: 6px;
        """)

        hex_layout.addWidget(lbl_hex_title)
        hex_layout.addWidget(self.lbl_hex_x)
        hex_layout.addWidget(self.lbl_hex_y)
        hex_layout.addStretch(1)

        top_layout.addLayout(hex_layout)

        # Agregar la parte superior al layout principal del grupo
        varios_layout.addLayout(top_layout)

        # --- Spacer para empujar botones al fondo ---
        varios_layout.addStretch(1)

        # --- Parte inferior: botones ---
        self.boton_visualizar = QPushButton("VISUALIZAR ON")
        self.boton_visualizar.setObjectName("roundButton")
        self.boton_visualizar.setFixedHeight(40)
        self.boton_visualizar.setMinimumWidth(200)
        varios_layout.addWidget(self.boton_visualizar)

        self.btn_reset_db = QPushButton("🔄 Resetear DB")
        self.btn_reset_db.setObjectName("redButton")
        self.btn_reset_db.setFixedHeight(40)
        self.btn_reset_db.setMinimumWidth(200)
        self.btn_reset_db.clicked.connect(self.resetear_db)
        varios_layout.addWidget(self.btn_reset_db)

        varios_group.setLayout(varios_layout)



        # --- RECUADRO PLC REMACHADO ---
        plc_group = QGroupBox("⚙️ PLC REMACHADO")
        plc_layout = QGridLayout()

        led_plc = QLabel()
        led_plc.setFixedSize(24, 24)
        led_plc.setStyleSheet("background-color: #28A745; border-radius: 12px;")
        plc_layout.addWidget(led_plc, 0, 0)
        plc_layout.addWidget(QLabel("COMUNICACION OK"), 0, 1)

        boton_plc = QPushButton("P.PLC REMACHADO")
        boton_plc.setObjectName("roundButton")
        boton_plc.setFixedHeight(40)
        boton_plc.setMinimumWidth(200)
        boton_plc.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(3))
        plc_layout.addWidget(boton_plc, 1, 0, 1, 2)

        plc_group.setLayout(plc_layout)

        # --- Layout superior ---
        top_layout = QHBoxLayout()
        top_layout.addWidget(clinchadora_group)
        top_layout.addWidget(camara_group)
        top_layout.addWidget(plc_group)

        # --- RECUADRO IMAGENES ---
        imagen_group = QGroupBox("🖼️ PANTALLA IMÁGENES HEXÁGONOS TRATADA")
        self.imagen_label = QLabel("Aquí irán las imágenes procesadas")
        self.imagen_label.setAlignment(Qt.AlignCenter)
        #self.imagen_label.setFixedSize(1024, 683)
        self.imagen_label.setFixedSize(ancho, alto)

        self.imagen_label.setStyleSheet("background-color: #111; border: 2px solid #555;")
        imagen_layout = QVBoxLayout()
        imagen_layout.addWidget(self.imagen_label)

        # QTextEdit para terminal
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setFixedHeight(200)  # ajustable
        self.terminal_output.setStyleSheet("background-color: black; color: white; font-family: Consolas;")
        imagen_layout.addWidget(self.terminal_output)

        imagen_group.setLayout(imagen_layout)

        # --- Botón inferior derecho ---
        boton_visualizar = QPushButton("VISUALIZAR ON")
        boton_visualizar.setObjectName("roundButton")

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(imagen_group, 1)
        bottom_layout.addWidget(varios_group)




        # --- Layout final ---
        layout.addLayout(top_layout)
        layout.addLayout(bottom_layout)
        self.setLayout(layout)
        
        self.imprimir_signal.connect(self.terminal_output.append)

        # Redirigir salidas a terminal
        sys.stdout = EmisorSalida(self.imprimir_signal)
        sys.stderr = EmisorSalida(self.imprimir_signal)
        


    def resetear_db(self):
        self.signal_reset_db.emit(True)


# ---------------- Pantalla Clinchadora ----------------
class PantallaClinchadora(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        main_layout = QHBoxLayout()
        main_layout.setSpacing(10)

        clinchadora_group = QGroupBox("🔧 CLINCHADORA")
        clinchadora_layout = QVBoxLayout()
        clinchadora_layout.setSpacing(8)
        clinchadora_layout.setContentsMargins(10, 10, 10, 10)

        # Función para crear fila LED + label
        def crear_fila_led_label(texto):
            led = QLabel()
            led.setFixedSize(32, 32)
            led.setStyleSheet("background-color: #3399FF; border-radius: 6px; border: 2px solid #222;")
            label = QLabel(texto)
            label.setFixedHeight(32)
            label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
            row = QHBoxLayout()
            row.addWidget(led)
            row.addWidget(label)
            row.addStretch(1)
            return led, label, row

        # Crear filas
        self.led_bastidor, label_bastidor, row_bastidor = crear_fila_led_label("BASTIDOR READY")
        self.led_flanco, label_flanco, row_flanco = crear_fila_led_label("FLANCO BOTON")
        self.led_expulsar, label_expulsar, row_expulsar = crear_fila_led_label("PULSADOR EXPULSAR")

        # Agregar filas al layout
        clinchadora_layout.addLayout(row_bastidor)
        clinchadora_layout.addLayout(row_flanco)
        clinchadora_layout.addLayout(row_expulsar)

        # Grid recuadros
        self.int_labels = []
        self.int_names = []
        grid_layout = QGridLayout()
        grid_layout.setSpacing(6)
        for i, nombre in enumerate(NOMBRES_RECUADROS):
            frame = QFrame()
            frame.setStyleSheet("background-color: #A9C1E8; border-radius: 6px;")
            frame.setFixedSize(220, 36)
            layout_frame = QHBoxLayout()
            layout_frame.setContentsMargins(8, 0, 8, 0)
            name_lbl = QLabel(nombre)
            name_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            name_lbl.setStyleSheet("font-weight: bold; color: #222; font-size: 15px;")
            value_lbl = QLabel("---")
            value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_lbl.setStyleSheet(
                "font-weight: bold; color: #1A237E; font-size: 18px; background: #fff; "
                "border-radius: 4px; padding: 2px 10px;"
            )
            layout_frame.addWidget(name_lbl, 2)
            layout_frame.addWidget(value_lbl, 1)
            frame.setLayout(layout_frame)

            row = i // 2
            col = i % 2
            grid_layout.addWidget(frame, row, col)
            self.int_names.append(name_lbl)
            self.int_labels.append(value_lbl)

        clinchadora_layout.addLayout(grid_layout)
        clinchadora_group.setLayout(clinchadora_layout)

        # Right buttons
        right_layout = QVBoxLayout()
        right_layout.addStretch(1)
        self.btn_bastidor = QPushButton("Bastidor Ready")
        self.btn_bastidor.setFixedSize(140, 48)
        self.btn_bastidor.setObjectName("greenButton")
        right_layout.addWidget(self.btn_bastidor)

        self.btn_flanco = QPushButton("Flanco Boton")
        self.btn_flanco.setFixedSize(140, 48)
        self.btn_flanco.setObjectName("greenButton")
        right_layout.addWidget(self.btn_flanco)
        right_layout.addStretch(10)

        main_layout.addWidget(clinchadora_group, 3)
        main_layout.addLayout(right_layout, 1)

        # Bottom volver
        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch(1)
        boton_volver = QPushButton("Volver a Principal")
        boton_volver.setObjectName("roundButton")
        boton_volver.setFixedWidth(180)
        boton_volver.clicked.connect(lambda: stacked_widget.setCurrentIndex(0))
        bottom_layout.addWidget(boton_volver)

        final_layout = QVBoxLayout()
        final_layout.addLayout(main_layout)
        final_layout.addLayout(bottom_layout)
        self.setLayout(final_layout)

        # Conexiones
        self.btn_bastidor.clicked.connect(self.toggle_bastidor)
        self.btn_flanco.clicked.connect(self.toggle_flanco)
        

    def toggle_bastidor(self):
        color = self.led_bastidor.styleSheet()
        if "#3399FF" in color:
            self.led_bastidor.setStyleSheet("background-color: #28A745; border-radius: 6px; border: 2px solid #222;")
        else:
            self.led_bastidor.setStyleSheet("background-color: #3399FF; border-radius: 6px; border: 2px solid #222;")

    def toggle_flanco(self):
        color = self.led_flanco.styleSheet()
        if "#3399FF" in color:
            self.led_flanco.setStyleSheet("background-color: #28A745; border-radius: 6px; border: 2px solid #222;")
        else:
            self.led_flanco.setStyleSheet("background-color: #3399FF; border-radius: 6px; border: 2px solid #222;")



# ---------------- Pantalla Camara ----------------
class PantallaCamara(QWidget):
    puntos_asignados = pyqtSignal(list)   # señal para enviar los 4 puntos
    enviar_distancia = pyqtSignal(int)     # señal para enviar distancia mínima hexágonos
    enviar_offset_x = pyqtSignal(int)      # señal para offset primer punto en X
    enviar_offset_y = pyqtSignal(int)      # señal para offset primer punto en Y
    solicitar_seleccion_puntos = pyqtSignal() # señal para solicitar selección interactiva de puntos
    enviar_factor_mm = pyqtSignal(float)  # señal para enviar factor de conversión mm/px

    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.camera = None
        self.puntos = None  # valores por defecto
        self.offset_x = 0
        self.offset_y = 0
        self.factor_mm_px = 1.0
        self.distancia_minima = None  # valor para YOLO
        self.init_ui()
    

    def init_ui(self):
        self.setWindowTitle("Configuración de Cámara")
        self.setFixedWidth(500)  # solo ancho fijo
        self.setMinimumHeight(600)  # altura mínima, pero crece si hace falta


        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        '''# Estado de cámara
        estado_layout = QHBoxLayout()
        self.led_camara = QLabel()
        self.led_camara.setFixedSize(20, 20)
        self.led_camara.setStyleSheet("background-color: #DC3545; border-radius: 10px;")
        lbl_estado = QLabel("Comunicación cámara")
        lbl_estado.setStyleSheet("font-weight: bold; font-size: 14px;")
        estado_layout.addWidget(self.led_camara)
        estado_layout.addWidget(lbl_estado)
        estado_layout.addStretch()'''
        # Estado de cámara
        estado_layout = QHBoxLayout()
        self.led_camara = QLabel()
        self.led_camara.setFixedSize(24, 24)
        self.led_camara.setStyleSheet("background-color: #DC3545; border-radius: 10px;")
        lbl_estado = QLabel("Comunicación cámara")
        lbl_estado.setStyleSheet("font-weight: bold; font-size: 14px;")
        estado_layout.addWidget(self.led_camara)
        estado_layout.addWidget(lbl_estado)
        estado_layout.addStretch()


        # Grupo parámetros de cámara
        grp_camara = QGroupBox("📷 Parámetros de cámara")
        form_layout = QGridLayout()
        form_layout.setSpacing(10)

        self.expo_label = QLabel("Exposición (µs):")
        self.expo_input = QLineEdit()
        self.expo_input.setPlaceholderText("Ej: 5000")

        self.gain_label = QLabel("Ganancia (dB):")
        self.gain_input = QLineEdit()
        self.gain_input.setPlaceholderText("Ej: 0.5")

        form_layout.addWidget(self.expo_label, 0, 0)
        form_layout.addWidget(self.expo_input, 0, 1)
        form_layout.addWidget(self.gain_label, 1, 0)
        form_layout.addWidget(self.gain_input, 1, 1)

        

        self.btn_aplicar = QPushButton("Aplicar Configuración")
        self.btn_aplicar.setObjectName("greenButton")
        self.btn_aplicar.setFixedHeight(40)
        self.btn_aplicar.setMinimumWidth(150)
        self.btn_aplicar.clicked.connect(self.aplicar_config)

        form_layout.addWidget(self.btn_aplicar, 2, 0, 1, 2)

        grp_camara.setLayout(form_layout)

        # Grupo YOLO
        grp_yolo = QGroupBox("👁️ YOLO")
        yolo_layout = QVBoxLayout()
        yolo_layout.setSpacing(10)

        self.lbl_dist = QLabel("Distancia mínima hexágonos (px):")
        self.dist_input = QLineEdit()
        self.dist_input.setPlaceholderText("Ej: 100")
        self.dist_input.returnPressed.connect(self.asignar_distancia)  # al dar Enter

        yolo_layout.addWidget(self.lbl_dist)
        yolo_layout.addWidget(self.dist_input)

        self.lbl_offset_x = QLabel("Offset X primer hexágono (px):")
        self.offset_input = QLineEdit()
        self.offset_input.setPlaceholderText("Ej: 10")
        self.offset_input.returnPressed.connect(self.asignar_offset_x)

        yolo_layout.addWidget(self.lbl_offset_x)
        yolo_layout.addWidget(self.offset_input)

        self.lbl_offset_y = QLabel("Offset Y primer hexágono (px):")
        self.offset_y_input = QLineEdit()
        self.offset_y_input.setPlaceholderText("Ej: 5")
        self.offset_y_input.returnPressed.connect(self.asignar_offset_y)

        yolo_layout.addWidget(self.lbl_offset_y)
        yolo_layout.addWidget(self.offset_y_input)

        self.lbl_factor_mm = QLabel("Factor mm por pixel:")
        self.factor_input = QLineEdit()
        self.factor_input.setPlaceholderText("Ej: 0.15")
        self.factor_input.returnPressed.connect(self.asignar_factor_mm)

        yolo_layout.addWidget(self.lbl_factor_mm)
        yolo_layout.addWidget(self.factor_input)


        grp_yolo.setLayout(yolo_layout)


        # Grupo calibración
        grp_calib = QGroupBox("🎯 Calibración de perspectiva")
        calib_layout = QVBoxLayout()
        calib_layout.setSpacing(12)

        grid_puntos = QGridLayout()
        grid_puntos.setSpacing(10)

        self.puntos_inputs = []  # Lista para almacenar las tuplas de QLabel (X, Y)
        headers = ["X", "Y"]
        grid_puntos.addWidget(QLabel("Punto"), 0, 0)

        # Encabezados
        for j, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-weight: bold;")
            grid_puntos.addWidget(lbl, 0, j+1)

        # Filas de Puntos con QLabels
        for i in range(4):
            lbl = QLabel(f"📍 Punto {i+1}")
            grid_puntos.addWidget(lbl, i+1, 0)
            
            # Reemplazo de QLineEdit por QLabel
            lbl_x = QLabel("N/A")  # Etiqueta para la coordenada X
            lbl_x.setAlignment(Qt.AlignCenter)
            lbl_x.setStyleSheet("color: #007bff;") 
            
            lbl_y = QLabel("N/A")  # Etiqueta para la coordenada Y
            lbl_y.setAlignment(Qt.AlignCenter)
            lbl_y.setStyleSheet("color: #007bff;")

            # Almacenar las etiquetas
            self.puntos_inputs.append((lbl_x, lbl_y))
            
            grid_puntos.addWidget(lbl_x, i+1, 1)
            grid_puntos.addWidget(lbl_y, i+1, 2)

        calib_layout.addLayout(grid_puntos)

        self.btn_asignar = QPushButton("Lanzar selección puntos")
        self.btn_asignar.setObjectName("greenButton")
        calib_layout.addWidget(self.btn_asignar, alignment=Qt.AlignCenter)

        grp_calib.setLayout(calib_layout)


        # Botón volver
        self.boton_volver = QPushButton("Volver a Principal")
        self.boton_volver.setObjectName("roundButton")

        # Layout principal
        main_layout.addLayout(estado_layout)
        cam_yolo_layout = QHBoxLayout()
        cam_yolo_layout.addWidget(grp_camara, 1)
        cam_yolo_layout.addWidget(grp_yolo, 1)
        main_layout.addLayout(cam_yolo_layout)
        main_layout.addWidget(grp_calib)
        main_layout.addStretch()
        main_layout.addWidget(self.boton_volver, alignment=Qt.AlignCenter)

        self.setLayout(main_layout)

        # Conexiones
        self.btn_asignar.clicked.connect(self.solicitar_seleccion_puntos)
        self.boton_volver.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))

    def set_camera(self, camera):
        """Recibe la cámara desde PLCWorker"""
        self.camera = camera
        if self.camera:
            # ✅ Cámara conectada
            self.led_camara.setStyleSheet("background-color: #28A745; border-radius: 12px;")  # 🟢 Verde

            # Solo rellenar si los QLineEdit están vacíos
            if hasattr(self.camera, "exposure") and self.camera.exposure and not self.expo_input.text():
                self.expo_input.setText(str(self.camera.exposure))
            if hasattr(self.camera, "gain") and self.camera.gain and not self.gain_input.text():
                self.gain_input.setText(str(self.camera.gain))
        else:
            # ❌ Cámara no disponible
            self.led_camara.setStyleSheet("background-color: #DC3545; border-radius: 12px;")  # 🔴 Rojo

    def asignar_factor_mm(self):
        """Lee el factor mm/px y lo envía al hilo + guarda en JSON"""
        try:
            self.factor_mm_px = float(self.factor_input.text())
            self.enviar_factor_mm.emit(self.factor_mm_px)
            print("👉 Factor mm/px =", self.factor_mm_px)

            # 💾 Guardar en JSON
            self.stacked_widget.config.setdefault("yolo", {})
            self.stacked_widget.config["yolo"]["factor_mm_por_pixel"] = self.factor_mm_px
            guardar_config(self.stacked_widget.config)

            QMessageBox.information(
                self, "YOLO", f"Factor mm/px asignado: {self.factor_mm_px}"
            )
        except ValueError:
            QMessageBox.warning(self, "Error", "Introduce un número válido para el factor mm/px")



    def asignar_distancia(self):
        """Lee el valor del campo y lo asigna a una variable"""
        try:
            self.distancia_minima = float(self.dist_input.text())
            QMessageBox.information(self, "YOLO", f"Distancia mínima asignada: {self.distancia_minima}px")
            print("👉 Distancia mínima YOLO =", self.distancia_minima)
            self.enviar_distancia.emit(int(self.distancia_minima))
            # 💾 Guardar en JSON
            self.stacked_widget.config.setdefault("yolo", {})
            self.stacked_widget.config["yolo"]["distancia_minima_hexagonos"] = int(self.distancia_minima)
            guardar_config(self.stacked_widget.config)
        except ValueError:
            QMessageBox.warning(self, "Error", "Introduce un número válido para la distancia mínima")

    def aplicar_config(self):
        if not self.camera:
            QMessageBox.warning(self, "Error", "La cámara no está disponible")
            return

        try:
            exposure = int(self.expo_input.text()) if self.expo_input.text() else None
            gain = float(self.gain_input.text()) if self.gain_input.text() else None

            ret = self.camera.set_param(exposure=exposure, gain=gain)
            if ret == 0:
                # Actualizar QLineEdit con valores reales de la cámara
                self.expo_input.setText(str(self.camera.exposure))
                self.gain_input.setText(str(self.camera.gain))

                # 💾 GUARDAR EN JSON DESPUÉS DE MODIFICAR EN UI
                self.stacked_widget.config.setdefault("camara", {})
                self.stacked_widget.config["camara"]["exposicion"] = self.camera.exposure
                self.stacked_widget.config["camara"]["ganancia"] = self.camera.gain

                guardar_config(self.stacked_widget.config)
                QMessageBox.information(self, "Éxito", "Parámetros aplicados correctamente")
            else:
                QMessageBox.warning(self, "Error", f"No se pudo aplicar configuración: {self.camera.strError}")

        except ValueError:
            QMessageBox.warning(self, "Error", "Introduce valores numéricos válidos")

    def asignar_offset_x(self):
        """Lee el offset X y lo envía al hilo + guarda en JSON"""
        try:
            self.offset_x = int(self.offset_input.text())
            self.enviar_offset_x.emit(self.offset_x)
            print("👉 Offset X primer hexágono =", self.offset_x)

            # 💾 Guardar en JSON
            self.stacked_widget.config.setdefault("yolo", {})
            self.stacked_widget.config["yolo"]["offset_x_primer_hexagono"] = self.offset_x
            guardar_config(self.stacked_widget.config)

            QMessageBox.information(
                self, "YOLO", f"Offset X asignado: {self.offset_x}px"
            )
        except ValueError:
            QMessageBox.warning(self, "Error", "Introduce un número válido para el offset X")

    def asignar_offset_y(self):
        """Lee el offset Y y lo envía al hilo + guarda en JSON"""
        try:
            self.offset_y = int(self.offset_y_input.text())
            self.enviar_offset_y.emit(self.offset_y)
            print("👉 Offset Y primer hexágono =", self.offset_y)

            # 💾 Guardar en JSON
            self.stacked_widget.config.setdefault("yolo", {})
            self.stacked_widget.config["yolo"]["offset_y_primer_hexagono"] = self.offset_y
            guardar_config(self.stacked_widget.config)

            QMessageBox.information(
                self, "YOLO", f"Offset Y asignado: {self.offset_y}px"
            )
        except ValueError:
            QMessageBox.warning(self, "Error", "Introduce un número válido para el offset Y")


# ---------------- Pantalla PLC ----------------
class PantallaPLC(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Pantalla de configuración del PLC REMACHADO"))
        boton = QPushButton("Volver a Principal")
        boton.setObjectName("roundButton")
        boton.clicked.connect(lambda: stacked_widget.setCurrentIndex(0))
        layout.addWidget(boton)
        self.setLayout(layout)


# ---------------- PLC Worker ----------------
class PLCWorker(QThread):
    datos_actualizados = pyqtSignal(dict)
    resultado_detectado = pyqtSignal(object)
    imagen_procesada = pyqtSignal(object)
    camara_listo = pyqtSignal(object)  
    fps_actualizado = pyqtSignal(int)
    coordenadas_actualizadas = pyqtSignal(float, float)  # X, Y
    envio_pts_select = pyqtSignal(list)
    

    def __init__(self, client=None, parent=None):
        super().__init__(parent)
        self.client = client
        self._running = True
        self.camera = None
        self.yolo = None
        self.results = None
        self.img_result = None
        self.db_number = 601
        self.puntos = [[8, 186], [627, 179], [8, 367], [626, 396]]  
        #self.puntos = []
        self.centros_ordenados = None  
        self.camera_connected = False
        self.lock = threading.Lock()
        self.data = bytearray(200)
        self.bool_cam_online = False
        self.bool_in_cam_detect = False
        self.bool_in_cam_confirm_next_detect = False
        self.lado_bb = None
        self.img_proc = None
        self.img_orig = None
        self.img_transf = None
        self._parpadeo = False
        self._ultimo_tiempo_parpadeo = time.time()
        self._intervalo_parpadeo = 0.5  
        self.distancia_mínima = 100
        self.offset_x_primer_hex = 0 
        self.offset_y_primer_hex = 0 
        self.factor_px_to_mm = 1.0
        self.img_traformer = select_pixel_v4.ImageTransformer()
        self.img_traformer.puntos = self.puntos
        self.img_traformer.calculate_matrix()
        self.trayectoria_robot = []
        self.ruta_imagen_transformacion = r"1641556909.694041.jpg"

        self.bool_db_reset = False

    @pyqtSlot(int)
    def set_valor(self, valor):
        self.distancia_mínima = valor
        print(f"Valor actualizado en el hilo: {self.distancia_mínima}")

    @pyqtSlot(bool)
    def handle_reset_db(self, should_reset):
        if should_reset:
           self.bool_db_reset = True
           print(f"Valor RECIBIDO: {should_reset}")

    def set_puntos(self, puntos):
        """Recibe los 4 puntos (x,y) desde configuración o UI y los aplica directamente"""
        # puntos tipo [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
        self.puntos = np.float32(puntos)

        # Actualizar el transformer con estos puntos
        self.img_traformer.puntos = self.puntos
        self.img_traformer.calculate_matrix()

        # Enviar puntos a la UI para que se muestren
        self.envio_pts_select.emit(self.puntos.tolist())
        print("Puntos actualizados en PLCWorker:", self.puntos)

    @pyqtSlot()
    def seleccionar_puntos_interactivo(self):
        try:
            img = self.camera.capture()
            self.img_traformer.select_points(img)
            #self.img_traformer.select_points(self.ruta_imagen_transformacion)
            self.puntos = np.float32(self.img_traformer.puntos)
            self.img_traformer.calculate_matrix()
            self.envio_pts_select.emit(self.puntos.tolist())
            print("Puntos seleccionados manualmente en PLCWorker:", self.puntos)
        except Exception as e:
            print(f"Error en selección interactiva de puntos: {e}")

    @pyqtSlot(int)
    def set_offset_x(self, valor):
        self.offset_x_primer_hex = valor
        print(f"Offset X primer hexágono actualizado en hilo: {self.offset_x_primer_hex}")

    @pyqtSlot(float)
    def set_factor_mm(self, factor):
        self.factor_px_to_mm = float(factor)
        print(f"Factor mm/px actualizado en hilo: {self.factor_px_to_mm}")

    @pyqtSlot(int)
    def set_offset_y(self, valor):
        self.offset_y_primer_hex = valor
        print(f"Offset Y primer hexágono actualizado en hilo: {self.offset_y_primer_hex}")

    def run(self):
        # Inicializar YOLO
        self.yolo = YoloModelV5()

        # Inicializar cámara
        self.inicializar_camara()

        while self._running:
            try:
                if self.bool_db_reset:
                    try:
                        # 64 BOOL (8 bytes) + 32 INT (64 bytes) + 32 FLOAT (128 bytes) = 200 bytes
                        data = bytearray(200) # Inicializa 200 bytes a cero (0)
                        
                        self.client.db_write(self.db_number, 0, data)
                        print("DB RESETEADA a 200 bytes.")
                        self.bool_db_reset = False 

                    except Exception as e:
                        print("Error", f"No se pudo resetear la DB: {e}")
                        
                data = self.leer_area()
                #time.sleep(0.02)
                valores = self.mapear_datos_2(data)
                # --- VERIFICAR ESTADO DE CÁMARA ---
                if self.camera is None or not self.camera_connected:
                    # La cámara está desconectada
                    print("⚠️ Cámara desconectada")
                    #  PLC indicando desconexión:
                    #self.out_cam_online_false(data)
                    self.bool_cam_online = False
                    # Reconectar camara 
                    self.cerrar_camara()
                    time.sleep(2)
                    self.inicializar_camara()
                    time.sleep(5)
                    self.escribir_bits(self.db_number, 2, bit0=self.bool_in_cam_detect, bit1=self.bool_cam_online, bit2=self.bool_in_cam_confirm_next_detect, bit3=False)
                    continue
                else:
                    #La cámara está conectada
                    #self.out_cam_online_true(data)
                    self.bool_cam_online = True

                if valores["cam_Reset"] == True:
                    print("🔄 Reset de cámara solicitado desde PLC")

                    # 1) marcar reset de DB
                    self.bool_db_reset = True

                    # 2) resetear todos los estados internos
                    self.reset_estado_interno()

                    # 3) escribir bits de estado (todo a False menos cam_online si quieres)
                    self.escribir_bits(
                        self.db_number,
                        2,
                        bit0=self.bool_in_cam_detect,
                        bit1=self.bool_cam_online,
                        bit2=self.bool_in_cam_confirm_next_detect,
                        bit3=False
                    )

                    # saltamos al siguiente ciclo, no seguimos procesando esta iteración
                    continue

                if (valores["out_peticion_detect"] == True) and self.camera:
                    self.detectar_objeto()
                    centros = self.obtener_centros_hexagonos()
                    if self.img_proc is not None:
                        #img_a_enviar = cv2.flip(self.img_proc.copy(), -1)  # trabajar sobre copia
                        img_a_enviar = self.img_proc.copy() # trabajar sobre copia
                        self.imagen_procesada.emit(img_a_enviar)
                        
                    print("Centros detectados:", centros)
                    if centros.size > 0:
                        valores = self.mapear_datos_2(data)
                        #self.in_cam_detect_true(data)
                        self.bool_in_cam_detect = True
                        self.centros_ordenados = self.ordenar_centros(centros)
                        print("Centros ordenados:", self.centros_ordenados)
                        self.centros_ordenados = self.cortar_centros(self.centros_ordenados)
                        self.trayectoria_robot = self.construir_trayectoria_robot()
                        print("Trayectoria robot:", self.trayectoria_robot)
                    else:
                        valores = self.mapear_datos_2(data)
                        #self.in_cam_detect_false(data)
                        self.bool_in_cam_detect = False
                        print("No se detectaron centros para ordenar en True peticion.")
                else:
                    # si está vacío, escribir in_cam_Detect_False ...
                    if self.centros_ordenados is None:  #or self.centros_ordenados.size == 0:
                        valores = self.mapear_datos_2(data)
                        self.bool_in_cam_detect = False
                        self.bool_in_cam_confirm_next_detect = False
                        print("No se detectaron centros para ordenar en False peticion.")
                        self.coordenadas_actualizadas.emit(-1000000, -1000000)
                        self.imagen_procesada.emit(self.img_transf)
                    else:
                        centro_enviar = self.centros_ordenados[0]
                        valores = self.mapear_datos_2(data)
                        # ==============================
                        # ENVÍO AL PLC: TRAYECTORIA ROBOT
                        # ==============================
                        # x_br_px, y_br_px siempre en píxeles
                        if self.trayectoria_robot and len(self.trayectoria_robot) > 0:
                            x_br_px, y_br_px = self.trayectoria_robot[0]
                        else:
                            # Fallback: si por lo que sea no hay trayectoria, calculamos como antes
                            if self.img_transf is not None:
                                h, w = self.img_transf.shape[:2]
                                x_cv = float(centro_enviar[0])
                                y_cv = float(centro_enviar[1])

                                x_br_px = int((w - 1) - x_cv)
                                y_br_px = int((h - 1) - y_cv)
                            else:
                                x_br_px = int(centro_enviar[0])
                                y_br_px = int(centro_enviar[1])

                        # 🔹 Conversión a mm
                        x_mm = int(round(x_br_px * self.factor_px_to_mm))
                        y_mm = int(round(y_br_px * self.factor_px_to_mm))

                        # Saturar al rango de INT16 Siemens
                        x_mm = max(-32768, min(32767, x_mm))
                        y_mm = max(-32768, min(32767, y_mm))

                        # Enviar al PLC en milímetros (INT con signo)
                        self.escribir_dos_int(self.db_number, 8, x_mm, y_mm)
                        print(f"Enviado trayectoria al PLC (mm): ({x_mm}, {y_mm})")

                        # Enviar a la UI también en mm
                        self.coordenadas_actualizadas.emit(x_mm, y_mm)

 
                        if self.img_proc is not None:
                            t_actual = time.time()
                            if t_actual - self._ultimo_tiempo_parpadeo >= self._intervalo_parpadeo:
                                self._parpadeo = not self._parpadeo  # alterna True/False
                                self._ultimo_tiempo_parpadeo = t_actual

                            img_a_enviar = self.img_orig.copy()  # trabajar sobre copia
                            self.imagen_procesada.emit(img_a_enviar)
                            # dibujar cuadrados de otros centros
                            for centro in self.centros_ordenados[1:]:
                                self.dibujar_cuadrado(img_a_enviar, centro[0], centro[1], self.lado_bb, (0,0,255), 1)
                            self.imagen_procesada.emit(img_a_enviar)
                            # dibujar centro_enviar solo si _parpadeo es True
                            if self._parpadeo:
                                self.dibujar_cuadrado(img_a_enviar, centro_enviar[0], centro_enviar[1], self.lado_bb, (0,255,0), 1)

                            # emitir imagen procesada
                            self.imagen_procesada.emit(img_a_enviar)


                        
                        # Check cam_next_detect
                        valores = self.mapear_datos_2(data)
                        if valores["out_cam_Next_Detect"] == True:
                            # Check in_Cam_confirm_Next_Detect
                            valores = self.mapear_datos_2(data) 
                            if self.bool_in_cam_confirm_next_detect == False:
                                # Eliminar el primer centro y el primer movimiento si hay más
                                if self.centros_ordenados.shape[0] > 0:
                                    self.centros_ordenados = np.delete(self.centros_ordenados, 0, axis=0)

                                    if self.trayectoria_robot and len(self.trayectoria_robot) > 0:
                                        self.trayectoria_robot.pop(0)
                                    
                                    # Ponemos a false centros detectados si es el último centro
                                    if self.centros_ordenados.shape[0] == 1:
                                        self.bool_in_cam_detect = False
                                        print("Último centro enviado")

                                    valores = self.mapear_datos_2(data)
                                    self.img_proc = self.img_transf
                                    self.bool_in_cam_confirm_next_detect = True
                                else:
                                    self.bool_in_cam_detect = False
                                    print("No quedan más centros para enviar.")
                        else:
                            valores = self.mapear_datos_2(data)
                            self.bool_in_cam_confirm_next_detect = False


                self.datos_actualizados.emit(valores)
                self.escribir_bits(self.db_number, 2, bit0=self.bool_in_cam_detect, bit1=self.bool_cam_online, bit2=self.bool_in_cam_confirm_next_detect, bit3=False)
            except Exception as e:
                print(f"Error en lectura PLC o cámara: {e}")
                time.sleep(0.1)
                continue
            self.fps = self.camera.get_fps()
            self.fps_actualizado.emit(self.fps)
            #time.sleep(0.05)
        
        # Cierre seguro
        self.cerrar_camara()
        if self.client.get_connected():
            self.client.disconnect()
        print("Hilo PLC detenido")


    def inicializar_camara(self):
        """Función para inicializar la cámara"""
        self.camera = HikCamera()
        ret = self.camera.enum_devices()
        if ret != 0 or self.camera.get_cam_index() == 0:
            print("No se encontró ninguna cámara")
            self.camara_listo.emit(None)
            self.camera = None
        else:
            ret = self.camera.open_device()
            if ret != 0:
                print(f"No se pudo abrir la cámara, error {ret}")
                self.camara_listo.emit(None)
                self.camera = None
            else:
                self.camera.start_grabbing()
                print("Cámara iniciada correctamente")
                # PLC enviamos camara online
                data = self.client.db_read(self.db_number, 0, 200)
                self.out_cam_online_true(data)
                self.camara_listo.emit(self.camera)

    def cerrar_camara(self):
        """Cierra la cámara de forma segura"""
        if self.camera:
            if getattr(self.camera, "isGrabbing", False):
                self.camera.stop_grabbing()
                print("Grabación de cámara detenida")
            if getattr(self.camera, "isOpen", False):
                self.camera.close_device()
                print("Cámara cerrada")
            self.camera = None

    def set_camera_connected(self, conectado: bool):
        self.camera_connected = conectado

    def out_cam_online_false(self, data: bytes):
        """Escribe a False el bit out_cam_Online (byte 2, bit 1)"""
        try:
            if not self.client.get_connected(): return
            data[2] &= 0b11111101  # Bit1 del byte 2 a False
            self.client.db_write(self.db_number, 0, data)
        except Exception as e:
            print(f"Error escribiendo PLC: {e}")

    def out_cam_online_true(self, data: bytes):
        "Escribe a true el bit out_cam_Online (byte 2, bit 1)"
        try:
            if not self.client.get_connected(): return
            data[2] |= 0b00000010  # Bit1 del byte 2 a True
            self.client.db_write(self.db_number, 0, data)
        except Exception as e:
            print(f"Error escribiendo PLC: {e}")

    def reset_estado_interno(self):
        """Resetea todos los estados relacionados con detecciones / trayectoria."""
        # Listas y resultados
        self.centros_ordenados = None
        self.trayectoria_robot = []
        self.results = None

        # Imágenes
        self.img_proc = None
        self.img_orig = None
        self.img_transf = None

        # Flags de cámara / comunicación
        self.bool_in_cam_detect = False
        self.bool_in_cam_confirm_next_detect = False
        # OJO: bool_cam_online la dejamos según el estado real de cámara
        # self.bool_cam_online = False  # solo si quieres forzarla a false

        # Limpiar coordenadas en la UI
        self.coordenadas_actualizadas.emit(-1000000, -1000000)

        # Opcional: limpiar INTs de posición en el PLC (x,y) -> 0
        try:
            self.escribir_dos_int(self.db_number, 8, 0, 0)
        except Exception as e:
            print(f"Error limpiando INTs de posición: {e}")
        
        
    def detectar_objeto(self):
        global guardar
        if not self.camera or not self.yolo:
            return

        img = self.camera.capture()

        if img is None:
            print("⚠️ No se pudo capturar imagen de la cámara")
            return
        #ruta = r"G:\Mi unidad\Proyectos\Otis\App Python\Imagenes\sin tratar\1641556932.9621134.jpg"
        ruta = r"C:\Users\Seiki\Documents\Programa_OTIS\1641556909.694041.jpg"
        #ruta = r"C:\Users\Seiki\Documents\CLIENTES\OTIS\Scripts Hexagon Locator\screener\Perspectiva\1646720919.4561257.jpg"
        #img = cv2.imread(ruta)
        # Copias seguras

        

        # Convertir a RGB si es necesario
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        elif img.ndim == 3 and img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)
        self.img_proc = cv2.undistort(cv2.flip(img.copy(), -1), mtx, distMtx, None, camMxt)
        #self.img_proc = cv2.undistort(img.copy(), mtx, distMtx, None, camMxt)
    
        
        if guardar:
            try:
                cv2.imwrite("undistort.bmp", self.img_proc)
                print("guardada")
            except Exception as e:
                print(f"Error guardando                                        img: {e}")

            guardar  =   False
        
        self.img_transf = self.img_traformer.apply_transformation_to_cv_image(self.img_proc)
        self.img_proc = self.img_transf.copy()
        self.img_orig = self.img_transf.copy()
        #self.img_orig = cv2.flip(self.img_transf.copy(),-1)
        lista_resultados, _ = self.yolo.predict(self.img_proc)
        self.results = lista_resultados

        if len(self.results) > 0:
            self.lado_bb = math.sqrt(self.results[0]['area'])




    def dibujar_cuadrado(self, img, cx, cy, lado, color=(0, 255, 0), grosor=2):
        """
        Dibuja un cuadrado centrado en (cx, cy) en la imagen.

        Args:
            img (numpy.ndarray): Imagen sobre la que dibujar.
            cx (int): Coordenada x del centro del cuadrado.
            cy (int): Coordenada y del centro del cuadrado.
            lado (int): Longitud del lado del cuadrado.
            color (tuple): Color BGR. Default verde.
            grosor (int): Grosor de las líneas.
        
        Returns:
            numpy.ndarray: Imagen con el cuadrado dibujado.
        """
        mitad = lado // 2
        x1 = int(cx - mitad)
        y1 = int(cy - mitad)
        x2 = int(cx + mitad)
        y2 = int(cy + mitad)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, grosor)
        return img


    def obtener_centros_hexagonos(self):
        """
        Devuelve un array de todos los centros detectados de los hexágonos
        """
        if not self.results:
            return np.array([])  # vacío si no hay detecciones

        centros = [res['center'] for res in self.results if 'center' in res]

        mi_array = np.array(centros)
        for centro in mi_array:
            centro[0] = ancho-centro[0]
        
        return mi_array  # devuelve un numpy array


    def ordenar_centros(self, centros, tol_x=20):
        """
        Ordena los centros de hexágonos.

        - centros: np.array de forma (N,2), [[x1,y1],[x2,y2],...]
        - tol_x: tolerancia en pixeles para considerar alineación vertical
        """
        if len(centros) == 0:
            return centros

        # Primero, ordenar por x decreciente (derecha a izquierda)
        #centros_sorted = centros[np.argsort(-centros[:,0])]
        centros_sorted = centros[np.argsort(centros[:,0])]

        # Luego, dentro de cada columna vertical (x similar), ordenar por y decreciente (más abajo primero)
        resultado = []
        usados = np.zeros(len(centros_sorted), dtype=bool)

        for i in range(len(centros_sorted)):
            if usados[i]:
                continue
            grupo = [centros_sorted[i]]
            usados[i] = True
            # agrupar todos los que estén dentro de tol_x
            for j in range(i+1, len(centros_sorted)):
                if not usados[j] and abs(centros_sorted[j][0] - centros_sorted[i][0]) <= tol_x:
                    grupo.append(centros_sorted[j])
                    usados[j] = True
            # ordenar el grupo por y decreciente (más abajo primero)
            grupo = sorted(grupo, key=lambda p: -p[1])
            resultado.extend(grupo)

        # Una vez ordenados los centros, descartamos los que estén a una distancia mayor de 450 píxeles
        # entre dos consecutivos en la lista ordenada
        
        '''
        if len(resultado) > 1:
            resultado_filtrado = [resultado[0]]
            for k in range(1, len(resultado)):
                dist = np.linalg.norm(np.array(resultado[k]) - np.array(resultado_filtrado[-1]))
                if dist <= self.distancia_mínima:
                    resultado_filtrado.append(resultado[k])
            resultado = resultado_filtrado
'''
        # Solo se devolver la lista cuando el último del array ordenado esté a más de 450 píxeles
        # del final de la imagen, que es donde empieza en 0: es decir, la coordenada del último ha de ser 
        # mínimo 450. Sino, devolver array vacío
        if len(resultado) > 0 and resultado[-1][0] < self.distancia_mínima:
            return np.array([])
        
        return np.array(resultado)

    def cortar_centros(self, centros):
    
        resultado = []
        posi = 0


        for centro in centros:
            if ((centro[0] + self.distancia_mínima) < ancho):
                if (len(resultado) > 0):
                    # ya hay centros
                    if ((centro[0] - posi) < self.distancia_mínima):
                        resultado.append(centro)
                        posi = centro[0]
                    else:
                        break

                else:
                    # es el primer resultado
                    resultado.append(centro)
                    posi = centro[0]
            else:
                break

        return resultado


    def escribir_dos_int(self, db_number: int, byte_index: int, int1, int2):
        """
        Escribe dos enteros consecutivos en un DB (INT de Siemens, con signo).
        """
        int1 = int(int1)
        int2 = int(int2)

        # 👇 IMPORTANTE: signed=True para permitir negativos
        bytes_int1 = int1.to_bytes(2, byteorder='big', signed=True)
        bytes_int2 = int2.to_bytes(2, byteorder='big', signed=True)

        buffer = bytearray(bytes_int1 + bytes_int2)
        self.client.write_area(Areas.DB, db_number, byte_index, buffer)



    '''def escribir_bits(self, db_number: int, byte_index: int, bit0=None, bit1=None, bit2=None, bit3=None):
        byte_valor = 0
        if bit0 is not None: byte_valor |= (bit0 & 1) << 0
        if bit1 is not None: byte_valor |= (bit1 & 1) << 1
        if bit2 is not None: byte_valor |= (bit2 & 1) << 2
        if bit3 is not None: byte_valor |= (bit3 & 1) << 3

        buffer = bytearray([byte_valor])
        self.client.write_area(Areas.DB, db_number, byte_index, buffer)
        print(f"Byte escrito: {bin(byte_valor)} en DB{db_number} byte {byte_index}")'''

    def escribir_bits(self, db_number: int, byte_index: int, bit0=None, bit1=None, bit2=None, bit3=None):
        """
        Escribe los bits 0,1,2,3 de un byte en un DB sin leer el byte antes.

        Args:
            plc: cliente Snap7 conectado.
            db_number: número del DB.
            byte_index: índice del byte dentro del DB.
            bit0..bit3: valores 0 o 1 para cada bit. Si es None, el bit se pone a 0 por defecto.
        """
        # Construir el byte según los bits
        byte_valor = 0
        if bit0:
            byte_valor |= 1 << 0
        if bit1:
            byte_valor |= 1 << 1
        if bit2:
            byte_valor |= 1 << 2
        if bit3:
            byte_valor |= 1 << 3
        buffer = bytearray([byte_valor])
        self.client.write_area(Areas.DB, db_number, byte_index, buffer)
        #print(f"Byte escrito: {bin(byte_valor)} en DB{db_number} byte {byte_index}")

    def set_bit(client, db_number: int, byte_index: int, bit_index: int, value: bool):
        """
        Activa o desactiva un bit en un DB del PLC.

        Args:
            client: instancia de snap7.client.Client ya conectada.
            db_number: número del DB donde está el byte.
            byte_index: índice del byte a modificar (empezando en 0).
            bit_index: índice del bit a modificar (0-7, bit 0 = LSB).
            value: True para poner el bit en 1, False para ponerlo en 0.
        """
        # Leer el byte actual
        buffer = client.read_area(Areas.DB, db_number, byte_index, 1)

        # Modificar el bit
        if value:
            buffer[0] |= 1 << bit_index  # Activar bit
        else:
            buffer[0] &= ~(1 << bit_index)  # Desactivar bit

        # Escribir de nuevo el byte
        client.write_area(Areas.DB, db_number, byte_index, buffer)

    def leer_area(self):
        """ Leemos 1 byte """
        #valores = {}
        data = self.client.read_area(Areas.DB,self.db_number,0,1)
        '''primer_byte = buffer[0]

        # Extraer los bits individuales
        bit0 = (primer_byte >> 0) & 1  # bit menos significativo
        bit1 = (primer_byte >> 1) & 1
        bit2 = (primer_byte >> 2) & 1

        valores["out_peticion_detect"] = bit0
        valores["out_cam_Next_Detect"] = bit1'''

        return data

    def escribir_area(self):
        self.client.write_area(Areas.DB,self.db_number,0,self.areaInt)





    def escribir_centro_plc(self, data, centro):
        '''Escribe las coordenadas del centro (x,y) en los enteros int8 e int10 del PLC '''
        try:
            if not self.client.get_connected(): return
            data[8:10] = int(centro[0]).to_bytes(2, byteorder='big', signed=True)  # byte 8
            data[10:12] = int(centro[1]).to_bytes(2, byteorder='big', signed=True)  # byte 10
            self.client.db_write(self.db_number, 0, data)
        except Exception as e:
            print(f"Error escribiendo centro en PLC: {e}")

    def mapear_datos(self, data: bytes):
        """Mapea los datos leídos del PLC referentes a la cámara 
        (solo salidas del PLC) a un diccionario"""
        valores = {}
        valores["out_peticion_detect"] = get_bool(data, 0, 0)
        valores["out_cam_Next_Detect"] = get_bool(data, 0, 1)
        valores["cam_Reset"] = get_bool(data, 0, 2)
        valores["in_Cam_confirm_Next_Detect"] = get_bool(data, 2, 2)
        # Mapear 32 enteros desde el byte 8
        for i in range(32):
            valores[f"int{i}"] = get_int(data, 8 + i*2)
        # valores["mi_float0"] = get_real(data, 72)
        # valores["mi_float1"] = get_real(data, 76)
        return valores
    
    def mapear_datos_2(self, data: bytes):
        """Mapea los datos leídos del PLC referentes a la cámara 
        (solo salidas del PLC) a un diccionario"""
        valores = {}
        valores["out_peticion_detect"] = get_bool(data, 0, 0)
        valores["out_cam_Next_Detect"] = get_bool(data, 0, 1)
        valores["cam_Reset"] = get_bool(data, 0, 2)

        return valores
    
    def construir_trayectoria_robot(self):
        """
        Devuelve una lista de tuplas [(X0_abs, Y0_abs), (dX1, dY1), (dX2, dY2), ...]
        donde:
        - el primer elemento es absoluto (con offset X e Y aplicados),
        - los siguientes son movimientos relativos respecto al punto anterior.
        Todo en el sistema de coordenadas actual (origen abajo-derecha).
        """
        if self.centros_ordenados is None or len(self.centros_ordenados) == 0:
            return []

        if self.img_transf is None:
            return []

        h, w = self.img_transf.shape[:2]

        # 1) Convertir todos los centros a sistema "inferior-derecha" SIN offset
        '''
        puntos_base = []
        for (x_cv, y_cv) in self.centros_ordenados:
            x_br = int((w - 1) - float(x_cv))
            y_br = int((h - 1) - float(y_cv))
            puntos_base.append((x_br, y_br))
        '''
        puntos_base = []
        for (x_cv, y_cv) in self.centros_ordenados:
            puntos_base.append((x_cv, y_cv))

        movimientos = []
        for i, (x, y) in enumerate(puntos_base):
            if i == 0:
                # 2) Solo aquí aplicamos el offset en X e Y al punto absoluto
                x0 = x + int(self.offset_x_primer_hex)
                y0 = y + int(self.offset_y_primer_hex)
                movimientos.append((x0, y0))  # absoluto con offset
            else:
                # 3) Movimientos relativos calculados sobre los puntos SIN offset
                x_prev, y_prev = puntos_base[i - 1]
                dx = x - x_prev
                dy = y - y_prev
                movimientos.append((dx, dy))

        return movimientos




    def stop(self):
        self._running = False
        self.wait()

# ---------------- AIMEN Worker ----------------
class AimenWorker(QThread):
    datos_leidos = pyqtSignal(dict)

    def __init__(self, plc_siemens,plc_aimen, db_number=605):
        super().__init__()
        self.plc_siemens = plc_siemens
        self.db_number = db_number
        self._running = True
        self.simulacion = False  # Cambiar a False para datos reales
        self.plc_aimen = plc_aimen
        self.valores_leidos = False
        self.valores_escritos = False
        self.pulsador = False
        self.panel_salida = False

    def run(self):
        if self.plc_aimen == None:
            print("no aimen")
        else:
            print("si aimen")
        while self._running:
            # Lectura datos AIMEN
            valores = {}
            if self.simulacion:
                try:
                    import random
                    clin_altura_tramo_0 = random.randint(1, 999)
                    clin_altura_tramo_1 = random.randint(1, 999)
                    clin_altura_tramo_2 = random.randint(1, 999)
                    clin_altura_tramo_3 = random.randint(1, 999)
                    clin_ancho = random.randint(1, 999)
                    clin_largo = random.randint(1, 999)
                    clin_sector_0_x = random.randint(1, 999)
                    clin_sector_0_y = random.randint(1, 999)
                    clin_sector_1_x = random.randint(1, 999)
                    clin_sector_1_y = random.randint(1, 999)
                    valores["int0"] = clin_altura_tramo_0
                    valores["int1"] = clin_altura_tramo_1
                    valores["int2"] = clin_altura_tramo_2
                    valores["int3"] = clin_altura_tramo_3
                    valores["int4"] = clin_ancho
                    valores["int5"] = clin_largo
                    valores["int6"] = clin_sector_0_x
                    valores["int7"] = clin_sector_0_y
                    valores["int8"] = clin_sector_1_x
                    valores["int9"] = clin_sector_1_y
                    self.datos_leidos.emit(valores)    
                except Exception as e:
                    print(f"Error en simulación AIMEN: {e}")
                    time.sleep(0.1)
                    continue  
            else:
                try:
                    data_siemens = bytearray(32)
                    self.panel_salida = self.plc_aimen.getValue(AIMEN_PANEL_SALIDA)
                    set_bool(data_siemens, 0, 1, self.panel_salida)
                    if self.panel_salida and self.valores_leidos == False:
                        clin_ancho = self.plc_aimen.getValue(AIMEN_PANEL_ANCHO)
                        clin_largo = self.plc_aimen.getValue(AIMEN_PANEL_LARGO)
                        clin_altura_tramo_0 = self.plc_aimen.getValue(AIMEN_CHECK_ALTURA_MAX_TRAMO0)
                        clin_altura_tramo_1 = self.plc_aimen.getValue(AIMEN_CHECK_ALTURA_MAX_TRAMO1)
                        clin_altura_tramo_2 = self.plc_aimen.getValue(AIMEN_CHECK_ALTURA_MAX_TRAMO2)
                        clin_altura_tramo_3 = self.plc_aimen.getValue(AIMEN_CHECK_ALTURA_MAX_TRAMO3)
                        clin_sector_0_x = self.plc_aimen.getValue(AIMEN_SECTOR0_X)
                        clin_sector_0_y = self.plc_aimen.getValue(AIMEN_SECTOR0_Y)
                        clin_sector_1_x = self.plc_aimen.getValue(AIMEN_SECTOR1_X)
                        clin_sector_1_y = self.plc_aimen.getValue(AIMEN_SECTOR1_Y)
                        valores["int0"] = clin_altura_tramo_0
                        valores["int1"] = clin_altura_tramo_1
                        valores["int2"] = clin_altura_tramo_2
                        valores["int3"] = clin_altura_tramo_3
                        valores["int4"] = clin_ancho
                        valores["int5"] = clin_largo
                        valores["int6"] = clin_sector_0_x
                        valores["int7"] = clin_sector_0_y
                        valores["int8"] = clin_sector_1_x
                        valores["int9"] = clin_sector_1_y
                        self.valores_leidos = True
                        self.datos_leidos.emit(valores)


                except Exception as e:
                    print(f"Error leyendo AIMEN: {e}")
                    time.sleep(0.1)
                    continue 
            # Escritura datos en Siemens
            if self.valores_leidos == True:
                try:    
                    #if not self.plc_siemens.get_connected(): continue
                    data_pulsador = self.plc_siemens.read_area(Areas.DB,self.db_number,0,1)
                    clin_flanco_boton = get_bool(data_pulsador,0,0)
                    #self.panel_salida = get_bool(data_pulsador,0,1)
                    self.pulsador = get_bool(data_pulsador,0,2)
                    print("PULSADOR: ", self.pulsador)
                    # Mapear datos
                    set_bool(data_siemens, 0, 0, clin_flanco_boton)
                    set_bool(data_siemens, 0, 1, self.panel_salida)
                    #set_bool(data_siemens, 0, 2, self.pulsador)

                    data_siemens[8:10] = clin_altura_tramo_0.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[10:12] = clin_altura_tramo_1.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[12:14] = clin_altura_tramo_2.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[14:16] = clin_altura_tramo_3.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[16:18] = clin_ancho.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[18:20] = clin_largo.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[20:22] = clin_sector_0_x.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[22:24] = clin_sector_0_y.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[24:26] = clin_sector_1_x.to_bytes(2, byteorder='big', signed=True)
                    data_siemens[26:28] = clin_sector_1_y.to_bytes(2, byteorder='big', signed=True)
                except Exception as e:
                    print(f"Error escribiendo Siemens: {e}")
                    time.sleep(0.1)
                    continue    
            
                # Leemos datos de PLC Seiki
                #data = self.plc_siemens.db_read(self.db_number, 0, 200)
                #data[0] |= 0b00000001  # Bit0 del byte 2 a True

                #print("Data: ",data[0])

                #self.pulsador = plc.getValue(SEIKI_LANZAR_BASTIDOR)
                if self.pulsador == True:
                    # Escribimos datos en PLC Aimen
                    self.plc_aimen.setValueB(AIMEN_PULSADOR_SEIKI, self.pulsador)
                    self.valores_escritos = True
                    self.valores_leidos = False
                    '''
                    if (alto0 == 0 and alto1 == 0 and alto2 == 0 and alto3 == 0):
                        alto = 0
                    else:
                        alto = 1
                    '''
                else:
                    self.plc_aimen.setValueB(AIMEN_PULSADOR_SEIKI, self.pulsador)
            self.plc_siemens.db_write(self.db_number, 0, data_siemens)

            if self.pulsador == True:
                data_pulsador = self.plc_siemens.read_area(Areas.DB,self.db_number,0,1)
                self.pulsador = get_bool(data_pulsador,0,2)
                print("PULSADOR TRUE")
                if self.pulsador == False:
                    self.plc_aimen.setValueB(AIMEN_PULSADOR_SEIKI, self.pulsador)
                    print("PULSADOR FALSE")
            time.sleep(0.2)

    def mapear_datos(self, data: bytes):
        valores = {}
        
        return valores
    
    def stop(self):
        self._running=False
        self.wait()

# ---------------- Hilo para ping a cámara ----------------
class CameraPinger(QThread):
    ping_result = pyqtSignal(bool)  # True si responde, False si no

    def __init__(self, ip, intervalo=1, parent=None):
        super().__init__(parent)
        self.ip = ip
        self.intervalo = intervalo
        self._running = True
        self.camera_connected = False  

    def run(self):
        while self._running:
            estado = self.hacer_ping()
            # Emitir solo si hay cambio de estado
            if estado != self.camera_connected:
                self.camera_connected = estado
                self.ping_result.emit(self.camera_connected)
            time.sleep(self.intervalo)

    def hacer_ping(self):
        """Devuelve True si la cámara responde al ping, False si no"""
        param = "-n" if platform.system().lower() == "windows" else "-c"
        comando = ["ping", param, "1", self.ip]

        try:
            resultado = subprocess.run(
                comando,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return resultado.returncode == 0
        except Exception:
            return False

    def stop(self):
        self._running = False
        self.wait()



# ---------------- Ventana Principal ----------------
class VentanaPrincipal(QStackedWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interfaz PyQt5 - Clinchadora")
        self.pantalla_principal = PantallaPrincipal(self)
        self.pantalla_clinchadora = PantallaClinchadora(self)
        self.pantalla_camara = PantallaCamara(self)  
        self.pantalla_plc = PantallaPLC(self)
        
        self.addWidget(self.pantalla_principal)   # 0
        self.addWidget(self.pantalla_clinchadora) # 1
        self.addWidget(self.pantalla_camara)      # 2
        self.addWidget(self.pantalla_plc)         # 3

        #configuración inicial 
        self.config = cargar_config()

        # Hilo PLCWorker
        self.siemens_client = snap7.client.Client()
        self.ip_siemens = "192.168.1.3"
        self.rack_siemens = 0
        self.slot_siemens = 1
        self.db_number = 10100

        try:
            self.siemens_client.connect(self.ip_siemens, self.rack_siemens, self.slot_siemens)
            print("Conectado al PLC")
        except Exception as e:
            print(f"Error conectando PLC: {e}")
            return

        self.plc_thread = PLCWorker(client=self.siemens_client)
        self.plc_thread.imagen_procesada.connect(self.mostrar_imagen) # envia la imagen del hilo a la pantalla principal
        self.plc_thread.camara_listo.connect(self.pantalla_camara.set_camera)  # conecta señal
        self.plc_thread.fps_actualizado.connect(self.actualizar_fps)    # actualiza fps en pantalla principal
        self.plc_thread.coordenadas_actualizadas.connect(self.actualizar_coordenadas) # actualiza coordenadas de hexagono en pantalla principal
        self.plc_thread.envio_pts_select.connect(self.actualizar_puntos) # actualiza los puntos seleccionados para transformacion en pantalla camara

        self.pantalla_principal.signal_reset_db.connect(self.plc_thread.handle_reset_db)
        self.pantalla_camara.enviar_distancia.connect(self.plc_thread.set_valor)
        self.pantalla_camara.puntos_asignados.connect(self.plc_thread.set_puntos)
        self.pantalla_camara.solicitar_seleccion_puntos.connect(self.plc_thread.seleccionar_puntos_interactivo)
        self.pantalla_camara.enviar_offset_x.connect(self.plc_thread.set_offset_x)
        self.pantalla_camara.enviar_offset_y.connect(self.plc_thread.set_offset_y)
        self.pantalla_camara.enviar_factor_mm.connect(self.plc_thread.set_factor_mm)


        # Aplicar configuración inicial a UI y PLCWorker
        self.aplicar_config_inicial()

        # Iniciar el hilo PLCWorker
        self.plc_thread.start()

        

        

        # Hilo AIMENWorker
        self.siemens_client_clinchadora = snap7.client.Client()
        try:
            self.siemens_client_clinchadora.connect(self.ip_siemens, self.rack_siemens, self.slot_siemens)
            print("Conectado al PLC (clinchadora)")
        except Exception as e:
            print(f"Error conectando PLC clinchadora: {e}")
            return

        try:
            aimen_client = CommPLC_OPCUA("172.31.0.11", 4840)
            print("Conectado al PLC AIMEN")
        except Exception as e:
            print(f"ERROR CONEXION PLC AIMEN: {e}")        
        self.aimen_thread = AimenWorker(plc_siemens=self.siemens_client_clinchadora, plc_aimen=aimen_client)
        self.aimen_thread.datos_leidos.connect(self.actualizar_valores_aimen)
        self.aimen_thread.start()

        

        # Camera pinger -> actualiza el LED
        self.pinger = CameraPinger("192.168.3.10", intervalo=2)
        self.pinger.ping_result.connect(self.actualizar_led_camara)  
        self.pinger.ping_result.connect(self.plc_thread.set_camera_connected)
        self.pinger.start()

        self.setStyleSheet("""
            QWidget { background-color: #2E2E2E; color: #E0E0E0; font-family: 'Segoe UI'; font-size: 14px; }
            QGroupBox { background-color: #3C3F41; border: 2px solid #555555; border-radius: 10px; margin-top: 10px; font-weight: bold; padding: 10px; color: #FFFFFF; }
            QLabel { font-size: 13px; color: #E0E0E0; }
            QPushButton#roundButton { border-radius: 20px; background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #5A9BD5, stop:1 #2E75B6); color: white; font-weight: bold; padding: 8px 16px; }
            QPushButton#roundButton:hover { background-color: #1F4E79; }
            QPushButton#greenButton { background-color: #28A745; border-radius: 10px; color: white; font-weight: bold; padding: 10px 20px; }
            QPushButton#greenButton:hover { background-color: #1E7E34; }
        """)

    def closeEvent(self, event):
        try:
            if hasattr(self, 'plc_thread') and self.plc_thread.isRunning():
                self.plc_thread.stop()
            if hasattr(self, 'pinger'):
                self.pinger.stop()
        except Exception as e:
            print(f"Error al detener hilos: {e}")
        event.accept()

    def actualizar_puntos(self, puntos):
        # 'puntos' es una lista de 4 listas [[x1, y1], [x2, y2], ...]
        for i, punto in enumerate(puntos):
            x = punto[0]
            y = punto[1]

            lbl_x, lbl_y = self.pantalla_camara.puntos_inputs[i]
            lbl_x.setText(str(x))
            lbl_y.setText(str(y))

        # 💾 Guardar también en el JSON
        self.config.setdefault("calibracion", {})
        self.config["calibracion"]["puntos"] = puntos
        guardar_config(self.config)


    def actualizar_valores_aimen(self, valores):
        for i in range(10):
            valor = valores.get(f"int{i}", "---")
            self.pantalla_clinchadora.int_labels[i].setText(str(valor))

    def mostrar_imagen(self, img):
        if img is None:
            return
        try:
            rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Para poner numeros en los centros
            '''if self.plc_thread.results:
                centros = self.plc_thread.obtener_centros_hexagonos()
                if len(centros) > 0:
                    centros = self.plc_thread.ordenar_centros(centros)
                    for idx, (x, y) in enumerate(centros):
                        cv2.putText(
                            rgb_image, str(idx), (int(x), int(y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA
                        )'''

            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            pixmap = pixmap.scaled(
                self.pantalla_principal.imagen_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.pantalla_principal.imagen_label.setPixmap(pixmap)
        except Exception as e:
            print(f"Error mostrando imagen: {e}")

    def actualizar_led_camara(self, ok: bool):
        """Actualiza el LED con el resultado del ping"""
        if ok:
            self.pantalla_principal.led_camara.setStyleSheet(
                "background-color: #28A745; border-radius: 12px;"  # Verde
            )
        else:
            self.pantalla_principal.led_camara.setStyleSheet(
                "background-color: #DC3545; border-radius: 12px;"  # Rojo
            )

    def actualizar_fps(self, fps):
        self.pantalla_principal.lbl_fps.setText(f"FPS: {fps:.1f}")

    def actualizar_coordenadas(self, x, y):
        if x != -1000000 and y != -1000000:
            self.pantalla_principal.lbl_hex_x.setText(f"X: {x:.2f}")
            self.pantalla_principal.lbl_hex_y.setText(f"Y: {y:.2f}")
        else:
            self.pantalla_principal.lbl_hex_x.setText("X: ---")
            self.pantalla_principal.lbl_hex_y.setText("Y: ---")

    def aplicar_config_inicial(self):
        cfg = self.config or {}
        cam_cfg = cfg.get("camara", {})
        yolo_cfg = cfg.get("yolo", {})
        calib_cfg = cfg.get("calibracion", {})

        # --- Cámara: exposición y ganancia (solo UI de momento) ---
        exp = cam_cfg.get("exposicion")
        if exp is not None:
            self.pantalla_camara.expo_input.setText(str(exp))

        gain = cam_cfg.get("ganancia")
        if gain is not None:
            self.pantalla_camara.gain_input.setText(str(gain))

        # --- YOLO: distancia mínima hexágonos ---
        dist = yolo_cfg.get("distancia_minima_hexagonos")
        if dist is not None:
            self.pantalla_camara.dist_input.setText(str(dist))
            # también actualizar valor en el hilo
            try:
                self.plc_thread.set_valor(int(dist))
            except Exception as e:
                print(f"Error aplicando distancia mínima: {e}")
        
        # --- YOLO: offset X primer hexágono ---
        offset = yolo_cfg.get("offset_x_primer_hexagono")
        if offset is not None:
            self.pantalla_camara.offset_input.setText(str(offset))
            try:
                self.plc_thread.set_offset_x(int(offset))
            except Exception as e:
                print(f"Error aplicando offset X: {e}")

        offset_y = yolo_cfg.get("offset_y_primer_hexagono")
        if offset_y is not None:
            self.pantalla_camara.offset_y_input.setText(str(offset_y))
            try:
                self.plc_thread.set_offset_y(int(offset_y))
            except Exception as e:
                print(f"Error aplicando offset Y: {e}")

        # --- YOLO: factor mm por pixel ---
        factor_mm = yolo_cfg.get("factor_mm_por_pixel")
        if factor_mm is not None:
            self.pantalla_camara.factor_input.setText(str(factor_mm))
            try:
                self.plc_thread.set_factor_mm(float(factor_mm))
            except Exception as e:
                print(f"Error aplicando factor mm/px: {e}")

        # --- Calibración: puntos de perspectiva ---
        pts = calib_cfg.get("puntos")
        if pts and len(pts) == 4:
            # actualizar labels de la pantalla cámara
            for i, punto in enumerate(pts):
                if i >= len(self.pantalla_camara.puntos_inputs):
                    break
                x, y = punto
                lbl_x, lbl_y = self.pantalla_camara.puntos_inputs[i]
                lbl_x.setText(str(x))
                lbl_y.setText(str(y))
            # actualizar también en el hilo PLCWorker
            try:
                self.plc_thread.set_puntos(pts)
            except Exception as e:
                print(f"Error aplicando puntos de calibración: {e}")


# ---------------- Ejecutar ----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ventana = VentanaPrincipal()
    ventana.resize(950, 650)
    ventana.show()
    sys.exit(app.exec_())


