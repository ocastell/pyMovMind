#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#  Add emokit library from https://github.com/CerebralPower/emokit
#
#
# Control de versió: modmind.py versió 1.0 data 18-9-2014
#
import emotiv
#
# Add traditional librarys
#
import sys
import os
import gevent
import platform
import time
import numpy as np
import pickle
import pygame
from pygame import FULLSCREEN
from subprocess import check_output
#
# Add OpenCV Library
#
import cv2
#
# Add PyQt Library
#
from PyQt4 import Qt
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *
#
sys.path.append(os.getcwd())
#
# Add personal Librarys
#
from pyeeg import *
from pysvm import *
from filterpy.kalman import KalmanFilter
#
# Internacionalització de l'aplicació amb gettext
#
import locale
import gettext
#from gettext import gettext as _ 
#
# Global variables
#
quality_color = {
"0": (0, 0, 0),
"1": (255, 0, 0),
"2": (255, 0, 0),
"3": (255, 255, 0),
"4": (255, 255, 0),
"5": (0, 255, 0),
"6": (0, 255, 0),
"7": (0, 255, 0),
}
old_quality_color = {
"0": (0, 0, 0),
"1": (255, 0, 0),
"2": (255, 255, 0),
"3": (0, 255, 0),
"4": (0, 255, 0),
}
#
# Class emotiv (Principal class)
# Emotiv EPOC uses sequential sampling method, single ADC, at a rate of 128 SPS (Samples Per Second, 2048 Hz internal).
# Emotiv EPOC operates at a resolution of 14 bits (16 bit ADC, 2 bits instrumental noise floor discarded), with bandwidth being 0.2 - 45 Hz.
# 2 seconds => 128 sps x 2 secons = 256 samples 
# 4 seconds => 128 sps x 4 secons = 512 samples 
# 8 seconds => 128 sps x 8 secons = 1024 samples 
#
class Emotiv(QtGui.QWidget):
      def __init__ (self,language):
          super(Emotiv, self).__init__()
          self.y_print=False           # True Si volem imprimir informació per terminal
          self.ysimular = False        # True Si volem simular la senyal amb dades des d'un fitxer
          self.yguardar=False          # True Si volem guardar la senyal des del casc
          self.yconectar=False         # True Si volem obtenir dades de la senyal des del casc
          self.yparar = False          # True Si volem parar l'acció
          self.ydades = False          # True Si volem llegir la senyal de disc
          self.old_model=False         # True si el model del casc és anterior a Juny de 2014
          self.nsamples=256            # Número de mostres a pendre per defecte (2 segons)
          self.nom_fitxer=None         # Nom del fitxer per llegir i escriure
          self.ruta = "./dades"        # Ruta per defecte
          self.calibracio=3000.        # Calibració per veure l'escala de la senyal amb més detall
          self.max_paquets=8           # Número de pacquets per guardar en buffer (accelera el programa)
          self.screen_x_max=900        # Dimensió pantalla pygame X
          self.screen_y_max=500        # Dimensió pantalla pygame Y
          self.subscreen_x_max=250     # Dimensió subpantalla pygame X
          self.subscreen_y_max=100     # Dimensió subpantalla pygame Y
          self.screen_x_min=700        # Dimensió mínima per defecta pantalla del UI X
          self.screen_y_min=550        # Dimensió mínima per defecta pantalla del UI Y
          self.calibra_gyro_x=0        # Valor de calibració del gyroscopi eix X
          self.calibra_gyro_y=0        # Valor de calibració del gyroscopi eix Y
          self.ycalibrar=False         # True si s'ha de calibrar el gyroscopi
          self.ycalibrat=False         # True si s'ha calibrat el gyroscopi
          self.svm_load=False          # True si s'ha llegit fitxer SVM
          self.ann_load=False          # True si s'ha llegit fitxer ANN
          self.test_conectar=False     # True si s'ha conectat el casc per fer el test en temps real
          self.yserial_print=True      # True imprimeix el número de sèrie de l'aparell
          self.ykalman_filter=False    # True aplica un filtre Kalman a les mesures dels sensors
#
# Inicialització de dades
#
          self.headset = None
          self.senyal=[]
          self.qualitat=[]
          self.carac=[]
          self.text=["0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"]
          self.nom_sensors=["gyroX","gyroY","AF3","F7","F3","FC5","T7","P7","O1","O2","P8","T8","FC6","F4","F8","AF4"]
          self.Sensors=["AF3","F7","F3","FC5","T7","P7","O1","O2","P8","T8","FC6","F4","F8","AF4"]
          self.nom_carac=["HFD","SE","SVD","Fisher","DFA"]
          #self.nom_carac=["PSI","PFD","HFD","Hjörth","SE","SVD","Fisher","ApEN","DFA","Hurst"]
#
# Internacionalització: llenguatge per defecte es_CA (Català)
#
          if language == None:
             info_locale=locale.getdefaultlocale()
             self.language=str(info_locale[0])
          else:
             self.language=language
          if self.language == 'es_ES':
             lang0 = gettext.translation('movmind', 'idiomes', languages=['es_ES'], fallback=True)
          elif self.language == 'en_EN':
             lang0 = gettext.translation('movmind', 'idiomes', languages=['en_EN'], fallback=True)
          elif self.language == 'fr_FR':
             lang0 = gettext.translation('movmind', 'idiomes', languages=['fr_EN'], fallback=True)
          elif self.language == 'du_DU':
             lang0 = gettext.translation('movmind', 'idiomes', languages=['du_DU'], fallback=True)
          elif self.language == 'ca_ES':
             lang0 = gettext.translation('movmind', 'idiomes', languages=['ca_ES'], fallback=True)
          else:
             lang0 = gettext.translation('movmind', 'idiomes', languages=['ca_ES'], fallback=True)
          lang0.install(unicode=False)
#
# Plataformes identificades amb platform: aix3 aix4 atheos beos5 darwin freebsd2 freebsd3 freebsd4 freebsd5 freebsd6 freebsd7 
#                                         generic irix5 irix6 linux2 mac netbsd1 next3 os2emx riscos sunos5 unixware7
#
          _system=platform.system()
          _platform=platform.platform()
          _processador=platform.processor()
          _uname=platform.uname()
          if _system == "Linux":
              self.sistema="Linux"
          elif _platform == "Darwin":
              self.sistema="MacOSX"
          elif _platform == "Windows":
              self.sistema="Windows" 
          if self.sistema == "Windows": 
             os.system('cls')
          elif self.sistema == "Linux":
             os.system('clear')
          elif self.sistema == "MacOSX":
             os.system('clear')
          print "================================================================================================================================="
          print _("Sistema: "),_system
          print _("Plataforma: "),_platform
          print _("Processador: "),_processador
          print _("Informació uname: "),_uname
          print _("Llenguatge per defecte: "),self.language
          print "================================================================================================================================="
          QApplication.setStyle("windows")
          self.setStyleSheet("""QToolTip {
                                     background-color: black;
                                     color: white;
                                     border: black solid 1px
                                     }""")
          QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
#
# Geometria propietats de la finestra principal
#
          text=_('Aplicació Casc Emotiv Epoc').decode("utf-8")
          self.setToolTip(text)
          self.setGeometry(0,0, self.screen_x_min,self.screen_y_min)
          text=_('Emotiv App').decode("utf-8")
          self.setWindowTitle(text)
          self.setWindowIcon(QtGui.QIcon('icon.png'))
          self.resize(self.screen_x_min,self.screen_y_min)
          self.setMinimumSize(self.screen_x_min,self.screen_y_min)
          self.center()
          self.initUI()
      def center(self):
          screen = QtGui.QDesktopWidget().screenGeometry()
          size = self.geometry()
          self.move((screen.width()-size.width())/2, (screen.height()-size.height())/2)
      def initUI(self):
#
# Definim la paleta de colors per tot el programa
#
          self.palette=QtGui.QPalette()
#
# Definim les zones de treball
#
          tab_widget = QtGui.QTabWidget()
          self.tab1 = QtGui.QWidget()
          self.tab2 = QtGui.QWidget()
          self.tab3 = QtGui.QWidget()
          self.tab4 = QtGui.QWidget()
          self.p1_vertical = QtGui.QVBoxLayout(self.tab1)
          self.p2_vertical = QtGui.QVBoxLayout(self.tab2)
          self.p3_vertical = QtGui.QVBoxLayout(self.tab3)
          self.p4_vertical = QtGui.QVBoxLayout(self.tab4)
        
          text=_('Adquisició de la senyal').decode("utf-8")
          tab_widget.addTab(self.tab1, text)
          text=_('Caracterització').decode("utf-8")
          tab_widget.addTab(self.tab2, text)
          text=_('Entrenament').decode("utf-8")
          tab_widget.addTab(self.tab3, text)
          text=_('Tests').decode("utf-8")
          tab_widget.addTab(self.tab4, text)
          self.vbox = QtGui.QVBoxLayout()
          self.vbox.addWidget(tab_widget)
#
# Grid per p1 UI defined here
#
          self.p1_vertical.addWidget(self.controls_p1())
          self.p1_vertical.addWidget(self.logbox_p1())
          self.setLayout(self.vbox)
#
# Grid per p2 UI defined here
#
          self.p2_vertical.addWidget(self.controls_p2())
          self.p2_vertical.addWidget(self.dades_p2())
          self.setLayout(self.vbox)
#
# Grid per p3 UI defined here
#
          self.p3_vertical.addWidget(self.svm_p3())
          self.p3_vertical.addWidget(self.ann_p3())
#
# Grid per p4 UI defined here
#
          self.p4_vertical.addWidget(self.svm_p4())
          self.p4_vertical.addWidget(self.ann_p4())
          self.setLayout(self.vbox)
          self.show()
#
# Funció controls_p1()
#
      def controls_p1(self):
          text=_('Controls').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
          self.label1 = QtGui.QLabel()
          text=_('Casc: ').decode("utf-8")
          self.label1.setText(text)
          self.label2 = QtGui.QLabel()
          text=_('No connectat').decode("utf-8")
          self.label2.setText(text)
          self.label2.setStyleSheet('color: red')
          self.label3 = QtGui.QLabel()
          text=_('Bateria: ').decode("utf-8")
          self.label3.setText(text)
          self.labelbateria = QtGui.QLabel()
          self.labelbateria.setText('0'+'%')
          self.label1.resize(self.label1.sizeHint())
          self.label2.resize(self.label2.sizeHint())
          self.label3.resize(self.label3.sizeHint())
          self.labelbateria.resize(self.labelbateria.sizeHint())
#
# List Languages (per canviar l'idioma on-the-fly)
#
          self.label_idioma = QtGui.QLabel()
          text=_('Canvi idioma: ').decode("utf-8")
          self.label_idioma.setText(text)
          self.label_idioma.resize(self.label_idioma.sizeHint())
          self.combo_idioma = QtGui.QComboBox()
          self.combo_idioma.setEditable(False)
          idioma=[_('Català'),_('Castellà'),_('Anglès'),_('Francès'),_('Alemany')]
          for item in idioma:
              item=str(item).decode("utf-8")
              self.combo_idioma.addItem(item)
          self.combo_idioma.resize(self.combo_idioma.sizeHint())
          self.connect(self.combo_idioma, SIGNAL('activated(QString)'), self.change_combo_idioma)
#
# CheckBox Filtre Kalman
#   
          self.label_kalman = QtGui.QLabel()
          text=_('Filtre Kalman').decode("utf-8")
          self.label_kalman.setText(text)
          self.label_kalman.resize(self.label_kalman.sizeHint())
          self.kalman = QtGui.QCheckBox('kalman',self)
          self.kalman.setToolTip('Filtre Kalman')
          self.kalman.resize(self.kalman.sizeHint())
          self.kalman.setChecked(False)
#
# List calibracio (per canviar el valor de calibració)
#
          self.label_calibracio = QtGui.QLabel()
          text=_('Calibració: ').decode("utf-8")
          self.label_calibracio.setText(text)
          self.label_calibracio.resize(self.label_calibracio.sizeHint())
          self.combo_calibracio = QtGui.QComboBox()
          self.combo_calibracio.setEditable(False)
          cali=['8192','7192','6192','5192','4192','3192','2192','1192','500','300','200','100']
          for item in cali:
              self.combo_calibracio.addItem(item)
          self.combo_calibracio.resize(self.combo_calibracio.sizeHint())
          self.connect(self.combo_calibracio, SIGNAL('activated(QString)'), self.change_combo_calibracio)
#
# Boto calibrar
#
          text = _('Calibrar')
          self.calibrar = QtGui.QPushButton(text)
          text=_('Calibrat inicial del casc').decode("utf-8")
          self.calibrar.setToolTip(text)
          self.calibrar.resize(self.calibrar.sizeHint())
          self.connect(self.calibrar, SIGNAL("clicked()"), self.clic_calibrar_p1)
#
# Boto connectar
#
          text = _('Connectar')
          self.conect = QtGui.QPushButton(text)
          text=_('Connectar amb el casc').decode("utf-8")
          self.conect.setToolTip(text)
          self.conect.resize(self.conect.sizeHint())
          self.connect(self.conect, SIGNAL("clicked()"), self.clic_conect_p1)
#
# boto sortir
#
          text = _('Sortir')
          self.sortir = QtGui.QPushButton(text)
          text=_('Tancar el programa').decode("utf-8")
          self.sortir.setToolTip(text)
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# boto llegir
#
	  text = _('Simular')
	  self.simular = QtGui.QPushButton(text)
          text=_('Llegir dades del disc').decode("utf-8")
          self.simular.setToolTip(text)
          self.simular.resize(self.simular.sizeHint())
          self.connect(self.simular, SIGNAL("clicked()"), self.clic_simular_p1)
#
# boto guardar
#
	  text=_('Desar').decode("utf-8")
	  self.guardar= QtGui.QPushButton(text)
          text=_('Desar dades al disc').decode("utf-8")
          self.guardar.setToolTip(text)
          self.guardar.resize(self.guardar.sizeHint())
          self.guardar.setEnabled(True)
          self.connect(self.guardar, SIGNAL("clicked()"), self.clic_guardar_p1)
#
# Boto parar
#
          text=_('Parar').decode("utf-8")
          self.parar= QtGui.QPushButton(text)
          text=_('Parar la connexió o la simulacio').decode("utf-8")
          self.parar.setToolTip(text)
          self.parar.resize(self.parar.sizeHint())
          self.parar.setEnabled(False)
          self.connect(self.parar, SIGNAL("clicked()"), self.clic_parar_p1)
#
# Grid Layout
#
          gridlayout = QtGui.QGridLayout()
          impar = [1,3,5]
          par = [0,1]
          for i in impar:
              for j in par:
                 self.label_blanc = QtGui.QLabel(' ')
                 gridlayout.addWidget(self.label_blanc,j,i)
                 gridlayout.addWidget(self.label_blanc,j,i+1)
          gridlayout.addWidget(self.label1,0,0)
          gridlayout.addWidget(self.label2,0,1)
          #gridlayout.addWidget(self.label3,0,2)
          #gridlayout.addWidget(self.labelbateria,0,3)
          gridlayout.addWidget(self.label_calibracio,0,3)
          gridlayout.addWidget(self.combo_calibracio,0,4)
          gridlayout.addWidget(self.label_idioma,0,5)
          gridlayout.addWidget(self.combo_idioma,0,6)
          gridlayout.addWidget(self.conect,1,0)
          gridlayout.addWidget(self.simular,1,1)
          gridlayout.addWidget(self.guardar,1,2)
          gridlayout.addWidget(self.parar,1,3)
          gridlayout.addWidget(self.calibrar,1,4)
          gridlayout.addWidget(self.sortir,1,6)
          gridlayout.addWidget(self.label_kalman,2,0)
          gridlayout.addWidget(self.kalman,2,1)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(gridlayout)
          return groupBox
#
# Funció logbox_p1()
#
      def logbox_p1(self):
          text=_('log').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
          vboxlayout = QtGui.QVBoxLayout()
          self.progressBox_p1 = QtGui.QPlainTextEdit()
          self.progressBox_p1.setReadOnly(True)
          self.progressBox_p1.moveCursor(QTextCursor.End)
          vboxlayout = QtGui.QVBoxLayout()
          vboxlayout.addWidget(self.progressBox_p1)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(vboxlayout)
          return groupBox
#
# Funció svm_p4()
#
      def svm_p4(self):
          groupBox = QtGui.QGroupBox('Test SVM (Support Vector Machine)')
#
# Boto sortir
#
          text = _('Sortir')
          self.sortir = QtGui.QPushButton(text)
          text=_('Tancar el programa').decode("utf-8")
          self.sortir.setToolTip(text)
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# Boto llegir fitxer SVM
#
          text = _('Carregar SVM')
          self.llegir_svm_p4 = QtGui.QPushButton(text)
          text=_('Llegir dades del SVM des del disc').decode("utf-8")
          self.llegir_svm_p4.setToolTip(text)
          self.llegir_svm_p4.resize(self.llegir_svm_p4.sizeHint())
          self.connect(self.llegir_svm_p4, SIGNAL("clicked()"), self.clic_llegir_svm_p4)
#
# Boto llegir fitxer test
#
          text=_('Carregar test per SVM').decode("utf-8")
          self.llegir_test_svm_p4 = QtGui.QPushButton(text)
          text=_('Llegir fitxer senyal per test del SVM').decode("utf-8")
          self.llegir_test_svm_p4.setToolTip(text)
          self.llegir_test_svm_p4.resize(self.llegir_test_svm_p4.sizeHint())
          self.connect(self.llegir_test_svm_p4, SIGNAL("clicked()"), self.clic_llegir_test_svm_p4)
          self.llegir_test_svm_p4.setEnabled(False)
#
# Boto connect test (connecta el casc per fer el test en temps real)
#
          text = _('Connectar')
          self.conecta_test_svm_p4 = QtGui.QPushButton(text)
          text=_('Connectar el casc per fer test del SVM').decode("utf-8")
          self.conecta_test_svm_p4.setToolTip(text)
          self.conecta_test_svm_p4.resize(self.conecta_test_svm_p4.sizeHint())
          self.connect(self.conecta_test_svm_p4, SIGNAL("clicked()"), self.clic_conecta_test_svm_p4)
          self.conecta_test_svm_p4.setEnabled(False)
#
# VBox & HBox Layouts
#
          vboxlayout = QtGui.QVBoxLayout()
          self.progressBox_svm_test = QtGui.QPlainTextEdit()
          self.progressBox_svm_test.setReadOnly(True)
          self.progressBox_svm_test.moveCursor(QTextCursor.End)
          vboxlayout.addWidget(self.progressBox_svm_test)
          hboxlayout = QtGui.QHBoxLayout()
          vboxlayout.addLayout(hboxlayout)
          hboxlayout.addStretch(2)
          hboxlayout.addWidget(self.conecta_test_svm_p4)
          hboxlayout.addWidget(self.llegir_svm_p4)
          hboxlayout.addWidget(self.llegir_test_svm_p4)
          hboxlayout.addWidget(self.sortir)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(vboxlayout)
          return groupBox
#
# Funció ann_p4()
#
      def ann_p4(self):
          text=_('Test ANN (Artificial Neural Network)').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
#
# Boto sortir
#
          text = _('Sortir')
          self.sortir = QtGui.QPushButton(text)
          text=_('Tancar el programa').decode("utf-8")
          self.sortir.setToolTip(text)
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# Boto llegir fitxer ANN
#
          text = _('Carregar ANN')
          self.llegir_ann_p4 = QtGui.QPushButton(text)
          text=_('Llegir dades del ANN des del disc').decode("utf-8")
          self.llegir_ann_p4.setToolTip(text)
          self.llegir_ann_p4.resize(self.llegir_ann_p4.sizeHint())
          self.connect(self.llegir_ann_p4, SIGNAL("clicked()"), self.clic_llegir_ann_p4)
#
# Boto llegir fitxer test
#
          text=_('Carregar test per ANN').decode("utf-8")
          self.llegir_test_ann_p4 = QtGui.QPushButton(text)
          text=_('Llegir fitxer senyal per test del ANN').decode("utf-8")
          self.llegir_test_ann_p4.setToolTip(text)
          self.llegir_test_ann_p4.resize(self.llegir_test_ann_p4.sizeHint())
          self.connect(self.llegir_test_ann_p4, SIGNAL("clicked()"), self.clic_llegir_test_ann_p4)
          self.llegir_test_ann_p4.setEnabled(False)
#
# Boto connect test (connecta el casc per fer el test en temps real)
#
          text = _('Connectar')
          self.conecta_test_ann_p4 = QtGui.QPushButton(text)
          text=_('Connectar el casc per fer test del ANN').decode("utf-8")
          self.conecta_test_ann_p4.setToolTip(text)
          self.conecta_test_ann_p4.resize(self.conecta_test_ann_p4.sizeHint())
          self.connect(self.conecta_test_ann_p4, SIGNAL("clicked()"), self.clic_conecta_test_ann_p4)
          self.conecta_test_ann_p4.setEnabled(False)
#
# VBox & HBox Layouts
#
          vboxlayout = QtGui.QVBoxLayout()
          self.progressBox_ann_test = QtGui.QPlainTextEdit()
          self.progressBox_ann_test.setReadOnly(True)
          self.progressBox_ann_test.moveCursor(QTextCursor.End)
          vboxlayout.addWidget(self.progressBox_ann_test)
          hboxlayout = QtGui.QHBoxLayout()
          vboxlayout.addLayout(hboxlayout)
          hboxlayout.addStretch(2)
          hboxlayout.addWidget(self.conecta_test_ann_p4)
          hboxlayout.addWidget(self.llegir_ann_p4)
          hboxlayout.addWidget(self.llegir_test_ann_p4)
          hboxlayout.addWidget(self.sortir)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(vboxlayout)
          return groupBox
#
# Funció svm_p3()
#
      def svm_p3(self):
          text=_('SVM (Support Vector Machine)').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
#
# Boto sortir
#
          text=_('Sortir').decode("utf-8")
          self.sortir = QtGui.QPushButton(text)
          text=_('Tancar el programa').decode("utf-8")
          self.sortir.setToolTip(text)
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# Boto llegir
#
          text=_('Carregar Dades').decode("utf-8")
          self.llegir_svm_p3 = QtGui.QPushButton(text)
          text=_('Llegir dades del disc').decode("utf-8")
          self.llegir_svm_p3.setToolTip(text)
          self.llegir_svm_p3.resize(self.llegir_svm_p3.sizeHint())
          self.connect(self.llegir_svm_p3, SIGNAL("clicked()"), self.clic_llegir_svm_p3)
#
# VBox & HBox Layouts
#
          vboxlayout = QtGui.QVBoxLayout()
          self.progressBox_svm = QtGui.QPlainTextEdit()
          self.progressBox_svm.setReadOnly(True)
          self.progressBox_svm.moveCursor(QTextCursor.End)
          vboxlayout.addWidget(self.progressBox_svm)
          hboxlayout = QtGui.QHBoxLayout()
          vboxlayout.addLayout(hboxlayout)
          hboxlayout.addStretch(2)
          hboxlayout.addWidget(self.llegir_svm_p3)
          hboxlayout.addWidget(self.sortir)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(vboxlayout)
          return groupBox
#
# Funció ann_p3()
#
      def ann_p3(self):
          text = _('ANN (Artificial Neural Networks)')
          groupBox = QtGui.QGroupBox(text)
#
# Boto sortir
#
          text=_('Sortir').decode("utf-8")
          self.sortir = QtGui.QPushButton(text)
          text=_('Tancar el programa').decode("utf-8")
          self.sortir.setToolTip(text)
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# Boto llegir
#
          text=_('Carregar Dades').decode("utf-8")
          self.llegir_ann_p3 = QtGui.QPushButton(text)
          text=_('Llegir dades del disc').decode("utf-8")
          self.llegir_ann_p3.setToolTip(text)
          self.llegir_ann_p3.resize(self.llegir_ann_p3.sizeHint())
          self.connect(self.llegir_ann_p3, SIGNAL("clicked()"), self.clic_llegir_ann_p3)
#
# VBox & HBox Layouts
#
          vboxlayout = QtGui.QVBoxLayout()
          self.progressBox_ann = QtGui.QPlainTextEdit()
          self.progressBox_ann.setReadOnly(True)
          self.progressBox_ann.moveCursor(QTextCursor.End)
          vboxlayout.addWidget(self.progressBox_ann)
          hboxlayout = QtGui.QHBoxLayout()
          vboxlayout.addLayout(hboxlayout)
          hboxlayout.addStretch(2)
          hboxlayout.addWidget(self.llegir_ann_p3)
          hboxlayout.addWidget(self.sortir)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(vboxlayout)
          return groupBox
#
# Funció controls_p2()
#
      def controls_p2(self):
          text=_('Controls').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
#  1.- PSI (Power Spectrum Intensity) i RIR (Relative Intensity Ratio)
#  2.- PFD (Petrosian Factral Dimension)
#  3.- HFD (Higuchi Fractal Dimension)
#  4.- Hjörth movility and complexity
#  5.- Spectral Entropy (Shanon's entropy of RIRs)
#  6.- SVD Entropy
#  7.- Fisher Information
#  8.- Aproximate Entropy (ApEN)
#  9.- DFA (Detrended Fluctuation Analisis) ----- No es calcula
#  10.- Hurst Exponent ----- No es calcula
#
# CheckBox PSI
#   
          self.psi = QtGui.QCheckBox('PSI',self)
          self.psi.setToolTip('Power Spectrum Intensity')
          self.psi.resize(self.psi.sizeHint())
          self.psi.setChecked(False)
#
# CheckBox PFD
#   
          self.pfd = QtGui.QCheckBox('PFD',self)
          self.pfd.setToolTip('Petrosian Fractal Dimension')
          self.pfd.resize(self.psi.sizeHint())
          self.pfd.setChecked(False)
#
# CheckBox HFD
#   
          self.hfd = QtGui.QCheckBox('HFD',self)
          self.hfd.setToolTip('Higuchi Fractal Dimension')
          self.hfd.resize(self.hfd.sizeHint())
          self.hfd.setChecked(True)
#
# CheckBox Hjörth Movility & Complexity
#   
          text = 'Hjörth'
          text = text.decode("utf-8")
          self.hjorth = QtGui.QCheckBox(text,self)
          self.hjorth.setToolTip('Movility & complexity')
          self.hjorth.resize(self.hjorth.sizeHint())
          self.hjorth.setChecked(False)
#
# CheckBox Espectral Entropy
#   
          self.se = QtGui.QCheckBox('Spectral Entropy',self)
          self.se.setToolTip("Shanon's entropy of RIRs")
          self.se.resize(self.se.sizeHint())
          self.se.setChecked(True)
#
# CheckBox SVD Entropy
#   
          self.svd = QtGui.QCheckBox('SVD Entropy',self)
          self.svd.setToolTip("SVD Entropy")
          self.svd.resize(self.svd.sizeHint())
          self.svd.setChecked(True)
#
# CheckBox Fisher Information
#   
          self.fis = QtGui.QCheckBox('Fisher Inf.',self)
          self.fis.setToolTip("Fisher Information")
          self.fis.resize(self.fis.sizeHint())
          self.fis.setChecked(True)
#
# CheckBox Aproximate Entropy
#   
          self.apn = QtGui.QCheckBox('ApEn',self)
          self.apn.setToolTip("Aproximate Entropy")
          self.apn.resize(self.apn.sizeHint())
          self.apn.setChecked(False)
#
# CheckBox DFA
#   
          self.dfa = QtGui.QCheckBox('DFA',self)
          self.dfa.setToolTip("Detrended Fluctuation Analisis")
          self.dfa.resize(self.dfa.sizeHint())
          self.dfa.setChecked(True)
#
# CheckBox Hurst
#   
          self.hur = QtGui.QCheckBox('Hurst',self)
          self.hur.setToolTip("Hurst Exponent")
          self.hur.resize(self.hur.sizeHint())
          self.hur.setChecked(False)
#
# Boto sortir
#
          self.sortir = QtGui.QPushButton('Sortir')
          self.sortir.setToolTip('Tancar el programa')
          self.sortir.resize(self.sortir.sizeHint())
          self.connect(self.sortir, SIGNAL("clicked()"), self.clic_sortir)
#
# Boto llegir
#
          text=_('Carregar Dades').decode("utf-8")
          self.llegir_p2 = QtGui.QPushButton(text)
          text=_('Llegir dades del disc').decode("utf-8")
          self.llegir_p2.setToolTip(text)
          self.llegir_p2.resize(self.llegir_p2.sizeHint())
          self.connect(self.llegir_p2, SIGNAL("clicked()"), self.clic_llegir_p2)
#
# Grid Layout
#
          gridlayout = QtGui.QGridLayout()
          impar = [1,2,3]
          par = [1,2,3]
          for i in impar:
              for j in par:
                 self.label_blanc = QtGui.QLabel(' ')
                 gridlayout.addWidget(self.label_blanc,j,i)
                 gridlayout.addWidget(self.label_blanc,j,i+1)
          gridlayout.addWidget(self.psi,1,1)
          gridlayout.addWidget(self.pfd,1,2)
          gridlayout.addWidget(self.hfd,1,3)
          gridlayout.addWidget(self.hjorth,1,4)
          gridlayout.addWidget(self.se,2,1)
          gridlayout.addWidget(self.svd,2,2)
          gridlayout.addWidget(self.fis,2,3)
          gridlayout.addWidget(self.apn,2,4)
          gridlayout.addWidget(self.dfa,3,1)
          gridlayout.addWidget(self.hur,3,2)
          gridlayout.addWidget(self.llegir_p2,4,3)
          gridlayout.addWidget(self.sortir,4,4)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(gridlayout)
          return groupBox
#
# Funció dades_p2()
#
      def dades_p2(self):
          text=_('Resultats').decode("utf-8")
          groupBox = QtGui.QGroupBox(text)
          gridlayout = QtGui.QGridLayout()
          zero=0.0
          ii=0
          for i in range(15):
              if ii < 14:
                 self.labels_sensors = QLabel()
                 self.labels_sensors.setText(self.Sensors[i])
                 gridlayout.addWidget(self.labels_sensors,i+1,0)
                 self.labels_sensors.resize(self.labels_sensors.sizeHint())
              ii=ii+1
              for j in range(5):
                  self.labels_resultats = QLabel()
                  if i==0 :
                     self.labels_resultats.setObjectName(self.nom_carac[j])
                     self.labels_resultats.setText(self.nom_carac[j])
                     self.labels_resultats.setAlignment(QtCore.Qt.AlignCenter)
                     gridlayout.addWidget(self.labels_resultats,0,j+1)
                     self.labels_resultats.resize(self.labels_resultats.sizeHint())
                  else:
                     self.labels_resultats.setObjectName(self.nom_carac[j]+"sensor"+str(i))
                     self.labels_resultats.setText(str(zero))
                     self.labels_resultats.setAlignment(QtCore.Qt.AlignCenter)
                     gridlayout.addWidget(self.labels_resultats,i,j+1)
          groupBox.resize(groupBox.sizeHint())
          groupBox.setLayout(gridlayout)
          return groupBox
#
# Change select calibracio
#
      def change_combo_calibracio(self,cali):
          self.calibracio=float(str(cali))
#
# Change select idioma
#
      def change_combo_idioma(self,llengua):
#
# Internacionalització: llenguatge per defecte es_CA (Català)
#
          self.language=str(llengua.toUtf8()).decode("utf-8")
          if self.language == 'Castellà'.decode("utf-8"):
             lang0 = gettext.translation('movmind', 'idiomes', languages=['es_ES'], fallback=True)
          elif self.language == 'Anglés'.decode("utf-8"):
             lang0 = gettext.translation('movmind', 'idiomes', languages=['en_EN'], fallback=True)
          elif self.language == 'Francés'.decode("utf-8"):
             lang0 = gettext.translation('movmind', 'idiomes', languages=['fr_EN'], fallback=True)
          elif self.language == 'Alemany'.decode("utf-8"):
             lang0 = gettext.translation('movmind', 'idiomes', languages=['de_DE'], fallback=True)
          elif self.language == 'Català'.decode("utf-8"):
             lang0 = gettext.translation('movmind', 'idiomes', languages=['ca_ES'], fallback=True)
          else:
             lang0 = gettext.translation('movmind', 'idiomes', languages=['ca_ES'], fallback=True)
          lang0.install()
          print "Language", self.language
          self.update()
          return
#
# Clic boto conect
#
      def clic_calibrar_p1(self):
          self.yparar = False
          self.ycasc=False
          self.ycasc=self.buscaCasc()
          if self.ycasc:
             self.headset = emotiv.Emotiv(display_output=False, is_research=False)
             gevent.spawn(self.headset.setup)
             gevent.sleep(1)
             text=_('Connectat').decode("utf-8")
             self.label2.setText(text)
             self.label2.setStyleSheet('color: green')
             self.label2.resize(self.label2.sizeHint())
             self.simular.setEnabled(False)
             self.conect.setEnabled(False)
             self.parar.setEnabled(True)
             self.guardar.setEnabled(False)
             self.ysimular = False
#           
# Cridem funció per opcions i explicació de calibrat en una nova finestra           
#           
             self.ycalibrat=False
             self.ycalibrar=True
             self.clic_guardar_p1()
             self.state=True
             self.render()
          else:
             info_message= _("No es pot connectar amb el Casc")
             info=_("Informació")
             QtGui.QMessageBox.information(self, info, info_message)
#
# Funció per introduir opcions del casc al calibrat i explicació de com fer-ho
#
      def opcions(self):
          items=['2 segons','4 segons','6 segons','8 segons']
          text_info = _('Opcions de temps de gravació  de senyal')
          if self.ycalibrar:
             text_user = _('Recorda deixar el casc en absolut repos. Temps de gravació: ')
          else:
             text_user = _('Temps de gravació: ')
          text_info = text_info.decode("utf-8")
          text_user = text_user.decode("utf-8")
          text, ok = QtGui.QInputDialog.getItem(self, text_info,text_user,items,0,False)
          if ok:
             if str(text) == items[0]: 
                self.nsamples=256     # Número de mostres a pendre per defecte (2 segons)
             elif str(text) == items[1]:
                self.nsamples=512     # Número de mostres a pendre (4 segons)
             elif str(text) == items[2]:
                self.nsamples=768     # Número de mostres a pendre (6 segons)
             elif str(text) == items[3]:
                self.nsamples=1024    # Número de mostres a pendre (8 segons )
#
# Clic boto conect
#
      def clic_conect_p1(self):
          self.yparar = False
          self.ycasc=False
          self.ycasc=self.buscaCasc()
          if self.ycasc:
             self.headset = emotiv.Emotiv(display_output=False, is_research=False)
             gevent.spawn(self.headset.setup)
             gevent.sleep(1)
             text=_('Connectat').decode("utf-8")
             self.label2.setText(text)
             self.label2.setStyleSheet('color: green')
             self.label2.resize(self.label2.sizeHint())
             self.simular.setEnabled(False)
             self.conect.setEnabled(False)
             self.parar.setEnabled(True)
             self.guardar.setEnabled(False)
             self.ysimular = False
             self.ycalibrar = False
             self.test_conectar = False
             self.svm_load = False
             self.ann_load = False
             self.yconectar = True
             self.state=True
             self.render()
          else:
             info_message= _("No es pot connectar amb el Casc")
             info=_("Informació").decode("utf-8")
             QtGui.QMessageBox.information(self, info, info_message)
#
# Clic boto parar
#
      def clic_parar_p1(self):
          self.yparar = True
          text=_('No connectat').decode("utf-8")
          self.label2.setText(text)
          self.label2.setStyleSheet('color: red')
          self.label2.resize(self.label2.sizeHint())
          self.conect.setEnabled(True)
          self.parar.setEnabled(False)
          self.simular.setEnabled(True)
          self.guardar.setEnabled(True)
          self.yguardar=False
          self.ysimular=False
          self.yconectar=False
#
# Clic boto sortir
#
      def clic_sortir(self):
          if self.ysimular or self.yconectar:
             pygame.display.quit()
             if self.headset :
                self.headset.close()
          QApplication.quit()
#
# Clic boto guardar
#
      def clic_guardar_p1(self):
          self.guardar.setEnabled(False)
          self.simular.setEnabled(False)
          self.nom_fitxer=None
#           
# Cridem funció per opcions i explicació de calibrat en una nova finestra           
#           
          self.opcions()
	  text=_("Desar fitxer de dades").decode("utf-8")
          selectedFilter='*.dat'
	  self.nom_fitxer = QFileDialog.getSaveFileName(self, text, self.ruta, selectedFilter)
          if self.nom_fitxer :
             search = '.'
             string = str(self.nom_fitxer)
             index = string.find(search)
             if index == -1:
                name = string + '.dat' 
             else:   
                name = string[0:index] + '.dat' 
             self.nom_fitxer = name
             self.yguardar=True
	     self.ysimular= False
          else: 
             self.clic_guardar_p1()
#
# Clic boto llegir p1
#
      def clic_simular_p1(self):
          self.conect.setEnabled(False)
          self.parar.setEnabled(True)
          self.simular.setEnabled(False)
          self.guardar.setEnabled(False)
          self.nom_fitxer=None
	  text=_("Carregar fitxer").decode("utf-8")
          selectedFilter='*.dat'
	  self.nom_fitxer = QFileDialog.getOpenFileName(self, text, self.ruta, selectedFilter)
          if self.nom_fitxer :
             text=_('Simulació').decode("utf-8")
             self.label2.setText(text)
             self.label2.setStyleSheet('color: blue')
             self.ysimular= True
             self.yconectar= False
             self.yparar = False
             self.render()
#
# Clic boto llegir test_ann_p4
#
      def clic_llegir_test_ann_p4(self):
          self.nom_fitxers=None
	  text=_("Carrega fitxer(s) de senyal").decode("utf-8")
          selectedFilter='*.dat'
	  self.nom_fitxers = QFileDialog.getOpenFileNames(self, text, self.ruta, selectedFilter)
          if self.nom_fitxers:
             self.calcula_test_ann()
#
# Clic boto llegir test_svm_p4
#
      def clic_llegir_test_svm_p4(self):
          self.nom_fitxers=None
	  text=_("Carrega fitxer(s) de senyal").decode("utf-8")
          selectedFilter='*.dat'
	  self.nom_fitxers = QFileDialog.getOpenFileNames(self, text, self.ruta, selectedFilter)
          if self.nom_fitxers:
             self.calcula_test_svm()
#
# Clic boto conecta test_svm_p4
#
      def clic_conecta_test_svm_p4(self):
          self.llegir_test_svm_p4.setEnabled(False)
          self.conecta_test_svm_p4.setEnabled(False)
          self.yparar = False
          self.ycasc=False
          self.ycasc=self.buscaCasc()
          if self.ycasc:
             self.headset = emotiv.Emotiv(display_output=False, is_research=False)
             gevent.spawn(self.headset.setup)
             gevent.sleep(1)
             self.ysimular = False
             self.state=True
             self.yguardar=False
             self.test_conectar=True
             self.render()
          else:
             info_message= _("No es pot connectar amb el Casc").decode("utf-8")
             info=_("Informació").decode("utf-8")
             QtGui.QMessageBox.information(self, info, info_message)
             self.llegir_test_svm_p4.setEnabled(True)
             self.conecta_test_svm_p4.setEnabled(True)
#
# Clic boto conecta test_ann_p4
#
      def clic_conecta_test_ann_p4(self):
          self.llegir_test_ann_p4.setEnabled(False)
          self.conecta_test_ann_p4.setEnabled(False)
          self.yparar = False
          self.ycasc=False
          self.ycasc=self.buscaCasc()
          if self.ycasc:
             self.headset = emotiv.Emotiv(display_output=False, is_research=False)
             gevent.spawn(self.headset.setup)
             gevent.sleep(1)
             self.ysimular = False
             self.state=True
             self.yguardar=False
             self.test_conectar=True
             self.render()
          else:
             info_message= _("No es pot connectar amb el Casc").decode("utf-8")
             info=_("Informació").decode("utf-8")
             QtGui.QMessageBox.information(self, info, info_message)
             self.llegir_test_ann_p4.setEnabled(True)
             self.conecta_test_ann_p4.setEnabled(True)
#
# Clic boto llegir ann_p4
#
      def clic_llegir_ann_p4(self):
          self.nom_fitxer=None
	  text=_("Carrega fitxer ANN").decode("utf-8")
          selectedFilter='*.ann'
	  self.nom_fitxer = QFileDialog.getOpenFileName(self, text, self.ruta, selectedFilter)
          if self.nom_fitxer:
             self.nom_fitxer_ann=str(self.nom_fitxer)
             text=_('Fitxer del ANN: ')+self.nom_fitxer_ann+'\n'
             text=text.decode("utf-8")
             self.progressBox_ann_test.insertPlainText(text)
             self.llegir_test_ann_p4.setEnabled(True)
             self.conecta_test_ann_p4.setEnabled(True)
             self.ann_load=True
#
# Clic boto llegir svm_p4
#
      def clic_llegir_svm_p4(self):
          self.nom_fitxer=None
	  text=_("Carrega fitxer SVM").decode("utf-8")
          selectedFilter='*.svm'
	  self.nom_fitxer = QFileDialog.getOpenFileName(self, text, self.ruta, selectedFilter)
          if self.nom_fitxer:
             self.nom_fitxer_svm=str(self.nom_fitxer)
             text=_('Fitxer del SVM: ')+self.nom_fitxer_svm+'\n'
             text=text.decode("utf-8")
             self.progressBox_svm_test.insertPlainText(text)
             self.llegir_test_svm_p4.setEnabled(True)
             self.conecta_test_svm_p4.setEnabled(True)
             self.svm_load=True
#
# Clic boto llegir svm_p3
#
      def clic_llegir_svm_p3(self):
          self.nom_fitxers=None
	  text=_("Carrega fitxers de característiques").decode("utf-8")
          selectedFilter='*.data'
	  self.nom_fitxers = QFileDialog.getOpenFileNames(self, text, self.ruta, selectedFilter)
          if self.nom_fitxers:
             self.calcula_svm()
#
# Clic boto llegir ann_p3
#
      def clic_llegir_ann_p3(self):
          self.nom_fitxers=None
	  text=_("Carrega fitxers de característiques").decode("utf-8")
          selectedFilter='*.data'
	  self.nom_fitxers = QFileDialog.getOpenFileNames(self, text, self.ruta, selectedFilter)
          if self.nom_fitxers:
             self.calcula_ann()
#
# Clic boto llegir p2
#
      def clic_llegir_p2(self):
          self.nom_fitxers=None
	  text=_("Carrega fitxers de senyals EEG").decode("utf-8")
          selectedFilter='*.dat'
	  self.nom_fitxers = QFileDialog.getOpenFileNames(self, text, self.ruta, selectedFilter)
          if self.nom_fitxers :
             self.caracteristiques()
             self.resultats_p2()
#
# Funcio calcula_test_ann() per realitzar el test sobre la senyal a partir d'un fitxer ANN
#
      def calcula_test_ann(self):
#
# Lectura del fitxer del ANN
#
          nnet=cv2.ANN_MLP()
          nnet.load(str(self.nom_fitxer_ann))
          text=_('Fitxer del ANN llegit amb èxit!!').decode("utf-8")+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann_test.insertPlainText(text)
#
# Lectura del(s) fitxer(s) de senyal des del disk
#
          numero_fitxers=len(self.nom_fitxers)
          result1 = result2 = result3 = result4 = result5 = result6 = result7 = result8 = result9 = result10 = 0
          maxfile=20
          maxcara=20
          self.A=np.zeros((maxfile,14,maxcara))
          self.nfitxer=0
#
# Lectura dels fitxers de senyal
#
          for items in self.nom_fitxers:
              self.carac_dat=[]
              self.DATAT=[]
              self.arxiu=open(items,"r")
              self.senyal=[]
              sequencia=[]
              self.ylectura=True
              text=_('lectura de les senyals del fitxer: ')+str(items)+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
#
# Bucle de lectura sobre el fitxer items de la llista de fitxers
#
              self.iregistres=0
              while self.ylectura:
                 try:
                    sequencia=pickle.load(self.arxiu)
                    self.lectura = sequencia[0]
                    self.gyroX = sequencia[1]
                    self.gyroY = sequencia[2]
                    self.senyal = sequencia[3]
                    self.qualitat = sequencia[4]
                    self.DATAT.append(self.senyal)
                    self.iregistres=self.iregistres+1
                 except EOFError:
                    self.ylectura=False
              text=_('lectura finalitzada amb éxit!!')+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
#
# Calculem les característiques de les senyals amb pyeeg
#
              text=_('calcula les característiques de la senyal escollida ... ')+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
              self.calcula_caracteristiques()   
              text=_('càlcul realitzat amb èxit!!! ')+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
#
# Desar resultats en format Numpy float32 per la classificació
#
              self.carac_dat=np.array(self.carac_dat, dtype=np.float32)
#
# Prediccio del ANN
#
              samples=np.array(self.carac_dat,dtype=np.float32)
              predictions=nnet.predict(samples)
              predic=predictions[1][0]
              control=np.ones((len(predic)),dtype=np.int)
              diference=np.rint(np.round(predic))-control
              classe_predita=np.argmax(diference)+1
              text=_('Vector de la predicció per aquesta senyal ----> ')
              text=text+str(predic)+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
              text=_('Classe predita pel ANN per aquesta senyal ----> ')
              text=text+str(classe_predita)+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann_test.insertPlainText(text)
              self.arxiu.close()
#
# Funcio calcula_test_svm() per realitzar el test sobre la senyal a partir d'un fitxer SVM
#
      def calcula_test_svm(self):
#
# Lectura del fitxer del SVM
#
          clf = SVM()
          clf.load(str(self.nom_fitxer_svm))
          text=_('Fitxer del SVM llegit amb èxit!! ')
          text=text+'\n'
          text=text.decode("utf-8")
          self.progressBox_svm_test.insertPlainText(text)
#
# Lectura del(s) fitxer(s) de senyal des del disk
#
          numero_fitxers=len(self.nom_fitxers)
          result1 = result2 = result3 = result4 = result5 = result6 = result7 = result8 = result9 = result10 = 0
          maxfile=20
          maxcara=20
          self.A=np.zeros((maxfile,14,maxcara))
          self.nfitxer=0
#
# Lectura dels fitxers de senyal
#
          for items in self.nom_fitxers:
              self.carac_dat=[]
              self.DATAT=[]
              self.arxiu=open(items,"r")
              self.senyal=[]
              sequencia=[]
              self.ylectura=True
              text=_('lectura de les senyals del fitxer: ')
              text=text+str(items)+'\n'
              text=text.decode("utf-8")
              self.progressBox_svm_test.insertPlainText(text)
#
# Bucle de lectura sobre el fitxer items de la llista de fitxers
#
              self.iregistres=0
              while self.ylectura:
                 try:
                    sequencia=pickle.load(self.arxiu)
                    self.lectura = sequencia[0]
                    self.gyroX = sequencia[1]
                    self.gyroY = sequencia[2]
                    self.senyal = sequencia[3]
                    self.qualitat = sequencia[4]
                    self.DATAT.append(self.senyal)
                    self.iregistres=self.iregistres+1
                 except EOFError:
                    self.ylectura=False
              text=_('lectura finalitzada amb éxit!!')
              text=text+'\n'
              text=text.decode("utf-8")
              self.progressBox_svm_test.insertPlainText(text)
#
# Calculem les característiques de les senyals amb pyeeg
#
              text=_('calcula les característiques de la senyal escollida ... ')
              text=text+'\n'
              text=text.decode("utf-8")
              self.progressBox_svm_test.insertPlainText(text)
              self.calcula_caracteristiques()   
              text=_('càlcul realitzat amb èxit!!! ')
              text=text+'\n'
              text=text.decode("utf-8")
              self.progressBox_svm_test.insertPlainText(text)
#
# Desar resultats en format Numpy float32 per la classificació
#
              self.carac_dat=np.array(self.carac_dat, dtype=np.float32)
#
# Prediccio del SVM
#
              samples=np.array(self.carac_dat,dtype=np.float32)
              classe_predita = clf.predict(samples)
              text=_('Classe predita pel SVM per aquest vector ----> ')
              text=text+str(classe_predita)+'\n'
              text=text.decode("utf-8")
              self.progressBox_svm_test.insertPlainText(text)
              self.arxiu.close()
          return
#
# Funcio calcula_ann() per calcular la xarxa neuronal (NN) a partir de les característiques de les senyals
#
      def calcula_ann(self):
          samples = []
          targets = []
          ref=1.15
#
# 1 Classe per fitxer
#
          fitxers=self.nom_fitxers
          classe=0
          nnvalors=[]
          for fitxer in fitxers:
              classe=classe+1
              arxiu=open(fitxer,"r")
              if classe == 1:
                 samples=np.load(str(fitxer))
                 nvalors=len(samples)
                 nnvalors.append(nvalors)
                 text=_('Fitxer per la classe: ')
                 text=text+str(classe)+' '+str(fitxer)+'\n'
                 text=text.decode("utf-8")
                 self.progressBox_ann.insertPlainText(text)
              else:
                 valors0=np.load(str(fitxer))
                 nvalors=len(valors0)
                 samples=np.concatenate((samples,valors0),axis=0)
                 nnvalors.append(nvalors)
                 text=_('Fitxer per la classe: ')
                 text=text+str(classe)+' '+str(fitxer)+'\n'
                 text=text.decode("utf-8")
                 self.progressBox_ann.insertPlainText(text)
          targets = np.zeros( (len(samples), len(fitxers)), 'float' )
          class_targets = []
          classe=0
          k=0
          for max in nnvalors:
              for j in range(max):
                  targets[k][classe]=1.
                  class_targets.append(classe)
                  k=k+1
              classe=classe+1
          ninput_layer=len(samples[0])
          noutput_layer=len(targets[0])
          nhidden_layer=int(ref*ninput_layer)
#
# Create the neural network
#
          layers = np.array([ninput_layer, nhidden_layer, noutput_layer])
          text0='============================\n'
          text=_("S'ha creat la xarxa neuronal")
          text=text0+text+'\n'+text0
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          nnet = cv2.ANN_MLP(layers)
          text=_('Número de neurones entrada:')
          text=text+str(ninput_layer)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          text=_('Número de neurones ocultes:')
          text=text+str(nhidden_layer)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          text=_('Número de neurones sortida:')
          text=text+str(noutput_layer)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# Some parameters for learning.  Step size is the gradient step size
# for backpropogation.
#
          step_size = 0.01
          text=_('Mida del increment del gradient per backpropogation per iteració: ')
          text=text+str(step_size)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# Momentum can be ignored for this example.
#
          momentum = 0.0
          text=_('Moment: ')
          text=text+str(momentum)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# Max steps of training
#
          nsteps = 10000
          text=_("Número màxim d'iteracions: ")
          text=text+str(nsteps)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# Error threshold for halting training
#
          max_err = 0.000001
          text=_("Llindar màxim del error per parar l'entrenament: ")
          text=text+str(max_err)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# When to stop: whichever comes first, count or error
#
          condition = cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS
#
# Tuple of termination criteria: first condition, then # steps, then
# error tolerance second and third things are ignored if not implied
# by condition
#
          criteria = (condition, nsteps, max_err)
#
# params is a dictionary with relevant things for NNet training.
#
          params = dict( term_crit = criteria,
                         train_method = cv2.ANN_MLP_TRAIN_PARAMS_BACKPROP,
                         bp_dw_scale = step_size,
                         bp_moment_scale = momentum )
#
# Train our network
#
          samples=np.array(samples,dtype=np.float32)
          targets=np.array(targets,dtype=np.float32)
          num_iter = nnet.train(samples, targets, None, params=params)
          text0="============================\n"
          text=_("S'ha finalitzat l'entrenament")
          text=text0+text+'\n'+text0
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
#
# Es desa la xarxa neuronal al disc en un fitxer
#
          self.nom_fitxer=None
          while self.nom_fitxer == None:
	        text=_("Desar fitxer de ANN (Artificial Neural Network)")
                selectedFilter='*.ann'
	        self.nom_fitxer = QFileDialog.getSaveFileName(self, text, self.ruta, selectedFilter)
          search = '.'
          string = str(self.nom_fitxer)
          index = string.find(search)
          if index == -1:
             name = string + '.ann' 
          else:   
             name = string[0:index] + '.ann' 
          self.nom_fitxer = name
          nnet.save(str(self.nom_fitxer))
          text=_('Fitxer de resultats del ANN: ')
          text=text+str(self.nom_fitxer)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          nnet.load(str(self.nom_fitxer))
#
# Create a matrix of predictions (Test samples, targets)
#
          predictions = np.empty_like(targets)
#
# See how the network did.
#
          nnet.predict(samples, predictions)
#
# Compute sum of squared errors
#
          sse = np.sum( (targets - predictions)**2 )/len(class_targets)
#
# Compute correct
#
          ipredictions=np.rint(predictions).astype(int)
          itargets=np.rint(targets).astype(int)
          control=np.subtract(itargets,ipredictions)
          zeros=np.zeros(len(fitxers), dtype=np.int)
          num_correct=np.zeros(len(fitxers), dtype=np.int)
          i=0
          for row in control:
              if (row == zeros).all():
                 clase=class_targets[i]
                 num_correct[clase]=num_correct[clase]+1
              i=i+1
          text=_('Iteracions fins la convergència: ')
          text=text+str(num_iter)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          text=_('Error (suma quadrats de la diferència): ')
          text=text+str(sse)+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          for i in range(len(fitxers)):
              encerts_classe=num_correct[i]
              vectors_classe=nnvalors[i]
              text=_("Número de vectors de prova de la clase) {} = {}. Encerts al test {}. Exactitud de la classe {:.0%}.")
              text=text.format(str(i+1),str(nnvalors[i]),str(num_correct[i]),float(encerts_classe) / float(vectors_classe))
              text=text+'\n'
              text=text.decode("utf-8")
              self.progressBox_ann.insertPlainText(text)
          encerts_totals=np.sum(num_correct)
          vectors_totals=len(class_targets)
          text=_('Exactitud Total: {:.0%}')
          text=text.format(float(encerts_totals) / float(vectors_totals))
          text=text+'\n'
          text=text.decode("utf-8")
          self.progressBox_ann.insertPlainText(text)
          return
#
# Funcio calcula_svm() per calcular el SVM a partir de les característiques de les senyals
#
      def calcula_svm(self):
          classe=0
          y_train=[]
          self.vectors_classe=[]
          class_targets=[]
          nnvalors=[]
          for items in self.nom_fitxers:
              classe=classe+1
              fitxer=items
              arxiu=open(fitxer,"r")
              if classe == 1:
                 valors=np.load(arxiu)
                 nvalors=len(valors)
                 nnvalors.append(nvalors)
                 text=_('Fitxer per la classe: ')
                 text=text+str(classe)+' '+str(fitxer)
                 text=text.decode("utf-8")
                 self.progressBox_svm.insertPlainText(text)
              else:
                 valors0=np.load(arxiu)
                 nvalors=len(valors0)
                 nnvalors.append(nvalors)
                 valors=np.concatenate((valors,valors0),axis=0)
                 text=_('Fitxer per la classe: ')
                 text='\n'+text+str(classe)+' '+str(fitxer)
                 text=text.decode("utf-8")
                 self.progressBox_svm.insertPlainText(text)
              self.vectors_classe.append(nvalors)
              for j in range(nvalors):
                  y_train.append(float(classe))
                  class_targets.append(classe) 
              arxiu.close()
#
# Output cap al progressBox del SVM
#
          text=_('Número de classes: ')
          text='\n'+text+str(len(self.nom_fitxers))
          text=text.decode("utf-8")
          self.progressBox_svm.insertPlainText(text)
          llista = ''.join('['+str(int(e))+'] ' for e in self.vectors_classe)
          text='\n'
          text=_('Número de vectors de prova per classe: ')
          text='\n'+text+llista
          text=text.decode("utf-8")
          self.progressBox_svm.insertPlainText(text)
          samples=np.array(valors,dtype=np.float32)
          y_train=np.array(y_train,dtype=np.float32)
          if len(self.nom_fitxers) > 1:
             clf = SVM()
             clf.train(samples, y_train)
             text='\n'
             text=_("Entrenament l'espai SVM .... Èxit!!! ")
             text='\n'+text
             text=text.decode("utf-8")
             self.progressBox_svm.insertPlainText(text)
             self.nom_fitxer=None
             while self.nom_fitxer == None:
	           text=_("Desar fitxer de SVM (Support Vector Machine)")
                   text=text.decode("utf-8")
                   selectedFilter='*.svm'
	           self.nom_fitxer = QFileDialog.getSaveFileName(self, text, self.ruta, selectedFilter)
             search = '.'
             string = str(self.nom_fitxer)
             index = string.find(search)
             if index == -1:
                name = string + '.svm' 
             else:   
                name = string[0:index] + '.svm' 
             self.nom_fitxer = name
             clf.save(str(self.nom_fitxer))
             text=_('Fitxer de resultats del SVM: ')
             text='\n'+text+str(self.nom_fitxer)
             text=text.decode("utf-8")
             self.progressBox_svm.insertPlainText(text)
             clf.load(str(self.nom_fitxer))
             y_val = clf.predict(samples)
#
# Compute sum of squared errors
#
             sse = np.sum( (y_train - y_val)**2 )/len(class_targets)
             text=_('SSE Error del SVM: ')
             text='\n'+text+str(sse)
             text=text.decode("utf-8")
             self.progressBox_svm.insertPlainText(text)
#
# Compute correct
#
             ipredictions=np.rint(y_val).astype(int)
             itargets=np.rint(y_train).astype(int)
             control=np.subtract(itargets,ipredictions)
             zeros=np.zeros(len(self.nom_fitxers), dtype=np.int)
             num_correct=np.zeros(len(self.nom_fitxers), dtype=np.int)
             i=0
             for row in control:
                 if (row == zeros).all():
                    clase=class_targets[i]
                    num_correct[clase-1]=num_correct[clase-1]+1
                 i=i+1
             for i in range(len(self.nom_fitxers)):
                 encerts_classe=num_correct[i]
                 vectors_classe=nnvalors[i]
                 text=_("Número de vectors de prova de la clase {} = {}. Encerts al test {}. Exactitud de la classe {:.0%}.")
                 classe_error=text.format(str(i+1),str(nnvalors[i]),str(num_correct[i]),float(encerts_classe) / float(vectors_classe))
                 text='\n'+classe_error
                 text=text.decode("utf-8")
                 self.progressBox_svm.insertPlainText(text)
             encerts_totals=np.sum(num_correct)
             vectors_totals=len(class_targets)
             text=_('Exactitud Total: {:.0%}')
             total_error=text.format(float(encerts_totals) / float(vectors_totals))
             text='\n'+total_error
             text=text.decode("utf-8")
             self.progressBox_svm.insertPlainText(text)
             return
          else:
             print _("No es pot fer una classificació amb una sola classe ...").decode("utf-8")
             return
#
# Calcul de les característiques amb ajuda del PyEEG.py
#
      def calcula_caracteristiques(self):
#
# Comencem la caracterització: (feature extraction) utilitzem PyEEG.py
#
#  1.- PSI (Power Spectrum Intensity) i RIR (Relative Intensity Ratio)
#  2.- PFD (Petrosian Factral Dimension)
#  3.- HFD (Higuchi Fractal Dimension)
#  4.- Hjörth movility and complexity
#  5.- Spectral Entropy (Shanon's entropy of RIRs)
#  6.- SVD Entropy
#  7.- Fisher Information
#  8.- Aproximate Entropy (ApEN)
#  9.- DFA (Detrended Fluctuation Analisis) ----- No es calcula
#  10.- Hurst Exponent ----- No es calcula
#
              DIM = 10
              TAU = 4
              self.carac=[]
              for l in range(14):
                  self.ncarac=0
                  self.DATA=[]
                  for ll in range(self.iregistres-1):
                      self.DATA.append(self.DATAT[ll][l])
#
# PSI and RIR
#
                  if self.psi.isChecked():
                     Fs = 128
                     Band = [0.2,4,8,13,30,45]
                     result1 = bin_power(self.DATA, Band, Fs)
                     for k in result1[1]:
                         self.carac.append(float(k))
                         self.ncarac=self.ncarac+1
#
# PFD
#
                  if self.pfd.isChecked():
                     result2 = pfd(self.DATA)
                     self.carac.append(float(result2))
                     self.ncarac=self.ncarac+1
#
# HFD
#
                  if self.hfd.isChecked():
                     Kmax = 5
                     result3 = hfd(self.DATA, Kmax)
                     self.carac.append(float(result3))
                     self.ncarac=self.ncarac+1
#
# Hjörth
#
                  if self.hjorth.isChecked():
                     result4 = hjorth(self.DATA)
                     self.carac.append(float(result4[0]))
                     self.carac.append(float(result4[1]))
                     self.ncarac=self.ncarac+1
                     self.ncarac=self.ncarac+1
#
# Spectral Entropy
#
                  if self.se.isChecked():
                     Fs = 128
                     Band = [0.2,4,8,13,30,45]
                     result5 = spectral_entropy(self.DATA, Band, Fs)
                     self.carac.append(float(result5))
                     self.ncarac=self.ncarac+1
#
# SVD Entropy
#
                  if self.svd.isChecked():
                     M = embed_seq(self.DATA, TAU, DIM)
                     W = svd(M, compute_uv=0)
                     W /= sum(W)
                     result6 = svd_entropy(self.DATA, TAU, DIM, W)
                     self.carac.append(float(result6))
                     self.ncarac=self.ncarac+1
#
# Fisher Information
#
                  if self.fis.isChecked():
                     result7 = fisher_info(self.DATA, TAU, DIM, W)
                     self.carac.append(float(result7))
                     self.ncarac=self.ncarac+1
#
# Aproximate Entropy
#
                  if self.apn.isChecked():
                     R = std(self.DATA) * 0.3
                     result8 = ap_entropy(self.DATA, DIM, R)
                     self.carac.append(float(result8))
                     self.ncarac=self.ncarac+1
#
# DFA
#
                  if self.dfa.isChecked():
                     L = [16,32,64,128]
                     Ave = np.mean(self.DATA)
                     result9 = dfa(self.DATA,Ave,L)
                     self.carac.append(float(result9))
                     self.ncarac=self.ncarac+1
#
# Hurst
#
                  if self.hur.isChecked():
                     result10 = hurst(self.DATA)
                     if not isnan(result10):
                        result10 = float(result10)
                     else:
                        result10=0.5
                     self.carac.append(float(result10))
                     self.ncarac=self.ncarac+1
                  if not self.test_conectar:
                     for k in range(self.ncarac):
                         self.A[self.nfitxer][l][k]=self.carac[k+l*self.ncarac]
              if not self.test_conectar:
                 self.nfitxer=self.nfitxer+1
              self.carac_dat.append(self.carac)
              return
#
# Funcio caracteristiques per extreure les caracteristiques de una senyal:
# segons la bibliografia [] els més usats són PSD (Power Spectral Density), Paràmetres de Hjörth i AAR (Model adaptatiu autoregresiu m=>2)
#
      def caracteristiques(self):
#
# Bucle sobre tots els fitxers escollits
#
          numero_fitxers=len(self.nom_fitxers)
          self.fitxers_resultats=[]
          result1 = result2 = result3 = result4 = result5 = result6 = result7 = result8 = result9 = result10 = 0
          maxfile=20
          maxcara=20
          self.A=np.zeros((maxfile,14,maxcara))
          self.nfitxer=0
          self.carac_dat=[]
          for items in self.nom_fitxers:
              self.DATAT=[]
              self.fitxer=items
              self.arxiu=open(self.fitxer,"r")
              self.senyal=[]
              sequencia=[]
              self.ylectura=True
#
# Bucle de lectura sobre el fitxer items de la llista de fitxers
#
              self.iregistres=0
              while self.ylectura:
                 try:
                    sequencia=pickle.load(self.arxiu)
                    self.lectura = sequencia[0]
                    self.gyroX = sequencia[1]
                    self.gyroY = sequencia[2]
                    self.senyal = sequencia[3]
                    self.qualitat = sequencia[4]
                    self.DATAT.append(self.senyal)
                    self.iregistres=self.iregistres+1
                 except EOFError:
                    self.ylectura=False
#
# Calculem les característiques de les senyals amb pyeeg
#
              self.calcula_caracteristiques()   
#
#
# Cal fer el càlcul de la mitjana i de la desviació standard del total de les mostres per cada característica extreta
#
# Bucle sobre tots els fitxers de resultats per fer la mitja i desviació standard de cada característica extreta
#
          self.results=np.zeros((14,self.ncarac))
          self.desviacio_results=np.zeros((14,self.ncarac))
          for j in range(self.ncarac):
              for i in range(14):
                  valors=[]
                  for k in range(self.nfitxer):
                      valors.append(self.A[k][i][j])
                  mitja=np.mean(valors)
                  desviacio=np.std(valors)
                  self.results[i][j]=mitja
                  self.desviacio_results[i][j]=desviacio
#
# Desar resultats en format Numpy float32 per la classificació
#
          self.carac_dat=np.array(self.carac_dat, dtype=np.float32)
          self.nom_fitxer=None
          while self.nom_fitxer == None:
                selectedFilter='*.data'
	        self.nom_fitxer = QFileDialog.getSaveFileName(self, u"Desar fitxer de caracteístiques", self.ruta, selectedFilter)
          search = '.'
          string = str(self.nom_fitxer)
          index = string.find(search)
          if index == -1:
             name = string + '.data' 
          else:   
             name = string[0:index] + '.data' 
          self.nom_fitxer = name
          self.arxiu=open(self.nom_fitxer,"w")
          np.save(self.arxiu,self.carac_dat)    
          self.arxiu.close()
#
# Funció resultats_p2() mostra els resultats de les característiques
#
      def resultats_p2(self):
          for i in range(15):
              for j in range(5):
                  if i != 0:
                     self.label0 = self.findChild(QtGui.QLabel, self.nom_carac[j]+"sensor"+str(i))
                     if self.label0:
                        self.label0.setText('{:+f}'.format(self.results[i-1][j]))
#
# Funcio render
#
      def render(self):
#
# Dades render: modificacions per llegir des del disc dur simulació (Aquesta funció representa la senyal des d'un fitxer)
#
          graphers = []
          self.packet=None
          updated = False
          width = self.subscreen_x_max/2-10
          height = self.subscreen_y_max/2-10
          cursor_x, cursor_y = self.screen_x_max-self.subscreen_x_max/2, self.subscreen_y_max/2+10
#
# Definim el filtre kalman ---> 14 objectes de filtre kalman 1xcada sensor
#
          if self.kalman.isChecked():
             self.ykalman_filter = True
          else:
             self.ykalman_filter = False
          if self.ykalman_filter:
             filter = []
             for item in range(14):
                 f = KalmanFilter (dim_x=2, dim_z=1)
                 filter.append(f)
             for item in range(14):
                 filter[item].x = np.array([[2.],
                                            [0.]])       # initial state (location and velocity)
                 filter[item].F = np.array([[1.,1.],
                                            [0.,1.]])    # state transition matrix
                 filter[item].H = np.array([[1.,0.]])    # Measurement function
                 filter[item].P *= 1000.                 # covariance matrix
                 filter[item].R = 5                      # state uncertainty
                 filter[item].Q = 0.0001                 # process uncertainty
#
# Si fem el test en temps real ... amb SVM o ANN
#
          if self.test_conectar and (self.ann_load or self.svm_load):
             self.ycalibrar = False
             self.ycalibrat = True
             self.ysimular = False
             self.yguardar = False
             self.yconectar = True
#
# Si fem la calibració desem els valors del casc al disc
#
          if self.ycalibrar:
             self.ysimular = False
             self.yguardar = True
             self.yconectar = True
          if self.yconectar:
             self.ysimular=False
#
# Obrim el fitxer per llegir dades
#
          if self.ysimular and not self.yguardar and not self.yconectar:
             if os.path.exists(self.nom_fitxer):
                arxiu=open(self.nom_fitxer,"r")
#
# Obrim fitxers per escriure dades
#
          if self.yguardar and self.yconectar and not self.ysimular:
             arxiu2=open(self.nom_fitxer,"w")
#
# Init pygame
#
          pygame.init()
          screen = pygame.display.set_mode((self.screen_x_max, self.screen_y_max))
          text=_('Senyals EEG. Calibració actual: ')
          text=text+str(self.calibracio)
          text=text
          pygame.display.set_caption(text)
          for name in self.Sensors:
              graphers.append(Grapher(screen, name, len(graphers),self.calibracio))
          fullscreen = False
          font = pygame.font.Font(None, 20)
#
# Inicia el bucle del render pygame
#
          lec=0
          self.state=True
          gyroX_sum=0.
          gyroY_sum=0.
          self.DATAT=[]
          while self.state:
              for event in pygame.event.get():
                  if event.type == pygame.QUIT:
                      self.state=False
                      self.yparar=True
                      self.yguardar=False
                      self.ysimular=False
                      self.yconectar=False
                      if self.yguardar:
                         arxiu2.close()
                      pygame.display.quit()
                      if self.test_conectar:
                         self.test_conectar=False
                         self.svm_load=False
                         self.ann_load=False
                      self.clic_parar_p1()
                      return
                  if (event.type == pygame.KEYDOWN):
                      if (event.key == pygame.K_ESCAPE):
                         self.state=False
                         self.yparar=True
                         self.yguardar=False
                         self.ysimular=False
                         self.yconectar=False
                         if self.yguardar:
                            arxiu2.close()
                         pygame.display.quit()
                         if self.test_conectar:
                            self.test_conectar=False
                            self.svm_load=False
                            self.ann_load=False
                         self.clic_parar_p1()
                         return
                      elif (event.key == pygame.K_f):
                          if fullscreen:
                              screen = pygame.display.set_mode((self.screen_x_max, self.screen_y_max))
                              fullscreen = False
                          else:
                              screen = pygame.display.set_mode((self.screen_x_max,self.screen_y_max), FULLSCREEN, 16)
                              fullscreen = True
              self.packetsInQueue = 0
              while self.packetsInQueue < self.max_paquets and self.state:
                  self.senyal=[]
                  self.qualitat=[]
                  self.buffer2=[]
                  if self.ysimular and not self.yconectar and not self.yguardar:
                     try:
                        sequencia=pickle.load(arxiu)
                     except EOFError:
                        arxiu.seek(0)
                        cursor_x, cursor_y = self.screen_x_max-self.subscreen_x_max/2, self.subscreen_y_max/2
                        sequencia=pickle.load(arxiu)
                     self.lectura = sequencia[0]
                     self.gyroX = sequencia[1]
                     self.gyroY = sequencia[2]
                     self.senyal = sequencia[3]
                     self.qualitat = sequencia[4]
                     self.old_model = sequencia[5]
                     if self.ykalman_filter:
                        #                                                    
                        # Càlcul del filtre kalman pels valors de cada sensor
                        #                                                    
                        self.senyal_filtre=[]
                        nsens=0
                        for item in self.senyal:
                            z=item
                            filter[nsens].update(z)
                            filter[nsens].predict()
                            self.senyal_filtre.append(filter[nsens].x[0,0])
                            nsens=nsens+1
                        self.senyal=list(map(int,self.senyal_filtre))
                  if self.yconectar and not self.ysimular:
                     if self.headset:
                        self.packet = self.headset.dequeue()
                     if self.packet:
                        self.lectura=self.packet.counter
                        self.gyroX=self.packet.gyro_x
                        self.gyroY=self.packet.gyro_y
                        gyroX_sum=gyroX_sum+self.gyroX
                        gyroY_sum=gyroY_sum+self.gyroY
                        text=str(self.packet.battery)+'%'
                        self.labelbateria.setText(text)
                        for name in self.Sensors:
                            self.buffer2.append([self.packet.sensors[name]['value'], self.packet.sensors[name]['quality'], self.packet.old_model])
                        for i, (value, quality, old_model ) in enumerate(self.buffer2):
                            self.senyal.append(value)
                            self.qualitat.append(quality)
                            self.old_model = old_model
                        if self.ykalman_filter:
                           #                                                    
                           # Càlcul del filtre kalman pels valors de cada sensor
                           #                                                    
                           self.senyal_filtre=[]
                           nsens=0
                           for item in self.senyal:
                               z=item
                               filter[nsens].update(z)
                               filter[nsens].predict()
                               self.senyal_filtre.append(filter[nsens].x[0,0])
                               nsens=nsens+1
                           self.senyal=list(map(int,self.senyal_filtre))
#
# Aquí ja tenim self.senyal, self.qualitat, self.lectura, self.gyroX i self.giroY ... Ja podem guradar dades i dibuixar pygame
#
#
# Guardem dades al fitxer en format pickle
#
                  if (self.yguardar and not self.ysimular) or self.test_conectar:
                     if lec == self.nsamples:
                        if self.ycalibrar:
                           gyroX_sum=gyroX_sum-self.gyroX
                           gyroY_sum=gyroY_sum-self.gyroY
                           self.calibra_gyro_x=int(float(gyroX_sum)/float(self.nsamples))
                           self.calibra_gyro_y=int(float(gyroY_sum)/float(self.nsamples))
                           self.ycalibrat=True
                           self.ycalibrar=False
                        if self.ycalibrar or self.yguardar:
                           lec = 0
                           self.yguardar= False
                           arxiu2.close()
                           self.state=False
                           self.yparar=True
                           self.yguardar=False
                           self.ysimular=False
                           self.yconectar=False
                           pygame.display.quit()
                           self.clic_parar_p1()
                           return 
#
# Aquí fem l'extracció de característiques de la senyal i la introduim al classificador
#
                        if self.test_conectar:
                           self.iregistres=lec
                           self.carac_dat=[]
                           self.calcula_caracteristiques()
                           self.carac_dat=np.array(self.carac_dat, dtype=np.float32)
                           samples=np.array(self.carac_dat,dtype=np.float32)
#
# Prediccio del SVM
#
                           if self.svm_load:
                              clf = SVM()
                              clf.load(str(self.nom_fitxer_svm))
                              classe_predita = clf.predict(samples)
                              text=_('Classe predita pel SVM per aquest vector ----> ')
                              text=text+str(classe_predita)+'\n'
                              text=text.decode("utf-8")
                              self.progressBox_svm_test.insertPlainText(text)
                              print text
#
# Prediccio del ANN
#
                           if self.ann_load:
                              nnet=cv2.ANN_MLP()
                              nnet.load(str(self.nom_fitxer_ann))
                              predictions=nnet.predict(samples)
                              predic=predictions[1][0]
                              control=np.ones((len(predic)),dtype=np.int)
                              diference=np.rint(np.round(predic))-control
                              classe_predita=np.argmax(diference)+1
                              text=_('Vector de la predicció per aquesta senyal ----> ')
                              text=text+str(predic)+'\n'
                              text=text.decode("utf-8")
                              print text
                              self.progressBox_ann_test.insertPlainText(text)
                              text=_('Classe predita pel ANN per aquesta senyal ----> ')
                              text=text+str(classe_predita)+'\n'
                              text=text.decode("utf-8")
                              self.progressBox_ann_test.insertPlainText(text)
                              print text
                           self.DATAT=[]
                           lec=0
                     else:
                        lec=lec+1
                     if self.test_conectar:
                        self.DATAT.append(self.senyal)
                     if self.yguardar:
                        sequencia=[]
                        sequencia.append(self.lectura)
                        sequencia.append(self.gyroX)
                        sequencia.append(self.gyroY)
                        sequencia.append(self.senyal)
                        sequencia.append(self.qualitat)
                        sequencia.append(self.packet.old_model)
                        pickle.dump(sequencia, arxiu2)
#
# Imprimim resultats al terminal si self.y_print (Ojo que pot ralentir el programa)
#
                  if self.y_print:
                     text=_("Lectura número: ")
                     txt_lectura=text+str(self.lectura)
                     txt_lectura=txt_lectura.decode('utf-8')
                     self.print_there(22.,0.,txt_lectura)
                     text=_("Valors dels sensors: ")
                     txt_senyal=text+str(self.senyal)+"                            "
                     txt_senyal=txt_senyal.decode('utf-8')
                     self.print_there(24.,0.,txt_senyal)
                     text=_("Valors de la qualitat: ")
                     txt_qualitat=text+str(self.qualitat)+"                            "
                     txt_qualitat=txt_qualitat.decode('utf-8')
                     self.print_there(26.,0.,txt_qualitat)
#
# Contrucció de l'estring Nom_Sensors = Valor per printar al pygame
#
                  item=0
                  text_sensor=[]
                  for valor in self.senyal:
                      text_sensor.append(self.Sensors[item]+" --> "+str(valor))
                      item=item+1
#
# Fase de dibuix
#
#
# Variació del gyroscopy ---> self.calibra_gyro_x i self.calibra_gyro_y són valors per calibrar el giroscopy
# per calcular aquests valors fem mesura del casc en repos (sense cap moviment) i ens fixem en els valors que proporciona.
# Aquest valors s'han de sumar de forma que al casc en repos els valors de gyroX i gyroY siguin 0.
#
                  if lec == 8*128:
                     cursor_x, cursor_y = self.screen_x_max-self.subscreen_x_max/2, self.subscreen_y_max/2+10
                  if abs(self.gyroX) > 1:
                      cursor_x -= self.gyroX-self.calibra_gyro_x
                  if abs(self.gyroY) > 1:
                      cursor_y += self.gyroY-self.calibra_gyro_y
#
# Activació d'un cursor amb el giroscopy dins d'una subfinestra al cantó dret superior de la pantalla
#
                  pygame.draw.rect(screen, (50,50,75), 
                                   (self.screen_x_max-self.subscreen_x_max, 10,
                                    self.subscreen_x_max-10, self.subscreen_y_max-10), 0)
#
# rectangle per escriure els valors dels sensors
#
                  pygame.draw.rect(screen, (50,50,75),
                                   (self.screen_x_max-self.subscreen_x_max, self.subscreen_y_max+10,
                                    self.subscreen_x_max-10, self.screen_y_max-self.subscreen_y_max-20), 0)
                  pygame.display.update((self.screen_x_max-self.subscreen_x_max, 0,
                                         self.subscreen_x_max, self.screen_y_max))
                  x = cursor_x
                  y = cursor_y
                  if x > (self.screen_x_max-10):
                     x = self.screen_x_max-10
                  elif x < (self.screen_x_max-self.subscreen_x_max):
                     x = self.screen_x_max-self.subscreen_x_max
                  if y > (self.subscreen_y_max-10):
                     y = self.subscreen_y_max-10
                  elif y < (10):
                     y = 10
                  #print x,y
                  pygame.draw.polygon(screen, (0,150,0),
                                      ((self.screen_x_max-10, self.subscreen_y_max/2+10),
                                       (self.screen_x_max-self.subscreen_x_max-10, self.subscreen_y_max/2+10),
                                       (x,y)),1) 
                  pygame.draw.polygon(screen, (0,150,0),
                                      ((self.screen_x_max-self.subscreen_x_max/2, 10),
                                       (self.screen_x_max-self.subscreen_x_max/2, self.subscreen_y_max),
                                       (x,y)),1) 
                  pygame.draw.circle(screen,(255,255,255),(x,y),10)
                     
#
# Update sensors' values to pygame windows
#
                  y_delta=(self.screen_y_max-self.subscreen_y_max-20)/15
                  x_text=self.screen_x_max-self.subscreen_x_max+40
                  y_text=self.subscreen_y_max+y_delta
                  text=_('Mesura dels sensors')
                  text_titol = font.render(text, 1, (255, 255, 255))
                  screen.blit(text_titol, (x_text+10, y_text-10))
                  y_text=y_text+y_delta
                  for item in text_sensor:
                      text0 = item+' microVolts'
                      text = font.render(text0, 1, (255, 255, 255))
                      screen.blit(text, (x_text, y_text))
                      y_text=y_text+y_delta*0.9
                  pygame.display.update((self.screen_x_max-self.subscreen_x_max, 0,
                                         self.subscreen_x_max, self.screen_y_max))
#
# Dibuix de la senyal dels sensors
#
                  map(lambda x: x.update(self.packet,self.ysimular,self.senyal,self.qualitat,self.old_model), graphers)
                  updated = True
                  self.packetsInQueue += 1
              if updated:
                  screen.fill((250, 250, 250))
                  map(lambda x: x.draw(), graphers)
                  pygame.display.update((0, 0,
                                         self.screen_x_max-self.subscreen_x_max, self.screen_y_max))
                  pygame.image.save(screen,"./images/screen.jpg")
                  updated = False
              if self.yconectar:
                 gevent.sleep(0)
#
# Final del while True
#
          pygame.display.quit()
          if self.yconectar:
             self.packet=None
             self.yconectar=False
          if self.yguardar:
             arxiu2.close()
             self.yguardar=False
          if self.ysimular:
             arxiu.close()
             self.ysimular=False
          self.state=False
          self.yparar=True
          return
#
# Funció per printar el text en una posició (x,y) determinada del terminal
#
      def print_there(self, x, y, text):
           sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (x, y, text))
           sys.stdout.flush()
#
# Funcio per buscar el casc en linux amb hidraw
#
      def buscaCasc(self):
          rawinputs = []
          for filename in os.listdir("/sys/class/hidraw"):
              try:
                 realInputPath = check_output(["realpath", "/sys/class/hidraw/" + filename])
              except:
                 return False
              sPaths = realInputPath.split('/')
              s = len(sPaths)
              s = s - 4
              i = 0
              path = ""
              while s > i:
                  path = path + sPaths[i] + "/"
                  i += 1
              rawinputs.append([path, filename])
          hiddevices = []
          for input in rawinputs:
              try:
                  with open(input[0] + "/manufacturer", 'r') as f:
                      manufacturer = f.readline()
                      f.close()
                  if "Emotiv Systems" in manufacturer:
                      with open(input[0] + "/serial", 'r') as f:
                          serial = f.readline().strip()
                          f.close()
                      #print "Serial: " + serial + " Device: " + input[1]
                      # Great we found it. But we need to use the second one...
                      hidraw = input[1]
                      hidraw_id = int(hidraw[-1])
                      # The dev headset might use the first device, or maybe if more than one are connected they might.
                      hidraw_id += 1
                      hidraw = "hidraw" + hidraw_id.__str__()
                      if self.yserial_print:
                         print "Serial: " + serial + " Device: " + hidraw + " (Active)"
                         self.yserial_print = False
                  if ("Emotiv Systems Inc." in manufacturer) or ("Emotiv Systems Pty Ltd" in manufacturer) :
                      return True
              except IOError as e:
                  print _("No es pot connectar amb el casc")
#
# Class Grapher
#
class Grapher(object):
      def __init__(self, screen, name, i, calibracio):
          self.Sensors=["AF3","F7","F3","FC5","T7","P7","O1","O2","P8","T8","FC6","F4","F8","AF4"]
          self.gheight = 475 / 14
          self.value0= self.gheight
          self.screen = screen
          self.name = name
          self.range = calibracio
          self.x_offset = 40
          self.y_offset = (i +1) * self.gheight
          self.buffer = []
          self.font = pygame.font.Font(None, 24)
          self.screen_x_max, self.screen_y_max = self.screen.get_size()
          self.subscreen_x_max=250
          self.subscreen_y_max=100
          self.first_packet=True

      def update(self, packet, yoscar, senyal, qualitat, old_model):
#
# buffer és una llista de parelles (valor,qualitat) per cada sensor
#
          if len(self.buffer) == self.screen_x_max - self.subscreen_x_max-50:
              self.buffer = self.buffer[1:]
          if yoscar:
             ipos=self.Sensors.index(self.name)    
             self.buffer.append([senyal[ipos],qualitat[ipos],old_model])
          else:
             self.buffer.append([packet.sensors[self.name]['value'], packet.sensors[self.name]['quality'],old_model])

      def calc_y(self, val0, val):
          """
          Calculates line height from value.
          """
          return (val0-val) * self.gheight / self.range 

      def draw(self):
          if len(self.buffer) == 0:
              return
          if self.first_packet: 
             self.value0 = self.buffer[0][0]
             self.first_packet=False
          for i, (value, quality, old_model ) in enumerate(self.buffer):
              x = self.x_offset + i
              if i > 0:
                 delta_y = self.calc_y(self.value0,value)
                 y = self.y_offset + delta_y
                 poss = (x, y)
                 color = (0,0,0)
                 pygame.draw.line(self.screen, color, pos, poss)
                 pos=(x,y)
              elif i == 0:
                 y = self.y_offset
                 pos=(x,y)
              if old_model:
                  color = old_quality_color[str(quality)]
              else:
                  color = quality_color[str(quality)]
              self.text = self.font.render(self.name, 1, color)
              self.text_pos = self.text.get_rect()
              self.text_pos.centery = self.y_offset
          self.screen.blit(self.text, self.text_pos)
#
# Fluxe principal del program
#
def main(argv): 
#
# PyQt exec
#
    language = None
    app = QtGui.QApplication(sys.argv)
    if len(argv) > 0 :
       language = argv[0]
    ex = Emotiv(language)
    sys.exit(app.exec_())

if __name__ == '__main__':
     main(sys.argv[1:])
