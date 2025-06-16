from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import keyboard
import win32gui
import time
from threading import Thread
import sys

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
    

class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        super(PlotWindow, self).__init__(parent)
        # 重新调整大小
        self.resize(800, 659)
        # 添加菜单中的按钮
        self.menu = QMenu("绘图")
        self.menu_action = QAction("绘制",self.menu)
        self.menu.addAction(self.menu_action)
        self.menuBar().addMenu(self.menu)
        # 添加事件
        self.menu_action.triggered.connect(self.plot_)
        self.setCentralWidget(QWidget())
    def setFigure(self, figur, ax):
        self.figur = figur
        self.ax = ax
        cavans = FigureCanvas(self.figur)
        # 将绘制好的图像设置为中心 Widget
        self.setCentralWidget(cavans)
    
    # 绘图方法
    def plot_(self):
        # 清屏
        plt.cla()
        # 获取绘图并绘制
        fig = plt.figure()
        ax =fig.add_axes([0.1,0.1,0.8,0.8])
        ax.set_xlim([-1,6])
        ax.set_ylim([-1,6])
        ax.plot([0,1,2,3,4,5],'o--')

class TableTips(QWidget):
    def __init__(self, handle):
        super(TableTips, self).__init__()
        self.handle = handle
        self.btnMaxSize = 500
        self.hwnd_title = dict()
        #self.setWindowFlags( Qt.WindowStaysOnTopHint|Qt.CustomizeWindowHint|Qt.Tool | Qt.X11BypassWindowManagerHint)
        self.setWindowTitle('')
        self.initAllWidgets()
        self.move(1000,0)
        self.resize(800, 800)
        self.show()
        
    def initAllWidgets(self):
        vlayout = QVBoxLayout()
        layout = self.initOrderWidgets()
        vlayout.addLayout(layout)
        layout = self.initDisplayWidgets()
        vlayout.addLayout(layout)
        layout = self.initExcute()
        vlayout.addLayout(layout)
        
        self.setLayout(vlayout)
    def initOrderWidgets(self):
        vlayout = QVBoxLayout()
        hlayout = QHBoxLayout()
        btn_vlayout = QVBoxLayout()
        
        self.text = QTextEdit()
        self.text.setMaximumHeight(150)
        vlayout.addWidget(self.text)
        
        self.timesText = QLineEdit()
        vlayout.addWidget(self.timesText)
        
        hlayout.addLayout(vlayout)
        
        btn_send = QPushButton('send', self)
        btn_send.setShortcut('Esc')
        btn_send.setToolTip('按照当前命令框给出的orders来进行控制')
        btn_send.setMaximumSize(self.btnMaxSize, self.btnMaxSize)
        btn_send.clicked.connect(self.setSendFlag)
        btn_vlayout.addWidget(btn_send)
        
        btn_go = QPushButton('just_go', self)
        btn_go.setToolTip('按照控制算法给出的orders来进行控制')
        btn_go.setMaximumSize(self.btnMaxSize, self.btnMaxSize)
        btn_go.clicked.connect(self.justGo)
        btn_vlayout.addWidget(btn_go)
        # btn_purchase = QPushButton('purchase', self)
        # btn_purchase.clicked.connect(self.setSendFlag)
        # self.btn_vlayout.addWidget(btn_purchase)
        
        hlayout.addLayout(btn_vlayout)
        return hlayout
    def initDisplayWidgets(self):
        vlayout = QVBoxLayout()
        hlayout = QHBoxLayout()
        btn_vlayout = QVBoxLayout()
        
        self.textDisplay = QTextEdit()
        vlayout.addWidget(self.textDisplay)
        hlayout.addLayout(vlayout)
        
        btn_send = QPushButton('clear', self)
        btn_send.setMaximumSize(self.btnMaxSize, self.btnMaxSize)
        btn_send.clicked.connect(lambda: self.clearTextDisplay(self.textDisplay))
        btn_vlayout.addWidget(btn_send)
        
        # btn_purchase = QPushButton('purchase', self)
        # btn_purchase.clicked.connect(self.setSendFlag)
        # self.btn_vlayout.addWidget(btn_purchase)
        
        hlayout.addLayout(btn_vlayout)
        
        return hlayout
    def initExcute(self):
        vlayout = QVBoxLayout()
        vlayout_right = QVBoxLayout()
        hlayout = QHBoxLayout()
        
        self.excuteText = QTextEdit()
        self.excuteText.setMaximumHeight(50)
        vlayout.addWidget(self.excuteText)
        
        self.excuteTextDisplay = QTextEdit()
        vlayout.addWidget(self.excuteTextDisplay)
        
        hlayout.addLayout(vlayout)
        
        btn_excute = QPushButton('excute', self)
        btn_excute.setShortcut('Ctrl+Q')
        btn_excute.setMaximumSize(self.btnMaxSize, self.btnMaxSize)
        btn_excute.clicked.connect(self.excuteSentence)
        vlayout_right.addWidget(btn_excute)
        
        btn_send = QPushButton('clear', self)
        btn_send.setMaximumSize(self.btnMaxSize, self.btnMaxSize)
        btn_send.clicked.connect(lambda: self.clearTextDisplay(self.excuteTextDisplay))
        vlayout_right.addWidget(btn_send)
        
        hlayout.addLayout(vlayout_right)
        return hlayout
    
    def setSendFlag(self):
        self.goToVScode()
        keyboard.press_and_release("enter")
        self.handle = win32gui.GetWindowText(win32gui.GetForegroundWindow())#获取vscode句柄
        time.sleep(0.1)
        #self.setFocus()
    def excuteSentence(self):
        self.goToVScode()
        keyboard.write(self.excuteText.toPlainText())
        keyboard.press_and_release('enter')
        self.handle = win32gui.GetWindowText(win32gui.GetForegroundWindow())#获取vscode句柄
        time.sleep(0.1)
    def getOrder(self):
        orders = self.text.toPlainText()
        order_list = orders.split('\n')
        out_order_list = []
        for order in order_list:
            if order != '':
                out_order_list.append(order)
        
        try:
            times = int(self.timesText.text())
        except Exception:
            times = 0
            pass
        
        return out_order_list, times
    def justGo(self):
        self.goToVScode()
        keyboard.write('just_go')
        keyboard.press_and_release("enter")
        self.handle = win32gui.GetWindowText(win32gui.GetForegroundWindow())#获取vscode句柄
        time.sleep(0.1)
    
    def setText(self, text_widget, text):
        text_widget.setText(text)
    def setDisplayText(self, text):
        self.textDisplay.moveCursor(QTextCursor.Start)
        self.textDisplay.insertPlainText(text)
    def clearTextDisplay(self, text_widget):
        text_widget.clear()
    def setDisplayExcute(self, text):
        self.excuteTextDisplay.moveCursor(QTextCursor.Start)
        self.excuteTextDisplay.insertPlainText(text)
    
    def get_all_hwnd(self, hwnd, mouse):
        if win32gui.IsWindow(hwnd) and win32gui.IsWindowEnabled(hwnd) and win32gui.IsWindowVisible(hwnd):
            self.hwnd_title.update({hwnd:win32gui.GetWindowText(hwnd)})
    def searchVScode(self):
        win32gui.EnumWindows(self.get_all_hwnd, 0)
        for h,t in self.hwnd_title.items():
            if 'Visual Studio Code' in t:
                self.handle = t
    def goToVScode(self):
        if 'Visual Studio Code' in self.handle:
            try:
                handle = win32gui.FindWindow(None, self.handle)
                win32gui.SetForegroundWindow(handle)
            except:
                self.searchVScode()
                handle = win32gui.FindWindow(None, self.handle)
                win32gui.SetForegroundWindow(handle)
                
class WindowManager(Thread):
    def __init__(self, vscode_handle):
        Thread.__init__(self)
        self.vscode_handle = vscode_handle
    def run(self):
        
        order_window = TableTips(self.vscode_handle)
        
        
