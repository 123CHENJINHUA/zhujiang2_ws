from PyQt5 import QtWidgets, QtCore, QtGui
from ui_pkg.scripts.Four_cabin.Face2 import Ui_MainWindow
import ui_pkg.scripts.Four_cabin.resources_rc

class FaceWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    """
    人脸识别界面。在此界面双击机器人头像可返回主页。
    构造函数接受 WindowManager 引用，用于切换界面。
    """

    def __init__(self, manager=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        # 窗口管理器
        self.manager = manager

        # 设置 GIF 动画
        self.movie = QtGui.QMovie(":/icons/icons/Robot-Face.gif")
        self.label.setMovie(self.movie)

        # 启动 GIF 动画并设置循环播放
        self.movie.setCacheMode(QtGui.QMovie.CacheAll)
        self.movie.setSpeed(100)  # 正常速度
        # PyQt5 的 QMovie 默认就是无限循环播放
        self.movie.start()

        # 设置双击事件，双击机器人返回主页
        self.label.mouseDoubleClickEvent = self.on_double_click
        # 使标签可以接收鼠标事件
        self.label.setMouseTracking(True)
        self.label.setCursor(QtCore.Qt.PointingHandCursor)  # 设置鼠标指针为手型

        # 初始化时不立即显示
        # self.showFullScreen()
        
    def on_double_click(self, event):
        """
        双击机器人头像时返回主页。通过 WindowManager 调用切换到主页。
        """
        # 直接请求窗口管理器切换到主页，不主动调用 hide()。
        # 隐藏由 WindowManager 统一处理，以避免切换时桌面露出。
        if self.manager:
            self.manager.show_main_window()
        
    def showEvent(self, event):
        # 窗口显示时确保GIF动画正在播放
        if self.movie and self.movie.state() != QtGui.QMovie.Running:
            self.movie.start()
        
        # 设置全屏显示
        self.showFullScreen()
        
        super().showEvent(event)