import sys

sys.path.insert(0, "/home/cjh/zhujiang2_ws/src")
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")

import rospy
from std_msgs.msg import String
import threading
import queue
import time
from playsound3 import playsound
from robot_msgs.msg import ui_show
from robot_msgs.msg import Door_open
from robot_msgs.srv import ui_get, ui_getRequest, ui_getResponse
from robot_msgs.srv import pick, pickResponse

from PyQt5.QtCore import QObject, pyqtSignal

import ui_pkg.scripts.Four_cabin.resources_rc
from PyQt5 import QtWidgets, QtCore, QtGui
from ui_pkg.scripts.Four_cabin.homepage import Ui_MainWindow
from ui_pkg.scripts.Four_cabin.Fourbins import Ui_Form as Ui_Fourbins
from ui_pkg.scripts.Four_cabin.send2 import Ui_Form as Ui_Send2
from ui_pkg.scripts.Four_cabin.TaskSuccessDialog2_class import TaskSuccessDialog
from ui_pkg.scripts.Four_cabin.Face2_class import FaceWindow

import ui_pkg.scripts.Four_cabin.status_bar_controller2 as status_bar_ctrl  # 你的状态栏控制器

class UiNode(QObject):
    signal_recv_msg = pyqtSignal(int)  # Qt信号

    def __init__(self):
        super(UiNode, self).__init__()
        # 订阅 /UI_show
        self.ui_show_sub = rospy.Subscriber("/UI_show", ui_show, self.ui_show_callback)
        # 订阅 /speach
        self.speach_sub = rospy.Subscriber("/speach", String, self.speach_callback)
        # 订阅 /calling
        self.calling_sub = rospy.Subscriber("/calling", String, self.calling_callback)
        # ui客户端
        self.ui_get_client = rospy.ServiceProxy('/UI_get', ui_get)

        # pickup 服务端
        self.pickup_service = rospy.Service('/pickup', pick, self.handle_pickup)

        # 添加Door_open话题发布者
        self.door_open_pub = rospy.Publisher('/ui_door_open', Door_open, queue_size=10)

        self.pickup_result = None  # 用于存储取件码结果
        self.Is_need_pickup_code = False  # 用于标记是否需要取件
        self.arrived = False  # 用于标记是否到达目的地

        # 音频播放相关
        self.voice_msgs_path = "/home/cjh/zhujiang2_ws/src/ui_pkg/voice_msgs"

        self.sound = playsound(f'{self.voice_msgs_path}/{99}.wav', block=False)

        self.is_music = False  # 用于判断是否正在播放音乐

        # 创建音频播放线程
        self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.audio_thread.start()

    def publish_door_open(self, door_number):
        """发布开门消息"""
        try:
            msg = Door_open()
            msg.door_num = door_number
            self.door_open_pub.publish(msg)
            rospy.loginfo(f"Published door open message: door_num={door_number}")
        except Exception as e:
            rospy.logerr(f"Failed to publish door open message: {e}")

    def _audio_worker(self):
        while not rospy.is_shutdown():
            try:
                if self.is_music and not self.sound.is_alive():  # 检查音频路径是否有效且当前没有音频在播放
                    self.sound = playsound(f'{self.voice_msgs_path}/{100}.wav', block=False)  # 播放音频
            except Exception as e:
                rospy.logerr(f"Audio playback error: {e}")


    # 订阅
    def ui_show_callback(self, msg):
        # rospy.loginfo("network: %s, odometry: %s, speed: %s, working_time: %s, battery: %s, task_status: %s", msg.network, msg.odometry, msg.speed, msg.working_time, msg.battery, msg.task_status)
        self.signal_recv_msg.emit(msg.battery)  # 发射任务状态信号(battery)

    def speach_callback(self, msg):
        wav_path = f'{self.voice_msgs_path}/{msg.data}.wav'
        rospy.loginfo("Received speech message: %s", msg.data)
        if msg.data == '100':
            self.is_music = True  # 设置正在播放音乐状态
        else:
            if self.is_music:
                self.sound.stop()  # 停止当前音乐
                self.is_music = False  # 重置音乐状态
            playsound(wav_path, block=True)  # 播放语音消息
            


    def calling_callback(self, msg):
        rospy.loginfo("data: %s", msg.data)

    # 客户端
    def call_ui_get(self, bin_numbers, delivery_addresses):
        try:
            req = ui_getRequest()
            req.bin_numbers = bin_numbers
            req.delivery_addresses = delivery_addresses
            resp = self.ui_get_client.call(req)
            if resp.received:
                rospy.loginfo("UI_get call success, task list received.")
            else:
                rospy.logwarn("UI_get call failed, task list not accepted.")
            return resp
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
        
    def handle_pickup(self, req):
        rospy.loginfo("Received pickup request: %d", req.pickup_code)
        self.pickup_result = None  # 每次请求前重置
        self.arrived = True

        self.Is_need_pickup_code = False
        resp = pickResponse()

        # 等待用户输入，直到 self.pickup_result 被设置
        while self.pickup_result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        resp.success = self.pickup_result if self.pickup_result is not None else False
        return resp


# ----------- SendWindow -------------
class SendWindow(QtWidgets.QWidget, Ui_Send2):
    """
    收货地址选择窗口。通过 WindowManager 统一管理窗口切换，不再直接引用
    上一个窗口。构造函数接受 manager 和 bin_index，用于在任务确认后回调
    WindowManager 更新货仓状态。
    """

    # 发出用户选择的配送地址的信号
    addressConfirmed = QtCore.pyqtSignal(str)

    def __init__(self, manager=None, comm_node=None, bin_index: int = None):
        super().__init__()
        self.setupUi(self)
        # 去掉原生边框
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.comm_node = comm_node

        # 窗口管理器
        self.manager = manager
        # 当前操作的货仓索引
        self.bin_index = bin_index

        # 房间号
        self.room_number: str = ""
        self.init_dropdowns()
        self.setup_frequent_addresses()
        self.setup_numeric_keypad()

        # 状态栏注册和定时刷新
        status_bar_ctrl.register(self.lblTime, None, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(lambda: status_bar_ctrl.refresh(self.lblTime, None, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node))
        self.timer.start(1000)

        # 连接按钮事件
        self.btnBack.clicked.connect(self.go_back)
        self.btnCreateTaskConfirm.clicked.connect(self.on_create_task_confirm)
        # 修改房号编辑框的点击事件，实现键盘震动效果
        self.editRoom.mousePressEvent = self.on_room_input_clicked

        # 存储键盘按钮的原始样式
        self.original_keypad_styles: dict = {}
        for i in range(10):
            button = getattr(self, f"btnNum{i}")
            self.original_keypad_styles[button] = button.styleSheet()
        self.original_keypad_styles[self.btnNumClear] = self.btnNumClear.styleSheet()
        self.original_keypad_styles[self.btnNumConfirm] = self.btnNumConfirm.styleSheet()

        # 初始化界面
        self.reset_ui()
        # 默认全屏显示
        self.showFullScreen()
    
    def closeEvent(self, event):
        """
        窗口关闭时清理资源。由于所有窗口交由 WindowManager 管理，一般情况下
        我们不会主动调用 close()，而是通过 hide() 隐藏窗口；这里仍然保留
        清理逻辑，以防应用退出时释放资源。
        """
        try:
            # 注销状态栏中的标签，避免在对象已删除后继续刷新导致的错误
            try:
                status_bar_ctrl.unregister(self.lblTime, None, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node)
            except Exception:
                pass
            if hasattr(self, 'timer') and self.timer:
                self.timer.stop()
                self.timer.deleteLater()
                self.timer = None
        except Exception as e:
            print(f"Error in SendWindow closeEvent: {e}")
        super().closeEvent(event)
    
    # 旧版创建任务确认方法，被新版 on_create_task_confirm2 取代。保留在此以供参考，但不再连接信号。
    def _unused_on_create_task_confirm(self):
        try:
            if self.cmbBuilding.currentText() == "--":
                self.vibrate_field(self.cmbBuilding)
                return
            if self.cmbUnit.currentText() == "--":
                self.vibrate_field(self.cmbUnit)
                return
            if not self.room_number:
                self.vibrate_field(self.editRoom)
                return
            self.update_current_address()
            current_address = self.frmCurrentAddress.text()
            if not current_address:
                QtWidgets.QMessageBox.warning(self, "提示", "请先选择配送地址")
                return
            
            # 先通知主窗口准备显示，但不立即关闭当前窗口
            self.addressConfirmed.emit(current_address)
            self.reset_ui()
            
            # 不要立即关闭，让接收信号的方法来处理窗口切换
            # QtCore.QTimer.singleShot(100, self.close)  # 注释掉这行
            
        except Exception as e:
            print(f"Error in on_create_task_confirm: {e}")
            QtWidgets.QMessageBox.critical(self, "错误", f"操作失败: {str(e)}")

    # 旧版返回方法，被新版 go_back2 取代。保留在此以供参考，但不再连接信号。
    def _unused_go_back(self):
        try:
            self.addressConfirmed.emit("")
            QtCore.QTimer.singleShot(100, self.close)
        except Exception as e:
            print(f"Error in go_back: {e}")
    
    def init_dropdowns(self):
        self.cmbBuilding.clear()
        self.cmbUnit.clear()
        self.cmbBuilding.addItem("--")
        self.cmbUnit.addItem("--")
        for i in range(1, 6):
            self.cmbBuilding.addItem(f"{i}栋")
        for i in range(1, 4):
            self.cmbUnit.addItem(f"{i}单元")

    def setup_frequent_addresses(self):
        frequent_addresses = [
            {"building": "1", "unit": "2", "room": "301"},
            {"building": "2", "unit": "3", "room": "502"},
            {"building": "3", "unit": "1", "room": "101"}
        ]
        btns = [self.btnQuickAddr1, self.btnQuickAddr2, self.btnQuickAddr3]
        for idx, addr in enumerate(frequent_addresses):
            btns[idx].setText(f"{addr['building']}栋{addr['unit']}单元{addr['room']}室")
            btns[idx].clicked.connect(lambda checked=False, a=addr: self.use_quick_address(a))

    def use_quick_address(self, address):
        building_text = f"{address['building']}栋"
        unit_text = f"{address['unit']}单元"
        building_index = self.cmbBuilding.findText(building_text)
        if building_index >= 0:
            self.cmbBuilding.setCurrentIndex(building_index)
        unit_index = self.cmbUnit.findText(unit_text)
        if unit_index >= 0:
            self.cmbUnit.setCurrentIndex(unit_index)
        self.room_number = address["room"]
        self.editRoom.setText(self.room_number)
        self.update_current_address()

    def setup_numeric_keypad(self):
        for i in range(10):
            getattr(self, f"btnNum{i}").clicked.connect(lambda _, d=str(i): self.on_numeric_key_pressed(d))
        self.btnNumClear.clicked.connect(self.on_clear_pressed)
        self.btnNumConfirm.clicked.connect(self.on_confirm_pressed)

    def on_room_input_clicked(self, event):
        self.vibrate_keypad()
        QtWidgets.QLineEdit.mousePressEvent(self.editRoom, event)

    def vibrate_keypad(self):
        for i in range(10):
            button = getattr(self, f"btnNum{i}")
            button.setStyleSheet(self.original_keypad_styles[button] + "border: 2px solid red;")
        self.btnNumClear.setStyleSheet(self.original_keypad_styles[self.btnNumClear] + "border: 2px solid red;")
        self.btnNumConfirm.setStyleSheet(self.original_keypad_styles[self.btnNumConfirm] + "border: 2px solid red;")
        QtCore.QTimer.singleShot(300, self.restore_keypad_style)

    def restore_keypad_style(self):
        for button, style in self.original_keypad_styles.items():
            button.setStyleSheet(style)

    def vibrate_field(self, widget):
        original_style = widget.styleSheet()
        widget.setStyleSheet(original_style + "border: 2px solid red;")
        QtCore.QTimer.singleShot(300, lambda: widget.setStyleSheet(original_style))

    def on_numeric_key_pressed(self, digit):
        self.room_number += digit
        self.editRoom.setText(self.room_number)

    def on_clear_pressed(self):
        self.room_number = ""
        self.editRoom.setText("")
        self.cmbBuilding.setCurrentIndex(0)
        self.cmbUnit.setCurrentIndex(0)
        self.frmCurrentAddress.setText("")

    def on_confirm_pressed(self):
        self.update_current_address()

    def on_create_task_confirm(self):
        """
        确认创建配送任务。校验楼栋、单元和房间号输入后，通过信号发送
        选择的地址，并利用窗口管理器回调四仓窗口更新状态。
        """
        # 校验楼栋选择
        if self.cmbBuilding.currentText() == "--":
            self.vibrate_field(self.cmbBuilding)
            return
        # 校验单元选择
        if self.cmbUnit.currentText() == "--":
            self.vibrate_field(self.cmbUnit)
            return
        # 校验房间号输入
        if not self.room_number:
            self.vibrate_field(self.editRoom)
            return
        # 更新当前地址
        self.update_current_address()
        current_address = self.frmCurrentAddress.text()
        if not current_address:
            QtWidgets.QMessageBox.warning(self, "提示", "请先选择配送地址")
            return
        # 发送信号，外部可以连接此信号处理
        self.addressConfirmed.emit(current_address)
        # 通过窗口管理器处理配送任务确认逻辑。具体处理逻辑在 WindowManager
        # 的信号回调中实现，这里不直接调用 handle_send_confirm。
        # 重置界面，方便下次使用
        self.reset_ui()

    def update_current_address(self):
        building = self.cmbBuilding.currentText()
        unit = self.cmbUnit.currentText()
        if building == "--" or unit == "--" or not self.room_number:
            self.frmCurrentAddress.setText("")
            return
        full_address = f"{building}{unit}{self.room_number}室"
        self.frmCurrentAddress.setText(full_address)

    def reset_ui(self):
        self.room_number = ""
        self.editRoom.setText("")
        self.init_dropdowns()
        self.frmCurrentAddress.setText("")

    def go_back(self):
        """
        返回四仓界面。通过 WindowManager 调用，而不是直接操作上一个窗口。
        点击返回按钮或者用户取消操作时调用此方法。

        不再主动调用 hide()，隐藏逻辑统一由 WindowManager 处理。这可
        以保证界面切换时始终有一个窗口覆盖屏幕，避免短暂暴露桌面。
        """
        # 发出空地址信号，代表取消
        self.addressConfirmed.emit("")
        # 使用窗口管理器切换界面
        if self.manager:
            self.manager.show_fourbins_window()

# ----------- FourbinsWindow -------------
class FourbinsWindow(QtWidgets.QWidget, Ui_Fourbins):
    """
    四仓主界面。通过 WindowManager 统一管理窗口切换，不直接引用
    主窗口或 SendWindow。负责展示每个货仓的状态，并提供打开仓门、
    返回主页和确认配送等功能。
    """

    def __init__(self, manager=None, comm_node=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.comm_node = comm_node

        # 保存窗口管理器引用
        self.manager = manager

        # 货仓状态列表，每个元素包含状态、地址和图标
        self.bin_status = [
            {"status": "空闲状态", "address": "--", "image": "Box.png", "idx": 0},
            {"status": "空闲状态", "address": "--", "image": "Box.png", "idx": 1},
            {"status": "空闲状态", "address": "--", "image": "Box.png", "idx": 2},
            {"status": "空闲状态", "address": "--", "image": "Box.png", "idx": 3},
        ]

        # 状态栏注册和定时刷新
        status_bar_ctrl.register(self.label, None, self.signalLabel_2, self.wifiLabel_2, self.label_5,self.comm_node)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(lambda: status_bar_ctrl.refresh(self.label, None, self.signalLabel_2, self.wifiLabel_2, self.label_5,self.comm_node))
        self.timer.start(1000)

        # 按钮事件连接
        self.BackhomepageButton.clicked.connect(self.go_back)
        self.Button1.clicked.connect(lambda: self.handle_bin_button(0))
        self.Button2.clicked.connect(lambda: self.handle_bin_button(1))
        self.Button3.clicked.connect(lambda: self.handle_bin_button(2))
        self.Button4.clicked.connect(lambda: self.handle_bin_button(3))
        self.ConfirmButton.clicked.connect(self.confirm_delivery)

        # 初始化界面状态
        self.update_all_bins()
        # 默认全屏显示
        self.showFullScreen()
    
    def closeEvent(self, event):
        """
        窗口关闭时清理资源。这里停止状态栏定时器并注销控件。
        其它子窗口由 WindowManager 管理，不在此处手动关闭。
        """
        try:
            # 注销状态栏中的标签，避免在对象已删除后继续刷新导致的错误
            try:
                status_bar_ctrl.unregister(self.label, None, self.signalLabel_2, self.wifiLabel_2, self.label_5,self.comm_node)
            except Exception:
                pass
            if hasattr(self, 'timer') and self.timer:
                self.timer.stop()
                self.timer.deleteLater()
                self.timer = None
        except Exception as e:
            print(f"Error in FourbinsWindow closeEvent: {e}")
        super().closeEvent(event)

    def handle_bin_button(self, bin_index):
        try:
            status = self.bin_status[bin_index]["status"]
            # 如果仓门空闲，则由 WindowManager 打开送货窗口
            if status == "空闲状态":
                self.comm_node.publish_door_open(door_number=bin_index) #打开仓门
                if self.manager:
                    self.manager.show_send_window(bin_index)
            elif status == "已装货":
                # 已装货状态显示详情
                QtWidgets.QMessageBox.information(
                    self,
                    "已有包裹",
                    f"{bin_index + 1}号仓已经有包裹\n配送地址: {self.bin_status[bin_index]['address']}",
                )
            else:
                # 其他状态（维护中等）
                QtWidgets.QMessageBox.information(self, "提示", f"{bin_index + 1}号仓不可用")
        except Exception as e:
            print(f"Error in handle_bin_button: {e}")
            QtWidgets.QMessageBox.critical(self, "错误", f"操作失败: {str(e)}")

    def on_address_confirmed(self, bin_index, address):
        try:
            # 根据地址更新货仓状态和地址
            if address:
                self.bin_status[bin_index]["status"] = "已装货"
                self.bin_status[bin_index]["address"] = address
                self.bin_status[bin_index]["image"] = "package_in_box.png"
            # 刷新界面展示
            self.update_all_bins()
        except Exception as e:
            print(f"Error in on_address_confirmed: {e}")

    # 旧版返回方法，被新版 go_back2 取代。保留在此以供参考。
    def _unused_go_back(self):
        try:
            if self.main_window:
                self.main_window.show()
                self.main_window.raise_()
                self.main_window.activateWindow()
                QtCore.QTimer.singleShot(50, self.close)
        except Exception as e:
            print(f"Error in go_back: {e}")
    
    # 旧版确认配送方法，被新版 confirm_delivery2 取代。保留在此以供参考。
    def _unused_confirm_delivery(self):
        try:
            loaded_bins = [bin_info for bin_info in self.bin_status if bin_info["status"] == "已装货"]
            if not loaded_bins:
                QtWidgets.QMessageBox.information(self, "提示", "没有需要派送的包裹")
                return
            
            addresses = [bin_info["address"] for bin_info in loaded_bins]
            dialog = TaskSuccessDialog(addresses=addresses, parent=self)
            result = dialog.exec_()
            
            if result == QtWidgets.QDialog.Accepted:
                for i, bin_info in enumerate(self.bin_status):
                    if bin_info["status"] == "已装货":
                        self.bin_status[i]["status"] = "空闲状态"
                        self.bin_status[i]["address"] = "--"
                        self.bin_status[i]["image"] = "Box.png"
                
                self.update_all_bins()
                self.manager.show_face_window()
            else:
                QtWidgets.QMessageBox.information(self, "提示", "已取消派送任务")
        except Exception as e:
            print(f"Error in confirm_delivery: {e}")
            QtWidgets.QMessageBox.critical(self, "错误", f"操作失败: {str(e)}")

    def _unused_closeEvent(self, event):
        """
        旧版 closeEvent 方法，保留以供参考。
        真实的资源释放在前面的 closeEvent 方法中已经处理。
        """
        try:
            status_bar_ctrl.unregister(self.label, None, self.signalLabel_2, self.wifiLabel_2, self.label_5,self.comm_node)
        except Exception:
            pass
        if hasattr(self, 'timer') and self.timer:
            self.timer.stop()
            self.timer.deleteLater()
        super().closeEvent(event)
    
    def update_all_bins(self):
        for i in range(4):
            name_label = getattr(self, f"Name{i+1}")
            status_label = getattr(self, f"Status{i+1}")
            address_label = getattr(self, f"Address{i+1}")
            light_label = getattr(self, f"Light{i+1}" if i != 3 else "light4")
            button = getattr(self, f"Button{i+1}")
            box_label = getattr(self, f"Box{i+1}")
            name_label.setText(f"{i+1}号货仓")
            status = self.bin_status[i]["status"]
            status_label.setText(status)
            address_label.setText(f"地址：{self.bin_status[i]['address']}")
            if status == "空闲状态":
                light_label.setStyleSheet("background:#23c037; border-radius:6px;")
                button.setEnabled(True)
                button.setText("打开仓门")
                box_label.setPixmap(QtGui.QPixmap(":/icons/icons/Box.png"))
            elif status == "已装货":
                light_label.setStyleSheet("background:#ff8c1a; border-radius:6px;")
                button.setEnabled(True)
                button.setText("查看详情")
                box_label.setPixmap(QtGui.QPixmap(":/icons/icons/package_in_box.png"))
            else:
                light_label.setStyleSheet("background:#888; border-radius:6px;")
                button.setEnabled(False)
                button.setText("维护中")
                box_label.setPixmap(QtGui.QPixmap(":/icons/icons/Box.png"))

    def go_back(self):
        """
        返回主页。通过 WindowManager 切换界面，不直接引用主窗口。

        不再主动调用 hide()，隐藏逻辑统一由 WindowManager 完成，
        以保证界面切换时不会短暂出现桌面或其他应用。
        """
        if self.manager:
            self.manager.show_main_window()
            
    def confirm_delivery(self):
        # 检查是否有已装货的货仓
        loaded_bins = [bin_info for bin_info in self.bin_status if bin_info["status"] == "已装货"]
        if not loaded_bins:
            QtWidgets.QMessageBox.information(self, "提示", "没有需要派送的包裹")
            return
            
        # 获取所有已装货货仓的地址
        addresses = [bin_info["address"] for bin_info in loaded_bins]

        # 获取所有已装货货仓的箱号和地址
        bin_numbers = []
        addresses_send = []
        for bin_info in loaded_bins:
            addr = bin_info["address"]  # 例如: "1栋2单元301室"
            
            
            # 提取栋号 (去掉"栋"字)
            building = addr.split('栋')[0]
            
            # 提取单元号 (去掉"单元"字)
            unit = addr.split('单元')[0].split('栋')[1]
            
            # 提取室号 (去掉"室"字)
            room = addr.split('单元')[1].replace('室', '')
            
            # 处理层号和房间号
            if len(room) == 3:  # 3位数：第1位是层，后2位是室
                floor = room[0]
                room_num = room[1:]
            elif len(room) == 4:  # 4位数：前2位是层，后2位是室
                floor = room[:2]
                room_num = room[2:]
            else:
                # 处理其他情况，可以根据需要添加
                continue
                
            bin_numbers.append(bin_info["idx"])  # 添加箱号
            # 添加到地址列表
            addresses_send.append(f"{building},{unit},{floor},{room_num}")
        
        # 创建并显示任务成功对话框
        dialog = TaskSuccessDialog(comm_node=self.comm_node, addresses=addresses, bin_numbers=bin_numbers, delivery_addresses=addresses_send, parent=self)
        result = dialog.exec_()
        
        if result == QtWidgets.QDialog.Accepted:
            # 用户点击确认按钮：重置所有已装货的货仓状态
            for i, bin_info in enumerate(self.bin_status):
                if bin_info["status"] == "已装货":
                    self.bin_status[i]["status"] = "空闲状态"
                    self.bin_status[i]["address"] = "--"
                    self.bin_status[i]["image"] = "Box.png"
            # 更新界面显示
            self.update_all_bins()
            # 通过 WindowManager 显示人脸识别界面
            if self.manager:
                self.manager.show_face_window()
            # 不在此处主动隐藏当前窗口，隐藏逻辑由 WindowManager 统一处理
        else:
            # 用户点击取消按钮：不做状态更改，只提示
            QtWidgets.QMessageBox.information(self, "提示", "已取消派送任务")

# ----------- MainWindow -------------
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    """
    应用主页界面。点击屏幕中央进入四仓界面，通过 WindowManager 管理窗口
    切换，不直接创建 FourbinsWindow。
    """

    def __init__(self, manager=None, comm_node=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.comm_node = comm_node

        # 保存窗口管理器引用
        self.manager = manager

        # 状态栏注册和定时刷新
        status_bar_ctrl.register(self.lblTime, self.lblDate, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(lambda: status_bar_ctrl.refresh(self.lblTime, self.lblDate, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node))
        self.timer.start(1000)

        # 点击整个框架进入四仓界面
        self.frame.mousePressEvent = self.open_fourbins_window
        # 默认全屏显示
        self.showFullScreen()

    def closeEvent(self, event):
        """
        窗口关闭时清理资源。停止状态栏更新，不主动关闭其他窗口，
        其他窗口由 WindowManager 管理。
        """
        try:
            # 注销状态栏中的标签
            try:
                status_bar_ctrl.unregister(self.lblTime, self.lblDate, self.lblSignal, self.lblwifi, self.lblBattery,self.comm_node)
            except Exception:
                pass
            if hasattr(self, 'timer') and self.timer:
                self.timer.stop()
                self.timer.deleteLater()
                self.timer = None
        except Exception as e:
            print(f"Error in MainWindow closeEvent: {e}")
        super().closeEvent(event)

    def open_fourbins_window(self, event):
        """
        响应主页点击，切换到四仓界面。通过 WindowManager 调用。

        注意：不在此处主动隐藏主页窗口，统一交由 WindowManager
        处理隐藏逻辑。这样可以保证新界面已经显示并全屏后再
        隐藏旧界面，避免短暂暴露其他应用界面。
        """
        try:
            if self.manager:
                self.manager.show_fourbins_window()
        except Exception as e:
            print(f"Error in open_fourbins_window: {e}")
            QtWidgets.QMessageBox.critical(self, "错误", f"打开四仓窗口失败: {str(e)}")

# ----------- 窗口管理器 -------------
class WindowManager:
    """窗口管理器，统一管理所有窗口"""
    def __init__(self,comm_node):
        self.main_window = None
        self.fourbins_window = None
        self.send_window = None
        self.face_window = None
        self.comm_node = comm_node

    def cleanup_window(self, window):
        """
        清理窗口资源。停止窗口计时器并关闭窗口。用于当窗口
        不再需要时释放其资源。仅在需要完全销毁窗口对象时使用。
        """
        if window:
            try:
                # 停止窗口内部计时器（若存在）
                if hasattr(window, 'timer') and window.timer:
                    window.timer.stop()
                    window.timer.deleteLater()
                window.close()
                window.deleteLater()
            except Exception:
                pass

    def show_main_window(self):
        """
        显示主页界面。如果不存在则创建。使用单次显示操作避免重复渲染。
        """
        if not self.main_window:
            self.main_window = MainWindow(manager=self,comm_node=self.comm_node)
        # 先隐藏其他窗口
        self._hide_others(exclude=self.main_window)
        # 一次性显示主窗口
        self.main_window.showFullScreen()


    def show_fourbins_window(self):
        """
        显示四仓界面。如果不存在则创建。使用单次显示操作避免重复渲染。
        """
        if not self.fourbins_window:
            self.fourbins_window = FourbinsWindow(manager=self,comm_node=self.comm_node)
        # 先隐藏其他窗口
        self._hide_others(exclude=self.fourbins_window)
        # 一次性显示四仓窗口
        self.fourbins_window.showFullScreen()

    def show_send_window(self, bin_index: int):
        """
        显示送货地址选择窗口。每次打开时创建新窗口并注册地址确认回调。
        使用单次显示操作避免重复渲染。
        """
        # 关闭旧的送货窗口
        if self.send_window:
            self.cleanup_window(self.send_window)
        # 创建新的送货窗口
        self.send_window = SendWindow(manager=self, comm_node=self.comm_node, bin_index=bin_index)
        # 连接地址确认信号到管理器的处理方法
        self.send_window.addressConfirmed.connect(
            lambda addr, idx=bin_index: self.handle_send_confirm(idx, addr)
        )
        # 先隐藏其他窗口
        self._hide_others(exclude=self.send_window)
        # 一次性显示送货窗口
        self.send_window.showFullScreen()

    def handle_send_confirm(self, bin_index: int, address: str):
        """
        处理送货窗口确认后的回调。更新四仓状态并返回四仓界面。
        """
        # 更新四仓窗口的货仓状态
        if self.fourbins_window:
            try:
                self.fourbins_window.on_address_confirmed(bin_index, address)
            except Exception as e:
                print(f"Error updating bin status: {e}")
        # 直接显示四仓界面，隐藏送货窗口的逻辑交由 show_fourbins_window
        # 中的 _hide_others 统一处理，以避免闪烁
        self.show_fourbins_window()

    def show_face_window(self):
        """
        显示人脸识别界面。如果不存在则创建。使用单次显示操作避免重复渲染。
        """
        if not self.face_window:
            self.face_window = FaceWindow(manager=self, comm_node=self.comm_node)
        # 先隐藏其他窗口
        self._hide_others(exclude=self.face_window)
        # 一次性显示人脸识别界面
        self.face_window.showFullScreen()

    def _hide_others(self, exclude):
        """
        隐藏除指定窗口之外的其它窗口。用于在新窗口已经显示后
        延迟隐藏旧窗口，避免界面切换时暴露桌面。
        """
        for w in [self.main_window, self.fourbins_window, self.send_window, self.face_window]:
            if w and w is not exclude:
                try:
                    w.hide()
                except Exception:
                    pass

# 在单独线程中运行rospy.spin
def ros_spin():
    rospy.spin()

def main(args=None):
    app = QtWidgets.QApplication(sys.argv)

    rospy.init_node("ui_node")
    node = UiNode()
    rospy.loginfo("Waiting for UI_get service...")
    node.ui_get_client.wait_for_service()
    rospy.loginfo("UI_get service is available.")

    window_manager = WindowManager(comm_node=node)
    window_manager.show_main_window()

     # 启动ROS spin线程
    thread_spin = threading.Thread(target=ros_spin)
    thread_spin.daemon = True
    thread_spin.start()
    sys.exit(app.exec_())

# ----------- 入口 -------------
if __name__ == "__main__":
    main()



