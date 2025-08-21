from PyQt5 import QtCore, QtGui

# 创建一个全局的控制器实例
_controller = None

def register(lblTime, lblDate, lblSignal, lblwifi, lblBattery, comm_node):
    global _controller
    if _controller is None:
        _controller = StatusBarController(comm_node=comm_node)
    _controller.register(lblTime, lblDate, lblSignal, lblwifi, lblBattery)

def unregister(lblTime, lblDate, lblSignal, lblwifi, lblBattery):
    """
    Remove a set of status bar widgets from the global controller.  If the
    controller hasn't been created yet, this is a no‑op.  See update_all for
    details on how the controller stores each group of widgets.  When a window
    containing these labels is closed, you should call this function to
    unregister its labels; otherwise the timer will continue to update deleted
    QLabel objects, leading to a `wrapped C/C++ object of type QLabel has
    been deleted` error.
    """
    global _controller
    if _controller is None:
        return
    _controller.unregister(lblTime, lblDate, lblSignal, lblwifi, lblBattery)

def refresh(lblTime, lblDate, lblSignal, lblwifi, lblBattery, comm_node):
    global _controller
    if _controller is None:
        _controller = StatusBarController(comm_node=comm_node)
    _controller.update_all()

def set_charging(is_charging, comm_node):
    global _controller
    if _controller is None:
        _controller = StatusBarController(comm_node=comm_node)
    _controller.set_charging(is_charging)

class StatusBarController(QtCore.QObject):
    def __init__(self, comm_node, parent=None):
        super().__init__(parent)
        self.status_bars = []
        self.charging = False  # 全局充电状态
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_all)
        self.timer.start(1000)  # 1秒刷新一次
        self.comm_node = comm_node
        self.battery_status = 0

        self.comm_node.signal_recv_msg.connect(self.set_recv_msgs)

    def set_recv_msgs(self, battery_status):
        """
        接收来自comm_node的消息，更新电池状态。
        :param battery_status: 电池状态值
        """
        self.battery_status = battery_status

    def register(self, lblTime, lblDate, lblSignal, lblwifi, lblBattery):
        # 允许lblDate为None（如果没有日期标签）
        self.status_bars.append((lblTime, lblDate, lblSignal, lblwifi, lblBattery))

    def unregister(self, lblTime, lblDate, lblSignal, lblwifi, lblBattery):
        """
        Remove a previously registered group of labels from the internal list.
        If the tuple isn't found, nothing happens.  This method prevents the
        update timer from operating on Qt widgets that have already been
        destroyed.
        """
        try:
            self.status_bars.remove((lblTime, lblDate, lblSignal, lblwifi, lblBattery))
        except ValueError:
            pass

    def set_charging(self, is_charging):
        self.charging = is_charging
        self.update_all()

    def update_all(self):
        import datetime, random
        # sip.isdeleted allows us to check whether a Qt object has been deleted.
        try:
            import sip  # type: ignore
        except ImportError:
            sip = None
        now = datetime.datetime.now()
        week_map = ['一', '二', '三', '四', '五', '六', '日']
        date_str = f'{now.year}年{now.month}月{now.day}日 星期{week_map[now.weekday()]}'
        signal_icons = [
            ":/icons/icons/Signal_no.png",
            ":/icons/icons/Signal_1.png",
            ":/icons/icons/Signal_2.png",
            ":/icons/icons/Signal_3.png",
            ":/icons/icons/Signal_Full.png"
        ]
        wifi_icons = [
            ":/icons/icons/wifi_no.png",
            ":/icons/icons/wifi_1.png",
            ":/icons/icons/wifi_2.png",
            ":/icons/icons/wifi_3.png",
            ":/icons/icons/wifi_full.png"
        ]
        battery_icons = [
            ":/icons/icons/battery_25.png",
            ":/icons/icons/battery_50.png",
            ":/icons/icons/battery_full.png",
            ":/icons/icons/battery_charging_full.png"
        ]
        # 充电状态处理
        if self.charging:
            bat = battery_icons[-1]  # 显示充电图标
        else:
            if self.battery_status <= 25:
                bat = battery_icons[0]  # 低电量
            elif self.battery_status <= 50:
                bat = battery_icons[1]  # 中等电量
            elif self.battery_status > 50:  # 高电量
                bat = battery_icons[2]  # 高电量
            else:
                bat = battery_icons[3]  # 默认充电图标

        sig = random.choice(signal_icons)
        wifi = random.choice(wifi_icons)

        # 更新每一组状态栏控件。若某个控件已经被删除，则跳过该组。
        valid_bars = []
        for lblTime, lblDate, lblSignal, lblwifi, lblBattery in list(self.status_bars):
            # 如果sip模块可用且任一控件已经被删除，则跳过该组
            if sip is not None:
                try:
                    deleted = sip.isdeleted(lblTime) or \
                              (lblDate is not None and sip.isdeleted(lblDate)) or \
                              sip.isdeleted(lblSignal) or \
                              sip.isdeleted(lblwifi) or \
                              sip.isdeleted(lblBattery)
                    if deleted:
                        continue
                except Exception:
                    pass
            try:
                lblTime.setText(now.strftime("%H:%M"))
                if lblDate is not None:
                    lblDate.setText(f'<span style="font-size:10pt;">{date_str}</span>')
                lblSignal.setPixmap(QtGui.QPixmap(sig))
                lblwifi.setPixmap(QtGui.QPixmap(wifi))
                lblBattery.setPixmap(QtGui.QPixmap(bat))
                valid_bars.append((lblTime, lblDate, lblSignal, lblwifi, lblBattery))
            except RuntimeError:
                # 控件可能已经被删除，忽略
                continue
        # 更新内部列表，移除已经被删除的条目
        self.status_bars = valid_bars
