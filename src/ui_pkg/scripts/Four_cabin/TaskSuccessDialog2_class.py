from PyQt5 import QtWidgets, QtCore, QtGui
from ui_pkg.scripts.Four_cabin.TaskSuccessDialog2 import Ui_Dialog

class TaskSuccessDialog(QtWidgets.QDialog, Ui_Dialog):
    def __init__(self, comm_node = None, addresses=None, bin_numbers=None, delivery_addresses=None, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.comm_node = comm_node  # 保存通信节点引用
        self.bin_numbers = bin_numbers  # 保存箱号列表
        self.delivery_addresses = delivery_addresses  # 保存配送地址列表
        
        # 设置对话框大小为863*464，背景为白色，带16px圆角
        self.setFixedSize(863, 464)
        self.setStyleSheet("""
            QDialog {
                background-color: white;
                border-radius: 16px;
            }
        """)
        
        # 创建半透明黑色背景遮罩
        self.overlay = QtWidgets.QWidget(parent)
        if parent:
            self.overlay.setGeometry(parent.rect())
            self.overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")
            self.overlay.hide()
        
        # 保存父窗口引用，用于在showEvent中居中显示
        self.parent_window = parent
        
        # 设置对话框标题和提示文本
        self.label_2.setText("任务创建成功")
        self.label_3.setText("包裹已成功装入货仓")
        self.label_4.setText("配送地址：")
        
        # 设置地址文本，使用顿号连接所有地址
        if addresses and len(addresses) > 0:
            # 过滤掉空地址和占位符地址
            valid_addresses = [addr for addr in addresses if addr and addr != "--"]
            if valid_addresses:
                # 调整字体大小和行间距，确保完整显示
                address_text = "、".join(valid_addresses)
                
                # 设置样式，与label_4保持一致
                style = "QLabel {\n" \
                        "    color: rgb(0, 0, 0);\n" \
                        "    font-size: 20px;\n" \
                        "    font-weight: normal;\n" \
                        "    font-family: 'Microsoft YaHei';\n" \
                        "    background: transparent;\n" \
                        "}"
                self.label_5.setStyleSheet(style)
                
                # 设置地址文本
                self.label_5.setText(address_text)
                
                # 根据地址数量和长度调整标签大小和位置
                # 扩大标签尺寸以适应多行文本，并确保右侧有间距
                self.label_5.setGeometry(QtCore.QRect(461, 190, 352, 80)) # 调整y坐标以对齐顶部，并增加高度
                self.label_5.setWordWrap(True)  # 允许文本换行
                self.label_5.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

                # 如果地址非常多，进一步调整对话框布局 (保持原有逻辑，但位置已调整)
                if len(valid_addresses) > 3:
                    # 调整按钮位置，向下移动
                    self.pushButton.setGeometry(QtCore.QRect(190, 330, 200, 60))
                    self.pushButton_2.setGeometry(QtCore.QRect(470, 330, 200, 60))
            else:
                self.label_5.setText("无有效地址")
        else:
            self.label_5.setText("无地址信息")
        
        # 设置按钮文本
        self.pushButton.setText("取消")
        self.pushButton_2.setText("确认")
        
        # 连接按钮信号，并确保在关闭对话框时隐藏背景遮罩
        self.pushButton.clicked.connect(self.on_cancel)  # 取消按钮
        self.pushButton_2.clicked.connect(self.on_confirm)  # 确认按钮
        
    def showEvent(self, event):
        """在对话框显示时调整位置，并显示背景遮罩"""
        # 先显示背景遮罩
        if self.parent_window and hasattr(self, 'overlay'):
            self.overlay.show()
            # 确保遮罩大小与父窗口一致
            self.overlay.setGeometry(self.parent_window.rect())
            # 将遮罩提升到顶层
            self.overlay.raise_()
        
        super().showEvent(event)
        
        # 直接设置对话框的固定位置
        self.move(78, 72)  # 使用固定坐标而不是动态计算
        # 将对话框提升到遮罩之上
        self.raise_()
            
    def closeEvent(self, event):
        """在对话框关闭时隐藏背景遮罩"""
        # 隐藏背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
        super().closeEvent(event)
        
    def on_cancel(self):
        """取消按钮点击处理"""
        # 隐藏背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
        self.reject()
        
    def on_confirm(self):
        """确认按钮点击处理"""

        resp = self.comm_node.call_ui_get(self.bin_numbers, self.delivery_addresses)
        if resp and resp.received:
            print("配送信息已发布:", self.bin_numbers, self.delivery_addresses)
        else:
            print("配送服务调用失败！")

        # 隐藏背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
        self.accept()
        
    def hideEvent(self, event):
        """在对话框隐藏时隐藏背景遮罩"""
        # 隐藏背景遮罩
        if hasattr(self, 'overlay'):
            self.overlay.hide()
        super().hideEvent(event)