#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import threading
import time

class TCPServer:
    def __init__(self, host='0.0.0.0', port=5001):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        
    def start_server(self):
        """启动TCP服务器"""
        try:
            # 创建socket对象
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 设置端口重用
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 绑定IP和端口
            self.server_socket.bind((self.host, self.port))
            
            # 开始监听，最大连接数为5
            self.server_socket.listen(5)
            self.running = True
            
            print(f"TCP服务器已启动，监听地址: {self.host}:{self.port}")
            print("等待客户端连接...")
            
            while self.running:
                try:
                    # 接受客户端连接
                    client_socket, client_address = self.server_socket.accept()
                    print(f"客户端已连接: {client_address}")
                    
                    # 为每个客户端创建新线程
                    client_thread = threading.Thread(
                        target=self.handle_client, 
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.error as e:
                    if self.running:
                        print(f"服务器错误: {e}")
                    break
                    
        except Exception as e:
            print(f"启动服务器失败: {e}")
        finally:
            self.stop_server()
    
    def handle_client(self, client_socket, client_address):
        """处理客户端连接"""
        try:
            # 发送消息给客户端
            message = "bigDoorOpen"
            client_socket.send(message.encode('utf-8'))
            print(f"已向 {client_address} 发送消息: {message}")
            
            # 接收客户端返回的消息
            try:
                # 设置接收超时时间为30秒
                client_socket.settimeout(30.0)
                response = client_socket.recv(1024)
                if response:
                    response_str = response.decode('utf-8')
                    print(f"收到客户端 {client_address} 的回复: {response_str}")
                else:
                    print(f"客户端 {client_address} 断开连接")
            except socket.timeout:
                print(f"等待客户端 {client_address} 回复超时")
            except Exception as recv_e:
                print(f"接收客户端 {client_address} 消息失败: {recv_e}")
                
        except socket.error as e:
            print(f"客户端 {client_address} 连接断开: {e}")
        finally:
            client_socket.close()
            print(f"客户端 {client_address} 连接已关闭")
    
    def stop_server(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("服务器已停止")

def main():
    # 创建TCP服务器实例
    server = TCPServer(host='0.0.0.0', port=5001)
    
    try:
        # 启动服务器
        server.start_server()
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止服务器...")
        server.stop_server()

if __name__ == "__main__":
    main()
