#pragma once
#include <string>

class TcpServer {
public:
    TcpServer(int port,std::string allowed_ip);
    ~TcpServer();
    bool start();
    bool acceptClient();
    bool sendMessage(const std::string& msg);
    std::string receiveMessage(int timeout_sec = 3);
    void closeSocket();

    int timeout_door_open = 90;
    int timeout_delivery_cmd = 60;

private:
    int server_port_;
    int server_fd_;
    int client_fd_;

    // 只允许特定IP连接
    std::string allowed_ip_;
    
};