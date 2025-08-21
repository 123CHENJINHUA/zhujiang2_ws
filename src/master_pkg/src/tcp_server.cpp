#include "master_pkg/tcp_server.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

TcpServer::TcpServer(int port,std::string allowed_ip)
    : server_port_(port), server_fd_(-1), client_fd_(-1),allowed_ip_(allowed_ip) {}

TcpServer::~TcpServer() {
    closeSocket();
}

bool TcpServer::start() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) return false;

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); // 允许端口复用

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(server_port_);
    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) return false;
    if (listen(server_fd_, 1) < 0) return false;
    return true;
}

bool TcpServer::acceptClient() {
    sockaddr_in client_addr;
    socklen_t addrlen = sizeof(client_addr);
    while (true) {
        int fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addrlen);
        if (fd < 0) return false;

        // 获取客户端IP
        char ipstr[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &client_addr.sin_addr, ipstr, sizeof(ipstr));
        std::string client_ip(ipstr);

        if (client_ip == allowed_ip_) {
            client_fd_ = fd;
            return true;
        } else {
            close(fd); // 关闭不允许的连接
        }
    }
}

bool TcpServer::sendMessage(const std::string& msg) {
    if (client_fd_ < 0) return false;
    ssize_t sent = send(client_fd_, msg.c_str(), msg.size(), 0);
    return sent == (ssize_t)msg.size();
}

std::string TcpServer::receiveMessage(int timeout_sec) {
    char buffer[1024] = {0};
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(client_fd_, &readfds);
    struct timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    int ret = select(client_fd_ + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(client_fd_, &readfds)) {
        ssize_t len = recv(client_fd_, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            return std::string(buffer, len);
        }
    }
    return "";
}

void TcpServer::closeSocket() {
    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
    }
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}