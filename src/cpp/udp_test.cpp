// UDP port test utility - diagnose serial-to-Ethernet bridge configuration

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdint.h>

#pragma comment(lib, "ws2_32.lib")

int main(int argc, char* argv[]) {
    SetConsoleOutputCP(CP_UTF8);
    
    if (argc < 3) {
        printf("Usage: udp_test.exe <ip> <port>\n");
        printf("Example: udp_test.exe 192.168.1.50 8899\n");
        printf("\nTrying common ports:\n");
        
        const char* ip = "192.168.1.50";
        int ports[] = {8899, 4001, 23, 1234, 9999, 10001, 50000, 5000, 6000};
        
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
        
        SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
        
        for (int i = 0; i < sizeof(ports)/sizeof(ports[0]); i++) {
            sockaddr_in addr = {0};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(ports[i]);
            addr.sin_addr.s_addr = inet_addr(ip);
            
            uint8_t test_data[10] = {0xA9, 0x5B, 0x02, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00};
            
            int sent = sendto(sock, (char*)test_data, 10, 0, (sockaddr*)&addr, sizeof(addr));
            if (sent > 0) {
                printf("✅ 端口 %d: 发送成功 (%d字节)\n", ports[i], sent);
            } else {
                printf("❌ 端口 %d: 发送失败 (错误: %d)\n", ports[i], WSAGetLastError());
            }
            Sleep(100);
        }
        
        closesocket(sock);
        WSACleanup();
        return 0;
    }
    
    const char* ip = argv[1];
    int port = atoi(argv[2]);
    
    printf("测试UDP发送到 %s:%d\n", ip, port);
    
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2), &wsa);
    
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);
    
    // 发送测试数据
    uint8_t test_data[40] = {
        0xA9, 0x5B, 0x02, 0x00,  // 启动命令头
        0x08, 0x00, 0x00,  // Roll
        0x08, 0x00, 0x00,  // Pitch
        0x08, 0x00, 0x00   // Yaw
    };
    
    for (int i = 0; i < 10; i++) {
        int sent = sendto(sock, (char*)test_data, 40, 0, (sockaddr*)&addr, sizeof(addr));
        if (sent > 0) {
            printf("[%d] ✅ 发送成功: %d字节\n", i+1, sent);
        } else {
            printf("[%d] ❌ 发送失败: 错误代码 %d\n", i+1, WSAGetLastError());
        }
        Sleep(500);
    }
    
    closesocket(sock);
    WSACleanup();
    return 0;
}

