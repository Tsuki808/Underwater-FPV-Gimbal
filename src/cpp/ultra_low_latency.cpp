
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdint.h>

#pragma comment(lib, "ws2_32.lib")

// ==================== Configuration ====================
// Serial-to-Ethernet bridge configuration (adjust as needed)
// NOTE: Replace with the actual IP/port of your own serial server.
#define SERIAL_BRIDGE_IP "192.168.1.50" // IP address of the serial-to-Ethernet bridge
#define SERIAL_BRIDGE_PORT 4019         // TCP port

// OpenTrack UDP configuration
#define OPENTRACK_PORT 4242

// Gimbal work mode
#define WORK_MODE 0 // 0=Follow, 1=Lock, 2=FPV - Follow responds faster

// ==================== CRC16 calculation ====================
// CRC16 lookup table (precomputed)
const uint16_t CRC_TABLE[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef};

uint16_t calc_crc16(uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        uint8_t da = (crc >> 12) & 0x0F;
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (data[i] >> 4)];
        da = (crc >> 12) & 0x0F;
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (data[i] & 0x0F)];
    }
    return crc;
}

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    // Initialize Winsock
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    // UDP Socket - receive OpenTrack data
    SOCKET udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in udp_addr = {0};
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(OPENTRACK_PORT);
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    bind(udp_sock, (sockaddr *)&udp_addr, sizeof(udp_addr));
    printf(" Listening on UDP port %d (OpenTrack input)\n", OPENTRACK_PORT);

    // TCP Socket - connect to the serial-to-Ethernet bridge
    printf("\nConnecting to the serial-to-Ethernet bridge %s:%d ...\n", SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT);

    SOCKET tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sock == INVALID_SOCKET)
    {
        printf(" Failed to create TCP socket! Error: %d\n", WSAGetLastError());
        return 1;
    }

    sockaddr_in tcp_addr = {0};
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(SERIAL_BRIDGE_PORT);
    tcp_addr.sin_addr.s_addr = inet_addr(SERIAL_BRIDGE_IP);

    if (connect(tcp_sock, (sockaddr *)&tcp_addr, sizeof(tcp_addr)) == SOCKET_ERROR)
    {
        int err = WSAGetLastError();
        printf(" Connection failed! Error: %d\n", err);
        printf("\nPlease verify:\n");
        printf("  1. Device IP: %s (ping reachable)\n", SERIAL_BRIDGE_IP);
        printf("  2. Device port: %d\n", SERIAL_BRIDGE_PORT);
        printf("  3. Device TCP server is running\n");
        closesocket(tcp_sock);
        return 1;
    }

    printf(" Connected to %s:%d (TCP mode)\n", SERIAL_BRIDGE_IP, SERIAL_BRIDGE_PORT);
    printf("   Serial parameters: 115200 bps, 8N1\n");

    // Control byte: reserved[3 bits] + go_zero[1 bit] + wk_mode[2 bits] + op_type[2 bits]
    // wk_mode: 0=Follow, 1=Lock, 2=FPV
    // op_type: 0=angle control, 1=scaled angular velocity, 2=absolute angular velocity
    uint8_t control_byte = (0b000 << 5) | (0b0 << 4) | (WORK_MODE << 2) | 0;
    const char *mode_names[] = {"Follow", "Lock", "FPV"};
    printf("   Gimbal control mode: %s + angle control (control_byte=0x%02X)\n",
           mode_names[WORK_MODE], control_byte);

    // Data packet buffer (pre-allocated)
    uint8_t packet[40] = {
        0xA9, 0x5B,                            // sync[2]: sync header
        (0b000 << 5) | 4,                      // cmd: trig[3 bits] + valu[5 bits], cmd=4 (manual control)
        0x00,                                  // aux: reserved[3 bits] + fl_sens[5 bits]
        control_byte, 0, 0,                    // Roll: control_byte + op_valu[2 bytes]
        control_byte, 0, 0,                    // Pitch: control_byte + op_valu[2 bytes]
        control_byte, 0, 0,                    // Yaw: control_byte + op_valu[2 bytes]
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // uav (13 bytes)
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0     // cam (12 bytes)
    };

    // Send startup command (cmd=2)
    uint8_t startup_packet[40] = {0};
    startup_packet[0] = 0xA9;
    startup_packet[1] = 0x5B;
    startup_packet[2] = (0b000 << 5) | 2; // cmd=2 to start the gimbal, trig[3 bits]=000 + valu[5 bits]=2
    startup_packet[3] = 0x00;             // aux byte
    startup_packet[4] = control_byte;     // Roll control byte
    startup_packet[7] = control_byte;     // Pitch control byte
    startup_packet[10] = control_byte;    // Yaw control byte
    uint16_t startup_crc = calc_crc16(startup_packet, 38);
    startup_packet[38] = startup_crc >> 8;
    startup_packet[39] = startup_crc & 0xFF;

    int sent = send(tcp_sock, (char *)startup_packet, 40, 0);
    if (sent == SOCKET_ERROR)
    {
        printf("Failed to send startup command! Error: %d\n", WSAGetLastError());
    }
    else
    {
        printf("\n>>> Sent gimbal startup command (cmd=2), %d bytes\n", sent);
        printf("    Startup packet: %02X %02X %02X %02X %02X %02X %02X ...\n",
               startup_packet[0], startup_packet[1], startup_packet[2],
               startup_packet[3], startup_packet[4], startup_packet[5], startup_packet[6]);
    }
    Sleep(500);

    // Configure UDP receive timeout (avoid blocking)
    DWORD timeout = 100; // 100 ms timeout
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

    printf("Extreme mode started!\n");
    printf("\n=== Packet layout ===\n");
    printf("sync[2]: header 0xA9 0x5B\n");
    printf("cmd[1]:  command byte (trig[3 bits] + valu[5 bits])\n");
    printf("aux[1]:  auxiliary byte\n");
    printf("gbc[9]:  three-axis control (3 bytes per axis: control_byte + op_valu[2])\n");
    printf("         order: Roll[3] Pitch[3] Yaw[3]\n");
    printf("uav[13]: airframe data (currently zeros)\n");
    printf("cam[12]: camera data (currently zeros)\n");
    printf("crc[2]:  CRC16 (big endian)\n");
    printf("========================\n\n");

    uint8_t udp_buf[48];
    double last_yaw = 0, last_pitch = 0;
    LARGE_INTEGER freq, start, end;
    QueryPerformanceFrequency(&freq);
    int packet_count = 0;

    while (1)
    {
        QueryPerformanceCounter(&start);

        // Receive UDP (from OpenTrack)
        int len = recv(udp_sock, (char *)udp_buf, 48, 0);

        if (len == SOCKET_ERROR)
        {
            int err = WSAGetLastError();
            if (err != WSAETIMEDOUT)
            { // Timeouts are expected, so suppress logs
                printf("\nUDP recv failed with error: %d\n", err);
            }
            continue;
        }

        // Extra check for unexpected packet sizes
        if (len > 0 && len != 48)
        {
            printf("\nReceived an unexpected UDP size: %d bytes (expected 48)\n", len);
            continue;
        }

        if (len == 48)
        {
            // Extract only Yaw and Pitch
            double yaw = *(double *)(udp_buf + 24);
            double pitch = *(double *)(udp_buf + 32);

            // Skip duplicate samples
            if (yaw == last_yaw && pitch == last_pitch)
                continue;
            last_yaw = yaw;
            last_pitch = pitch;

            // Angle conversion (0.01-degree units)
            // Note: Pitch must be inverted (to match the Python implementation)
            int16_t roll_val = 0;
            int16_t pitch_val = (int16_t)(-pitch * 100); // Pitch inverted (coordinate-system correction)
            int16_t yaw_val = (int16_t)(yaw * 100);

            // Populate packet (little endian)
            // gbc[0]=Roll, gbc[1]=Pitch, gbc[2]=Yaw
            *(int16_t *)(packet + 5) = roll_val;  // Roll (offset 5)
            *(int16_t *)(packet + 8) = pitch_val; // Pitch (offset 8)
            *(int16_t *)(packet + 11) = yaw_val;  // Yaw (offset 11)

            // Compute CRC (big endian)
            uint16_t crc = calc_crc16(packet, 38);
            packet[38] = crc >> 8;
            packet[39] = crc & 0xFF;

            // Log first packet details
            if (packet_count == 0)
            {
                printf("\n>>> First control packet (cmd=4):\n");
                printf("    Full packet: ");
                for (int i = 0; i < 40; i++)
                {
                    printf("%02X ", packet[i]);
                    if (i == 1 || i == 3 || i == 12 || i == 25 || i == 37)
                        printf("| ");
                }
                printf("\n");
                printf("    Roll=%d, Pitch=%d, Yaw=%d (0.01-degree unit)\n", roll_val, pitch_val, yaw_val);
                printf("    CRC=0x%04X\n\n", crc);
            }
            packet_count++;

            // 发送到TCP Socket (串口转网口设备)
            int sent = send(tcp_sock, (char *)packet, 40, 0);
            if (sent == SOCKET_ERROR)
            {
                int err = WSAGetLastError();
                printf("\n TCP send failed! Error: %d\n", err);
                printf("   TCP connection may be lost, exiting\n");
                break;
            }

            QueryPerformanceCounter(&end);
            double ms = (end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
            printf("\rYaw:%6.2f° Pitch:%6.2f° | Roll=%d Pitch=%d Yaw=%d | Latency:%.2fms   ",
                   yaw, pitch, roll_val, pitch_val, yaw_val, ms);
        }
    }

    closesocket(tcp_sock);
    closesocket(udp_sock);
    WSACleanup();
    return 0;
}
