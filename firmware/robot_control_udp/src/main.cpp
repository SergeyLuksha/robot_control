#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdint.h>

// --- Настройки сети ---
const char* ssid = "ANDROID_LAB_33";
const char* password = "02091991";

// Порты для обмена
const unsigned int localPort = 4210;      // Порт, который слушает ESP32
const char* remoteIp = "192.168.10.62";   // IP твоего компьютера с ROS2
const unsigned int remotePort = 4211;     // Порт, который слушает ROS2 узел

WiFiUDP udp;

// --- Структуры данных (бинарно совместимы с C++ на ПК) ---
#pragma pack(push, 1)
struct CommandPacket {
    float linear_x;
    float angular_z;
};

struct StatePacket {
    int64_t left_ticks;
    int64_t right_ticks;
};
#pragma pack(pop)

CommandPacket cmd_in = {0.0f, 0.0f};
StatePacket state_out = {0, 0};

// Таймер для отправки данных (например, 20 Гц)
unsigned long last_send_time = 0;
const unsigned long send_interval = 50; 

void setup() {
    Serial.begin(115200);

    // Подключение к WiFi
    Serial.print("Connecting to WiFi....");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

    // Запуск UDP
    udp.begin(localPort);
}

void receiveCommands() {
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(CommandPacket)) {
        udp.read((char*)&cmd_in, sizeof(CommandPacket));
        
        // Здесь будет логика управления моторами
        Serial.printf("RX: lin_x: %.2f, ang_z: %.2f\n", cmd_in.linear_x, cmd_in.angular_z);
    }
}

void sendState() {
    // В будущем здесь будет опрос реальных энкодеров
    // Для теста просто инкрементируем значения
    state_out.left_ticks++;
    state_out.right_ticks--;

    udp.beginPacket(remoteIp, remotePort);
    udp.write((const uint8_t*)&state_out, sizeof(StatePacket));
    udp.endPacket();
}

void loop() {
    // 1. Прием команд от ROS2
    receiveCommands();

    // 2. Отправка состояния в ROS2 по таймеру
    unsigned long current_time = millis();
    if (current_time - last_send_time >= send_interval) {
        sendState();
        last_send_time = current_time;
    }

    // 3. Тут может быть твой регулятор скорости (PID)
}