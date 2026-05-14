#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include <Wire.h>

#define DEVICE_ADDRESS 0x05
#define M1_MODE 0x44
#define M1_POWER 0x45
#define M2_POWER 0x46

// Пины I2C для ESP32 30-pin
#define I2C_SDA 21
#define I2C_SCL 22

// Пины для энкодеров
#define ENC1_A 18 // Левый энкодер (прерывание)
#define ENC1_B 19 // Левый энкодер (направление)
#define ENC2_A 4  // Правый энкодер (прерывание)
#define ENC2_B 5  // Правый энкодер (направление)

// Глобальные переменные энкодеров
volatile int64_t encoder1Count = 0;
volatile int64_t encoder2Count = 0;

bool deviceAvailable = false;

// --- Настройки сети ---
const char *ssid = "ANDROID_LAB_33";
const char *password = "02091991";

// Порты для обмена
const unsigned int localPort = 4210;    // Порт, который слушает ESP32
const char *remoteIp = "192.168.10.62"; // IP твоего компьютера с ROS2
const unsigned int remotePort = 4211;   // Порт, который слушает ROS2 узел

WiFiUDP udp;

// --- Структуры данных (бинарно совместимы с C++ на ПК) ---
#pragma pack(push, 1)
struct CommandPacket
{
    float linear_x;
    float angular_z;
};

struct StatePacket
{
    int64_t left_ticks;
    int64_t right_ticks;
};
#pragma pack(pop)

CommandPacket cmd_in = {0.0f, 0.0f};
StatePacket state_out = {0, 0};

// Таймер для отправки данных (например, 20 Гц)
unsigned long last_send_time = 0;
const unsigned long send_interval = 50;

// Прототипы функций
void initEncoders();
void resetEncoders();
void applyPID(unsigned long current_time);
bool checkDevice();
int sendMotorCommand(uint8_t reg, int percent);
void sendDualMotorCommand(int p1, int p2);
void scanI2CBus();
void stopMotors();
void checkConnection();
void receiveCommands();
void sendState();

//-------------------------------------------------------------------------------------------------
// ЭНКОДЕРЫ
//-------------------------------------------------------------------------------------------------
void IRAM_ATTR encoder1ISR()
{
    int b = digitalRead(ENC1_B);
    if (b == HIGH)
    {
        encoder1Count++;
    }
    else
    {
        encoder1Count--;
    }
}

void IRAM_ATTR encoder2ISR()
{
    int b = digitalRead(ENC2_B);
    if (b == HIGH)
    {
        encoder2Count--;
    }
    else
    {
        encoder2Count++;
    }
}

void initEncoders()
{
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

    Serial.println("✓ Энкодеры инициализированы");
}

void resetEncoders()
{
    encoder1Count = 0;
    encoder2Count = 0;
    Serial.println("Счетчики энкодеров сброшены!");
}

//-------------------------------------------------------------------------------------------------
// PID
//-------------------------------------------------------------------------------------------------
// Настройки регулятора
float kp = 2.0; // Пропорциональный коэффициент (подбирается)
float ki = 0.01; // Интегральный коэффициент (подбирается)

// Целевые и текущие значения
float target_speed_l = 0; // в тиках за интервал
float target_speed_r = 0;
double last_enc1 = 0;
double last_enc2 = 0;

// Ошибки для PID
double integral_l = 0;
double integral_r = 0;

void applyPID(unsigned long current_time) {
    unsigned long dt_ms = current_time - last_send_time;
    if (dt_ms == 0) return;
    float dt = dt_ms / 1000.0; // Переводим в секунды

    // 1. Текущая скорость в тиках в секунду
    double cur_enc1 = encoder1Count;
    double cur_enc2 = encoder2Count;

    double v_l = (cur_enc1 - last_enc1) / (dt * 29);
    double v_r = (cur_enc2 - last_enc2) / (dt * 29);

    last_enc1 = cur_enc1;
    last_enc2 = cur_enc2;

    // 2. Вычисляем ошибку (в тиках/сек)
    float error_l = target_speed_l - v_l;
    float error_r = target_speed_r - v_r;

    // 3. Интеграл с ограничением (Anti-windup)
    // Ограничиваем влияние I-составляющей на мощность (например, не более 30% мощности)
    integral_l = constrain(integral_l + error_l * dt, -100, 100);
    integral_r = constrain(integral_r + error_r * dt, -100, 100);

    // 4. Расчет мощности (-100...100)
    int power_l = (int)(error_l * kp + integral_l * ki);
    int power_r = (int)(error_r * kp + integral_r * ki);

    if (target_speed_l == 0 && target_speed_r == 0) {
        power_l = 0; power_r = 0;
        integral_l = 0; integral_r = 0;
    }

    // ЛОГГИРОВАНИЕ (для отладки в Serial Plotter)
    // Формат: Target, Real, Power
    //Serial.printf(">Target_L:%.2f\n>Real_L:%.2f\n>Power_L:%d\n", target_speed_l, v_l, power_l);
    // Serial.printf("PID M1: Target speed: %f, Command: %d Real speed: %f | E_C: %f, E_P: %f P: %f I: %f\n", target_speed_l,power_l, v_l, cur_enc1, last_enc1, error_l, integral_l * ki); 

    //sendDualMotorCommand(power_l, power_r); //ЕСЛИ ПИД
    sendDualMotorCommand(target_speed_l, target_speed_r); //Напрямую без ПИД 


}
//-------------------------------------------------------------------------------------------------
// I2C МОТОРЫ
//-------------------------------------------------------------------------------------------------
bool checkDevice()
{
    Wire.beginTransmission(DEVICE_ADDRESS);
    uint8_t error = Wire.endTransmission(true);

    if (error == 0)
    {
        if (!deviceAvailable)
        {
            Serial.println("✓ Мотор-контроллер обнаружен!");
            deviceAvailable = true;
        }
        return true;
    }
    else
    {
        if (deviceAvailable)
        {
            Serial.printf("✗ Мотор-контроллер потерян! Ошибка: %d\n", error);
            deviceAvailable = false;
        }
        return false;
    }
}

// Попытка отправить команду с разными форматами
int sendMotorCommand(uint8_t reg, int percent)
{
    if (!checkDevice())
    {
        return 0;
    }

    // Ограничиваем значение
    if (percent < -100)
        percent = -100;
    if (percent > 100)
        percent = 100;

    // Конвертируем процент в байт (0-100 для скорости)
    // Некоторые контроллеры ожидают 0-100, а не -100..100
    uint8_t speedValue;
    if (percent >= 0)
    {
        speedValue = percent; // 0-100 вперед
    }
    else
    {
        speedValue = 128 + abs(percent); // 128-228 назад (или другой формат)
    }

    // Пробуем отправить как есть (байт со знаком)
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(reg);
    Wire.write((int8_t)percent);
    uint8_t error = Wire.endTransmission(true);

    if (error != 0)
    {
        // Пробуем альтернативный формат (0-100)
        delay(10);
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(reg);
        Wire.write(speedValue);
        error = Wire.endTransmission(true);

        if (error != 0)
        {
            Serial.printf("Ошибка 0x%02X: %d (процент=%d, альт=%d)\n", reg, error, percent, speedValue);
            return 0;
        }
    }

    return 1;
}

void sendDualMotorCommand(int p1, int p2)
{
    if (!checkDevice()) return;

    // Ограничиваем диапазон
    p1 = constrain(p1, -100, 100);
    p2 = constrain(p2, -100, 100);

    // Просто приводим к знаковому байту. 
    // Если p1 = -50, m1_val будет содержать 0xCE, что и нужно драйверу.
    int8_t m1_val = (int8_t)p1; 

    int8_t m2_val = (int8_t)p2; 


    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(M1_MODE); 
    Wire.write(0x00); 
    Wire.write(m1_val);   
    Wire.write(m2_val);  
    Wire.write(0x00);  
    uint8_t error = Wire.endTransmission();

    //Serial.printf("Send speed: %d\n", m1_val);
    
    if (error != 0) Serial.printf("I2C Error: %d\n", error);
}

//-------------------------------------------------------------------------------------------------
// СКАНИРОВАНИЕ I2C
//-------------------------------------------------------------------------------------------------
void scanI2CBus()
{
    Serial.println("\n=== СКАНИРОВАНИЕ I2C ШИНЫ ===");
    int nDevices = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission(true);

        if (error == 0)
        {
            Serial.printf("Найдено устройство: 0x%02X\n", address);
            nDevices++;

            if (address == DEVICE_ADDRESS)
            {
                Serial.println("  >>> Это ваш мотор-контроллер!");

                // Пробуем прочитать что-нибудь с контроллера
                Wire.beginTransmission(DEVICE_ADDRESS);
                Wire.write(0x00);
                if (Wire.endTransmission(false) == 0)
                {
                    Wire.requestFrom(DEVICE_ADDRESS, (uint8_t)8, true);
                    if (Wire.available())
                    {
                        Serial.print("  Данные: ");
                        while (Wire.available())
                        {
                            Serial.print((char)Wire.read());
                        }
                        Serial.println();
                    }
                }
            }
        }
    }

    if (nDevices == 0)
    {
        Serial.println("Устройства не найдены!");
    }
    Serial.println("=============================\n");
}

void stopMotors()
{
    sendDualMotorCommand(0, 0);
    Serial.println("Моторы остановлены");
}

// Можно вызывать при потере связи
void checkConnection()
{
    static unsigned long lastRxTime = millis();
    static bool motorsStopped = false;

    if (udp.parsePacket() > 0)
    {
        lastRxTime = millis();
        motorsStopped = false;
    }

    // Если 1 секунду нет команд - останавливаем моторы
    if (millis() - lastRxTime > 1000 && !motorsStopped)
    {
        stopMotors();
        motorsStopped = true;
        Serial.println("WARNING: No commands received, motors stopped!");
    }
}

void setup()
{
    Serial.begin(115200);

    // Подключение к WiFi
    Serial.print("Connecting to WiFi....");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

    // Запуск UDP
    udp.begin(localPort);

    // Инициализация энкодеров
    initEncoders();
    resetEncoders();

    // Инициализация I2C
    Serial.printf("Инициализация I2C: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(10000);
    delay(100);

    // Сканируем I2C шину
    scanI2CBus();

    // Проверяем мотор-контроллер
    if (checkDevice())
    {
        Serial.println("\n Мотор-контроллер готов к работе!\n");
    }

    delay(2000);
}

void receiveCommands()
{
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(CommandPacket))
    {
        udp.read((char *)&cmd_in, sizeof(CommandPacket));

        // Коэффициент перевода из м/с в проценты мощности драйвера
        // Подбери его экспериментально (например, если макс скорость 0.5 м/с, то k=200)
        float k = 1.0;

        target_speed_l = (cmd_in.linear_x - cmd_in.angular_z) * k;
        target_speed_r = (cmd_in.linear_x + cmd_in.angular_z) * k;

        // Serial.printf("M1: %d, M2: %d\n", target_speed_l, target_speed_r);
    }
}

void sendState()
{

    state_out.left_ticks = encoder1Count;  // левый энкодер
    state_out.right_ticks = encoder2Count; // правый энкодер

    udp.beginPacket(remoteIp, remotePort);
    udp.write((const uint8_t *)&state_out, sizeof(StatePacket));
    udp.endPacket();
}

void loop()
{
    // 1. Пем команд от ROe22
    receiveCommands();
    // checkConnection();

    // 2. Отправка состояния в ROS2 по таймеру
    unsigned long current_time = millis();
    if (current_time - last_send_time >= send_interval)
    {
        // Сначала считаем PID и рулим моторами
        applyPID(current_time);
        // Затем отправляем данные в ROS
        sendState();
        last_send_time = current_time;
    }

    // 3. Тут может быть твой регулятор скорости (PID)
}
