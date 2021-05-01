#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const char *WIFI_SSID = "myssid";
const char *WIFI_PASSWORD = "mypassword";
const char *TARGET_IP = "192.168.0.255";
const int TARGET_PORT = 3333;
boolean connected = false;
WiFiUDP udp;

Adafruit_MPU6050 mpu;

Adafruit_BMP280 bmp;
const float SEALEVELHPA = 1013.25;

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
        udp.begin(WiFi.localIP(), TARGET_PORT);
        connected = true;
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        connected = false;
        digitalWrite(LED_BUILTIN, LOW);
        break;
    default:
        break;
    }
}

void connectToWiFi(const char *ssid, const char *pwd)
{
    Serial.println("Connecting to WiFi network: " + String(ssid));

    WiFi.disconnect(true);
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(ssid, pwd);

    Serial.println("Waiting for WIFI connection...");
}

void setup_mpu()
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        error_loop();
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange())
    {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth())
    {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println("");
}

void print_mpu()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println();
}

void setup_bmp()
{
    if (!bmp.begin(0x76))
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!"));
        error_loop();
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void print_bmp()
{
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(SEALEVELHPA)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
}

void loop()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    const float bmp_tmp = bmp.readTemperature();
    const float bmp_alt = bmp.readAltitude(SEALEVELHPA);
    const float bmp_pre = bmp.readPressure();

    if (connected)
    {
        //print_mpu();
        //print_bmp();

        udp.beginPacket(TARGET_IP, TARGET_PORT);
        udp.printf("%lu %.2f %.3f %f %.3f %.3f %.3f %.2f %.2f %.2f %.2f",
            millis(),
            bmp_tmp, bmp_alt, bmp_pre,
            a.acceleration.x, a.acceleration.x, a.acceleration.z,
            g.gyro.x, g.gyro.y, g.gyro.z,
            temp.temperature);
        udp.endPacket();
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial)
        delay(10);

    setup_mpu();
    setup_bmp();

    connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
}