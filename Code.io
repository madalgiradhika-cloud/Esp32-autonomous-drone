#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <MPU9250.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

// ==================== HARDWARE PIN CONFIGURATION ====================
#define MOTOR_1_PIN 25
#define MOTOR_2_PIN 26
#define MOTOR_3_PIN 27
#define MOTOR_4_PIN 28

#define ULTRASONIC_TRIG 12
#define ULTRASONIC_ECHO 13
#define ULTRASONIC_TRIG2 32
#define ULTRASONIC_ECHO2 33

// LoRa Pins
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 4

// SIM800L Configuration
#define SIM800L_RX 16
#define SIM800L_TX 17
#define SIM800L_POWER_KEY 19

// GPS Configuration
#define GPS_RX 9
#define GPS_TX 10

// I2C Configuration
#define I2C_SDA 21
#define I2C_SCL 22

// ==================== SENSOR OBJECTS ====================
TinyGPSPlus gps;
Adafruit_BMP280 bmp280;
MPU9250 mpu9250;
SoftwareSerial simSerial(SIM800L_RX, SIM800L_TX);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// ==================== KALMAN FILTER IMPLEMENTATION ====================
class KalmanFilter {
private:
    float Q; // Process noise
    float R; // Measurement noise
    float P; // Estimation error
    float K; // Kalman gain
    float x; // Current state estimate

public:
    KalmanFilter(float q, float r, float p, float initValue) 
        : Q(q), R(r), P(p), x(initValue) {}

    float update(float measurement, float deltaTime) {
        // Predict
        P = P + Q;
        
        // Calculate Kalman gain
        K = P / (P + R);
        
        // Update
        x = x + K * (measurement - x);
        
        // Update error estimate
        P = (1 - K) * P;
        
        return x;
    }

    float getValue() { return x; }
};

// ==================== DRONE STATE STRUCTURE ====================
struct DroneState {
    // Position
    double currentLat, currentLng;
    double homeLat, homeLng;
    
    // Altitude & Barometric Data
    float currentAlt, targetAlt;
    float pressure, temperature;
    
    // IMU Data
    float rollAngle, pitchAngle, yawAngle;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    
    // Velocity
    float velNorth, velEast, velDown;
    
    // Motor Commands (PWM values)
    uint16_t motor1, motor2, motor3, motor4;
    
    // Status Flags
    bool homeSet;
    bool armed;
    bool isFlying;
    bool rthActive;
    bool waypointActive;
    
    // GPS Quality
    int satCount;
    float hdop;
};

DroneState droneState = {};

// ==================== KALMAN FILTERS ====================
KalmanFilter kalmanAlt(0.001, 0.1, 1.0, 0.0);
KalmanFilter kalmanLat(0.0001, 0.01, 1.0, 0.0);
KalmanFilter kalmanLng(0.0001, 0.01, 1.0, 0.0);
KalmanFilter kalmanRoll(0.001, 0.1, 1.0, 0.0);
KalmanFilter kalmanPitch(0.001, 0.1, 1.0, 0.0);
KalmanFilter kalmanYaw(0.001, 0.1, 1.0, 0.0);

// ==================== PID CONTROLLER ====================
class PIDController {
private:
    float Kp, Ki, Kd;
    float integral, prevError;
    float maxOutput, minOutput;
    unsigned long lastTime;

public:
    PIDController(float p, float i, float d, float minOut, float maxOut)
        : Kp(p), Ki(i), Kd(d), minOutput(minOut), maxOutput(maxOut), 
          integral(0), prevError(0), lastTime(0) {}

    float update(float setpoint, float measurement, unsigned long currentTime) {
        float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        float error = setpoint - measurement;
        integral += error * dt;
        integral = constrain(integral, minOutput / Ki, maxOutput / Ki);

        float derivative = (dt > 0) ? (error - prevError) / dt : 0;
        prevError = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        return constrain(output, minOutput, maxOutput);
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

// Altitude PID Controllers
PIDController altPID(0.5, 0.05, 0.1, 1200, 1800);
PIDController rollPID(1.0, 0.02, 0.08, -30, 30);
PIDController pitchPID(1.0, 0.02, 0.08, -30, 30);
PIDController yawPID(0.8, 0.01, 0.05, -100, 100);

// ==================== WAYPOINT NAVIGATION ====================
struct Waypoint {
    double lat, lng;
    float alt;
    float holdTime; // seconds
    uint8_t navMode; // 0=straightLine, 1=circle, 2=loiter
};

Waypoint waypoints[] = {
    {12.9716, 77.5946, 15.0, 5.0, 0},
    {12.9720, 77.5950, 20.0, 5.0, 0},
    {12.9715, 77.5955, 15.0, 5.0, 0},
};

int currentWaypoint = 0;
const int maxWaypoints = 3;

// ==================== OBSTACLE AVOIDANCE ====================
class ObstacleAvoidance {
private:
    float frontDist, leftDist, rightDist, bottomDist;
    const float DANGER_THRESHOLD = 1.5; // meters
    const float WARNING_THRESHOLD = 2.5; // meters

public:
    void update() {
        frontDist = getUltrasonicDistance(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
        bottomDist = getUltrasonicDistance(ULTRASONIC_TRIG2, ULTRASONIC_ECHO2);
    }

    bool isDanger() {
        return frontDist < DANGER_THRESHOLD || bottomDist < DANGER_THRESHOLD;
    }

    bool isWarning() {
        return frontDist < WARNING_THRESHOLD || bottomDist < WARNING_THRESHOLD;
    }

    float getAvoidanceYaw() {
        // Calculate yaw adjustment for obstacle avoidance
        if (frontDist < WARNING_THRESHOLD) {
            return (frontDist < DANGER_THRESHOLD) ? 90 : 45;
        }
        return 0;
    }

    float getFrontDistance() { return frontDist; }
    float getBottomDistance() { return bottomDist; }
};

ObstacleAvoidance obstacleAvoidance;

// ==================== LoRa TELEMETRY ====================
class LoraTelemetry {
private:
    unsigned long lastSendTime = 0;
    const unsigned long SEND_INTERVAL = 1000; // ms

public:
    void init() {
        LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
        if (!LoRa.begin(433E6)) {
            Serial.println("LoRa init failed!");
        } else {
            LoRa.setSpreadingFactor(12);
            LoRa.setSignalBandwidth(125000);
            LoRa.setCodingRate4(8);
            Serial.println("LoRa initialized successfully");
        }
    }

    void sendTelemetry(const DroneState& state) {
        if (millis() - lastSendTime < SEND_INTERVAL) return;
        lastSendTime = millis();

        LoRa.beginPacket();
        
        // Encode telemetry data
        char buffer[256];
        snprintf(buffer, sizeof(buffer),
            "$TELEM|LAT:%.6f|LNG:%.6f|ALT:%.2f|SAT:%d|HDOP:%.2f|"
            "ROLL:%.2f|PITCH:%.2f|YAW:%.2f|"
            "M1:%u|M2:%u|M3:%u|M4:%u|STATE:%d|RSSI:%d",
            state.currentLat, state.currentLng, state.currentAlt,
            state.satCount, state.hdop,
            state.rollAngle, state.pitchAngle, state.yawAngle,
            state.motor1, state.motor2, state.motor3, state.motor4,
            state.armed ? 1 : 0, LoRa.packetRssi());

        LoRa.write((uint8_t*)buffer, strlen(buffer));
        LoRa.endPacket();
    }

    void receiveCommands() {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            char command[128] = {0};
            int i = 0;
            while (LoRa.available() && i < sizeof(command) - 1) {
                command[i++] = (char)LoRa.read();
            }
            command[i] = '\0';
            
            processRemoteCommand(command);
        }
    }

private:
    void processRemoteCommand(const char* cmd) {
        if (strncmp(cmd, "ARM", 3) == 0) {
            droneState.armed = true;
        } else if (strncmp(cmd, "DISARM", 6) == 0) {
            droneState.armed = false;
        } else if (strncmp(cmd, "RTH", 3) == 0) {
            droneState.rthActive = true;
        }
    }
};

LoraTelemetry loraTelem;

// ==================== SIM800L TELEMETRY ====================
class SIM800LTelemetry {
private:
    unsigned long lastSendTime = 0;
    const unsigned long SEND_INTERVAL = 30000; // Send every 30 seconds
    bool simInitialized = false;

public:
    void init() {
        pinMode(SIM800L_POWER_KEY, OUTPUT);
        digitalWrite(SIM800L_POWER_KEY, LOW);
        
        simSerial.begin(9600);
        delay(2000);
        
        // Power on SIM800L
        digitalWrite(SIM800L_POWER_KEY, HIGH);
        delay(3000);
        digitalWrite(SIM800L_POWER_KEY, LOW);
        
        delay(5000); // Wait for module to initialize
        
        // Test AT command
        sendATCommand("AT", 1000);
        sendATCommand("AT+CMGF=1", 1000); // Set SMS text mode
        simInitialized = true;
    }

    void sendLocationViaGPRS(const DroneState& state) {
        if (millis() - lastSendTime < SEND_INTERVAL || !simInitialized) return;
        lastSendTime = millis();

        // Example: Send location via HTTP POST
        char url[256];
        snprintf(url, sizeof(url),
            "http://yourserver.com/location?"
            "lat=%.6f&lng=%.6f&alt=%.2f&sat=%d",
            state.currentLat, state.currentLng, state.currentAlt, state.satCount);

        // Send HTTP GET request (simplified)
        sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000);
        sendATCommand("AT+SAPBR=3,1,\"APN\",\"YOUR_APN\"", 1000);
        sendATCommand("AT+SAPBR=1,1", 5000);
        // Further HTTP commands...
    }

    void sendSMS(const char* phoneNumber, const DroneState& state) {
        if (!simInitialized) return;

        sendATCommand("AT+CMGF=1", 1000);
        
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phoneNumber);
        simSerial.println(cmd);
        delay(500);

        char message[256];
        snprintf(message, sizeof(message),
            "Lat:%.6f Lng:%.6f Alt:%.2fm Sat:%d",
            state.currentLat, state.currentLng, state.currentAlt, state.satCount);

        simSerial.println(message);
        simSerial.write(26); // Ctrl+Z
        delay(1000);
    }

private:
    void sendATCommand(const char* cmd, unsigned long timeout) {
        simSerial.println(cmd);
        unsigned long startTime = millis();
        while (millis() - startTime < timeout) {
            if (simSerial.available()) {
                Serial.write(simSerial.read());
            }
        }
        Serial.println();
    }
};

SIM800LTelemetry simTelem;

// ==================== SENSOR READING FUNCTIONS ====================
float getUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    float distance = duration * 0.034 / 2;
    return (distance > 0 && distance < 400) ? distance : 999.0; // cm to m, max 4m
}

void updateIMUData() {
    if (mpu9250.update()) {
        droneState.rollAngle = kalmanRoll.update(mpu9250.getRoll(), 0.01);
        droneState.pitchAngle = kalmanPitch.update(mpu9250.getPitch(), 0.01);
        droneState.yawAngle = kalmanYaw.update(mpu9250.getYaw(), 0.01);

        droneState.accelX = mpu9250.getAccelX_mss();
        droneState.accelY = mpu9250.getAccelY_mss();
        droneState.accelZ = mpu9250.getAccelZ_mss();

        droneState.gyroX = mpu9250.getGyroX_rads();
        droneState.gyroY = mpu9250.getGyroY_rads();
        droneState.gyroZ = mpu9250.getGyroZ_rads();

        droneState.magX = mpu9250.getMagX_uT();
        droneState.magY = mpu9250.getMagY_uT();
        droneState.magZ = mpu9250.getMagZ_uT();
    }
}

void updateBarometerData() {
    float measuredAlt = bmp280.readAltitude(1013.25); // Sea level pressure
    droneState.currentAlt = kalmanAlt.update(measuredAlt, 0.01);
    droneState.pressure = bmp280.readPressure();
    droneState.temperature = bmp280.readTemperature();
}

void updateGPSData() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        droneState.currentLat = kalmanLat.update(gps.location.lat(), 0.01);
        droneState.currentLng = kalmanLng.update(gps.location.lng(), 0.01);
        droneState.satCount = gps.satellites.value();
        droneState.hdop = gps.hdop.hdop();

        // Set home position on first GPS lock
        if (!droneState.homeSet && droneState.satCount >= 10) {
            droneState.homeLat = droneState.currentLat;
            droneState.homeLng = droneState.currentLng;
            droneState.homeSet = true;
            Serial.println("Home position set!");
        }
    }
}

// ==================== NAVIGATION FUNCTIONS ====================
float getDistanceBetween(double lat1, double lon1, double lat2, double lon2) {
    const float EARTH_RADIUS = 6371000.0; // meters

    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);

    float a = sin(dLat / 2.0) * sin(dLat / 2.0) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2.0) * sin(dLon / 2.0);

    float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS * c;
}

float getBearingTo(double lat1, double lon1, double lat2, double lon2) {
    float dLon = radians(lon2 - lon1);
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) -
              sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);

    float bearing = atan2(y, x);
    bearing = degrees(bearing);
    bearing = fmod((bearing + 360.0), 360.0); // Normalize to 0-360
    return bearing;
}

void navigateToWaypoint(const Waypoint& wp) {
    float distance = getDistanceBetween(droneState.currentLat, droneState.currentLng,
                                         wp.lat, wp.lng);
    float bearing = getBearingTo(droneState.currentLat, droneState.currentLng,
                                  wp.lat, wp.lng);

    // Calculate yaw error and adjust
    float yawError = bearing - droneState.yawAngle;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;

    // Proportional control for yaw
    float yawCommand = constrain(yawError * 0.5, -30, 30);

    // Pitch/Roll based on distance
    if (distance > 3.0) { // Move if more than 3m away
        float pitchCommand = constrain((distance / 50.0) * 30, -30, 30); // Max 30 deg pitch
        float rollCommand = 0; // Simplified: keep level

        // Update drone pitch/roll commands here
    }

    // Altitude control
    float altError = wp.alt - droneState.currentAlt;
    float throttleAdjust = altPID.update(wp.alt, droneState.currentAlt, millis());

    Serial.printf("WP: %.0fm away, bearing: %.1f°, alt error: %.1fm\n",
                  distance, bearing, altError);
}

void returnToHome() {
    if (!droneState.homeSet) {
        Serial.println("Home not set!");
        return;
    }

    navigateToWaypoint({droneState.homeLat, droneState.homeLng, 5.0, 0, 0});
}

// ==================== MOTOR CONTROL ====================
void setMotorSpeed(uint8_t motor, uint16_t pwm) {
    pwm = constrain(pwm, 1000, 2000); // Standard PWM range

    switch (motor) {
        case 1: droneState.motor1 = pwm; break;
        case 2: droneState.motor2 = pwm; break;
        case 3: droneState.motor3 = pwm; break;
        case 4: droneState.motor4 = pwm; break;
    }
}

void armDrone() {
    droneState.armed = true;
    setMotorSpeed(1, 1050);
    setMotorSpeed(2, 1050);
    setMotorSpeed(3, 1050);
    setMotorSpeed(4, 1050);
    delay(500);
}

void disarmDrone() {
    droneState.armed = false;
    setMotorSpeed(1, 1000);
    setMotorSpeed(2, 1000);
    setMotorSpeed(3, 1000);
    setMotorSpeed(4, 1000);
}

// ==================== FLIGHT CONTROL ====================
void updateFlightControl() {
    if (!droneState.armed) return;

    // Update sensor data
    updateIMUData();
    updateBarometerData();
    updateGPSData();
    obstacleAvoidance.update();

    // Obstacle avoidance logic
    if (obstacleAvoidance.isDanger()) {
        // Emergency maneuver
        disarmDrone();
        droneState.rthActive = true;
    }

    // Altitude hold
    float throttle = altPID.update(droneState.targetAlt, droneState.currentAlt, millis());

    // Attitude control
    float rollCmd = rollPID.update(0, droneState.rollAngle, millis());
    float pitchCmd = pitchPID.update(0, droneState.pitchAngle, millis());
    float yawCmd = yawPID.update(0, droneState.yawAngle, millis());

    // Apply attitude control to motors (simplified quadcopter mixing)
    uint16_t m1 = throttle + rollCmd - pitchCmd + yawCmd;
    uint16_t m2 = throttle - rollCmd - pitchCmd - yawCmd;
    uint16_t m3 = throttle - rollCmd + pitchCmd + yawCmd;
    uint16_t m4 = throttle + rollCmd + pitchCmd - yawCmd;

    setMotorSpeed(1, m1);
    setMotorSpeed(2, m2);
    setMotorSpeed(3, m3);
    setMotorSpeed(4, m4);

    // Waypoint navigation
    if (droneState.waypointActive && currentWaypoint < maxWaypoints) {
        navigateToWaypoint(waypoints[currentWaypoint]);

        float dist = getDistanceBetween(droneState.currentLat, droneState.currentLng,
                                        waypoints[currentWaypoint].lat,
                                        waypoints[currentWaypoint].lng);
        if (dist < 5.0) { // Reached waypoint
            currentWaypoint++;
        }
    }

    // RTH logic
    if (droneState.rthActive) {
        returnToHome();
    }

    // Telemetry transmission
    loraTelem.sendTelemetry(droneState);
    loraTelem.receiveCommands();
    simTelem.sendLocationViaGPRS(droneState);
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize sensors
    if (!bmp280.begin(0x76)) {
        Serial.println("BMP280 failed!");
    }
    if (!mpu9250.setup(0x68)) {
        Serial.println("MPU9250 failed!");
    }

    // Initialize GPS
    gpsSerial.begin(9600);

    // Initialize ultrasonic sensors
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(ULTRASONIC_TRIG2, OUTPUT);
    pinMode(ULTRASONIC_ECHO2, INPUT);

    // Initialize motor control
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);

    // Initialize communication modules
    loraTelem.init();
    simTelem.init();

    Serial.println("Drone initialized!");
}

// ==================== MAIN LOOP ====================
void loop() {
    updateFlightControl();

    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        lastDebug = millis();
        Serial.printf("Lat:%.6f Lng:%.6f Alt:%.2f Sat:%d Hdop:%.2f\n",
                      droneState.currentLat, droneState.currentLng,
                      droneState.currentAlt, droneState.satCount,
                      droneState.hdop);
        Serial.printf("Roll:%.1f° Pitch:%.1f° Yaw:%.1f°\n",
                      droneState.rollAngle, droneState.pitchAngle, droneState.yawAngle);
    }

    delay(10); // 100 Hz main loop
}
