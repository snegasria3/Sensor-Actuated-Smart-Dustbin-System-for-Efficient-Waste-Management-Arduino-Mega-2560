# Sensor-Actuated Smart Dustbin System for Efficient Waste Management

---

## 📄 Abstract

The **Smart Dustbin using Arduino Mega** is a sensor-actuator-based automation system designed to minimize human contact with waste, enhance hygiene, and demonstrate coordinated working of multiple electronic components.  

- **IR Sensor:** Detects user presence for touchless lid operation.  
- **Ultrasonic Sensor:** Monitors fill level of the dustbin.  
- **Servo Motor:** Opens/closes the lid automatically.  
- **GSM Module:** Sends SMS alerts when the bin is full.  
- **GPS Module:** Provides the location of the dustbin in real-time.  

The system enables real-time monitoring of waste levels, reduces manual effort, and supports smarter waste management practices.

---

## 🎯 Objectives

1. **Touchless Lid Operation:** Open/close the lid automatically using IR sensor detection.  
2. **Accurate Fill-Level Monitoring:** Measure bin occupancy with an ultrasonic sensor.  
3. **Geolocation Integration:** GPS coordinates to identify the bin location.  
4. **Automated Alerts:** GSM-based SMS notifications when the bin reaches a threshold (90% full).  
5. **Scalability and Cost Efficiency:** Modular design for deployment across urban areas.

---

## 🛠 System Design

**Components Used:**

| Component         | Role |
|------------------|------|
| Arduino Mega 2560 | Central controller |
| IR Sensor         | Detect user presence |
| Ultrasonic Sensor (HC-SR04) | Measure fill level |
| Servo Motor       | Open/close lid |
| GPS Module (NEO-6M) | Provide coordinates |
| GSM Module (SIM800L) | Send SMS alerts |

**Working Principle:**

1. IR sensor detects hand/object → Servo opens lid.  
2. Ultrasonic sensor measures distance → Arduino calculates fill %.  
3. If fill ≥ 90% → Arduino reads GPS coordinates → GSM sends SMS with location.  
4. Lid closes automatically after user moves away.  

---

## 🔧 Circuit Diagram & Connections

- Arduino Mega connects to all sensors and modules.  
- IR sensor → digital pin 2  
- Servo motor → digital pin 9  
- Ultrasonic sensor → TRIG pin 3, ECHO pin 4  
- GPS module → RX pin 5, TX pin 6  
- GSM module → RX pin 7, TX pin 12  

> Full circuit diagram is provided in the `Circuit Diagram(2).png` file.

---

## 💻 Software and Libraries

- **Arduino IDE**  
- Libraries used:
  - `Servo.h` – Controls servo motor  
  - `SoftwareSerial.h` – For communication with GPS and GSM modules  
  - `TinyGPS++.h` – Reads GPS data  

---

## 📂 Project Files

- `Codes` – Main Arduino program controlling the system  
- `Circuit Diagram(2).png` – Diagram of sensor and module connections
- `B-13.pdf` – Project documentation (Report)
- `ppt.pdf` – Project PPT Slides
- `README.md` – Project documentation  

---

## ⚙️ Arduino Code Overview

```cpp
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define IR_PIN 2
#define SERVO_PIN 9
#define TRIG_PIN 3
#define ECHO_PIN 4
#define GPS_RX 5
#define GPS_TX 6
#define GSM_RX 7
#define GSM_TX 12

Servo myservo;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);
TinyGPSPlus gps;

const int dustbinHeightCm = 16;
bool binFullSent = false;

void setup() {
    Serial.begin(9600);
    pinMode(IR_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    myservo.attach(SERVO_PIN);
    gpsSerial.begin(9600);
    gsmSerial.begin(9600);
}

void loop() {
    int irState = digitalRead(IR_PIN);
    myservo.write(irState == HIGH ? 90 : 0);

    long duration, distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;

    int levelPercent = map(distance, 0, dustbinHeightCm, 100, 0);
    levelPercent = constrain(levelPercent, 0, 100);
    Serial.println(levelPercent);

    while (gpsSerial.available()) { gps.encode(gpsSerial.read()); }

    if (levelPercent >= 90 && !binFullSent) {
        sendBinFullSMS();
        binFullSent = true;
    }

    if (levelPercent < 90) binFullSent = false;
    delay(1000);
}

void sendBinFullSMS() {
    String locationString = gps.location.isValid() ? 
        "Lat:" + String(gps.location.lat(), 6) + " Lng:" + String(gps.location.lng(), 6) :
        "Lat:10.9038 Lng:76.8984";
    
    gsmSerial.println("AT+CMGF=1");
    delay(2000);
    gsmSerial.println("AT+CMGS=\"+918618892922\"");
    delay(2000);
    gsmSerial.print("Dustbin full! Location: ");
    gsmSerial.print(locationString);
    gsmSerial.write(26);
    delay(3000);
}
