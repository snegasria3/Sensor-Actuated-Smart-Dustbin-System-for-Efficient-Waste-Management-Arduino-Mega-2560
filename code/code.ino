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
SoftwareSerial gpsSerial(GPS_RX
, GPS_TX);
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);
TinyGPSPlus gps;

const int dustbinHeightCm = 16; // Height of dustbin in cm

bool binFullSent = false; // To avoid sending multiple SMS repeatedly

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
  // IR sensor controls servo open/close
  int irState = digitalRead(IR_PIN);

// If using active LOW IR sensor
  if (irState == LOW) {     
    myservo.write(90);  // Open when hand detected
  } else {
      myservo.write(0);   // Close when no hand
  }


  // Ultrasonic distance measurement
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  // Convert to percentage full (100% means dustbin full)
  int levelPercent = map(distance, 0, dustbinHeightCm, 100, 00);
  levelPercent = constrain(levelPercent, 0, 100);
levelPercent =map(levelPercent , 30, 100, 0, 100);
  Serial.print("Dustbin Level (%): ");
  Serial.println(levelPercent);

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Check if dustbin is 80% or more full and send SMS once
  if (levelPercent >= 80 && !binFullSent) {
    sendBinFullSMS();
    binFullSent = true;
  }

  // Reset flag if level goes below 80%
  if (levelPercent < 80) {
    binFullSent = false;
  }

  delay(1000);
}

void sendBinFullSMS() {
  String locationString;

  if (gps.location.isValid() && gps.location.isUpdated()) {
    // GPS location available
    locationString = "Lat:" + String(gps.location.lat(), 6) + " Lng:" + String(gps.location.lng(), 6);
  } else {
    // GPS unavailable - send default location
    locationString = "Lat:10.9038 Lng:76.8984"; // Example default location (Bangalore)
  }

  gsmSerial.println("AT+CMGF=1");    // SMS text mode
  delay(2000);
  gsmSerial.println("AT+CMGS=\"+918610892922\""); // Replace with recipient number
  delay(2000);
  gsmSerial.print("Dustbin full! Location: ");
  gsmSerial.print(locationString);
  gsmSerial.write(26); // CTRL+Z to send SMS
  delay(3000);

  Serial.println("Bin full SMS sent.");
}