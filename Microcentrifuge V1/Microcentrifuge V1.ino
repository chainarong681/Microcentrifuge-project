// เซนเซอร์วัดระยะ VL6180X ต่อสาย VIN ต่อ 5V, GND ต่อ GND, SDA ต่อ A4 และ SCL ต่อ A5
// Servo SG90 ต่อสาย VIN ต่อ 5V, GND ต่อ GND และ สีส้ม(Signal) ต่อ D9
// FastLED ใช้ library นี้ต่อสาย VCC ต่อ 5V, GND ต่อ GND และ DIN ต่อ D5
// Relay ขา D2 ใช้สำหรับเปิดต่อใช้งาน ต่อไฟ out put ใช้ COM และ NO ต่อเพื่อจ่ายไฟ
// Switch ใช้ GND ต่อ GND และ ขั้วอีกอัน ต่อ D3

#include <Wire.h>
#include <SparkFun_VL6180X.h>
#include <Servo.h>
#include <FastLED.h>

// ---- VL6180X ----
#define VL6180X_ADDRESS 0x29  // ที่อยู่ I2C ของเซนเซอร์
VL6180x sensor(VL6180X_ADDRESS);

// ---- Servo ----
Servo myservo;  // สร้างอ็อบเจ็กต์ Servo

// ---- LED TM1803 ----
#define NUM_LEDS 1  // จำนวน LED
#define DATA_PIN 5  // ขา DIN
CRGB leds[NUM_LEDS];

// ---- D2 Output ----
#define RELAY_PIN 2  // หรือ LED/รีเลย์

// ตัวอย่าง: ระบบจะทำงานได้เมื่อกดปุ่มค้างไว้ (จำลอง "ฝาปิดอุปกรณ์")
#define BUTTON_PIN 3   // ขาที่ต่อกับปุ่มกด (หรือสวิตช์ฝาปิด)

void setup() {
  Serial.begin(115200);
  Wire.begin();
  myservo.attach(9);  // ต่อ Servo ที่ขา 9

  // ตั้งค่าขา D2 เป็น OUTPUT
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // เริ่มต้นปิดไว้ก่อน

  // ตั้งค่า FastLED
  FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);

  // เปิดใช้งาน internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(100);

  if (sensor.VL6180xInit() != 0) {
    Serial.println("Failed to initialize VL6180X. Check wiring.");
    while (1);
  }

  sensor.VL6180xDefautSettings();  // โหลดค่ามาตรฐานของเซนเซอร์
  Serial.println("VL6180X ready...");
  delay(500);
}

void loop() {

  //Button check
  int buttonState = digitalRead(BUTTON_PIN);
  
  int distance = sensor.getDistance();  // อ่านค่าระยะทาง (mm)
  Serial.print("Distance (mm): ");
  Serial.println(distance);

    if (buttonState == LOW) {
      // ปุ่มถูกกดค้างไว้ (ฝาปิดอยู่)
      if (distance > 250) {
        // ถ้าระยะมากกว่า 250 mm
        myservo.write(0);   // หมุนกลับที่เดิม
        leds[0] = CRGB::Green;  // ไฟสีเขียว
        FastLED.show();
    
        digitalWrite(RELAY_PIN, LOW);  // ปิดขา D2
        Serial.println("Servo 0° | D2 OFF | Distance > 250");
      } 
      else {
        // ถ้าระยะน้อยกว่าหรือเท่ากับ 250 mm
        myservo.write(90);  // หมุนไปขวา 90 องศา
        leds[0] = CRGB::Red;  // ไฟสีแดง
        FastLED.show();
    
        digitalWrite(RELAY_PIN, HIGH);  // เปิดขา D2
        Serial.println("Servo 90° | D2 ON | Distance <= 250");
        delay(2000);
      }
    } else {
      // ปุ่มไม่ถูกกด (ฝาเปิดอยู่)
      leds[0] = CRGB::Yellow;  // ไฟสีเหลือง
      FastLED.show();
      myservo.write(0);   // หมุนกลับที่เดิม
      digitalWrite(RELAY_PIN, LOW);  // ปิดขา D2
    }
  delay(500);
}
