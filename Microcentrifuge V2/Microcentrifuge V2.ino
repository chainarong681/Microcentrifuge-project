#include <Wire.h> 
#include <SparkFun_APDS9960.h>
#include <Servo.h>

//หลอด LED Module GND -> GND, ขา 9 -> R, ขา 10 -> G และ ขา 11 -> B
int redpin = 9;       // select the pin for the red LED
int greenpin = 10;    // select the pin for the  blue LED
int bluepin = 11;     // select the pin for the green LED

//เซนต์เซอร์ TCRT5000 infrared reflectance Obstacle avoidance line tracking ต่อสาย A0 -> A0 , 5V -> VCC และ GND -> GND
int sensor = A0;
int val = 0;

//GY-9960 RGB and Gesture Sensor ต่อสาย 3.3V -> VCC, GND -> GND, A4 -> SDA, A5 -> SCL และ 2 -> INT
#define APDS9960_INT 2
SparkFun_APDS9960 apds = SparkFun_APDS9960();
volatile int isr_flag = 0;
// สถานะอนุญาตให้ Gesture ทำงาน
volatile bool allowGesture = false;

//ตัวแปรใหม่สำหรับสถานะเครื่องปั่น
int centrifugework = 0;

// --- เพิ่มตัวแปร PWM --- pin 7 สำหรับต่อ PWM
int pwmPin = 7;        // ใช้ขา D7 เป็น PWM output
int pwmPot = A1;       // R10K ต่อที่ขา A1
int pwmValue = 0;      // ค่าที่จะส่งออก PWM

// --- BUZZER CONTROL ---
//Active Buzzer Module ต่อสาย ขา VCC -> 5V, ขา Gnd-> Gnd และ ขา I/O -> 6
int buzzer = 6;                // ขาที่ต่อ buzzer
int buzzerCount = 0;           // นับจำนวนครั้งที่ดังแล้ว
unsigned long buzzerLastTime = 0;
bool buzzerActive = false;
bool buzzerState = false;      // สถานะ HIGH/LOW ตอนกระพริบเสียง
unsigned long buzzerInterval = 150; // ความถี่การสลับเสียง

//Servo ใช้ pin 5
Servo myservo;  // สร้านออปเจ็กชื่อ myservo จากคลาส Servo เพื่อควบคุม servo micro

void setup() {
  //LED pin
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  //Gesture Sensor
  pinMode(APDS9960_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);
  apds.init();
  apds.enableGestureSensor(true);

  //PWM
  pinMode(pwmPin, OUTPUT);
  pinMode(pwmPot, INPUT);

  //Buzzer
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  //Servo
  myservo.attach(5);  // บอกว่าจะต่อ servo ที่ขา 5

  //Serial print
  Serial.begin(9600);

  //setLED(HIGH, HIGH, LOW);  // เเหลือง

  val = -1;  //บังคับให้ IR ถูกประมวลผลตั้งแต่เปิดเครื่อง
}

// --- ตัวแปรลดค่า PWM ---
unsigned long lastReduceTime = 0; 
int currentPWM = 0;   // ตัวแปรเก็บ PWM ขณะลดค่า

void loop() {
  //IR
  int newVal = digitalRead(sensor);
  Serial.println(newVal);
  
  //PWM
  int analogVal = analogRead(pwmPot);
  pwmValue = map(analogVal, 0, 1023, 0, 255);   //ค่า PWM สูงสุดเท่ากับ 255
  Serial.println(pwmValue);

  //ตรวจสอบการเปิดฝาถ้าเปิดค่าจะเท่ากับ 1
  if (newVal != val) {
    if (newVal == 1) {
      allowGesture = false;
      setLED(LOW, LOW, HIGH);  // ฟ้า
      // ปิดการทำงานเครื่องปั่นทันทีตามที่ต้องการ
      centrifugework = 0;
      analogWrite(pwmPin, 0);
      //Servo เปิดฝา
      myservo.write(0);     // ไปที่ 0°
      Serial.println("val = 1 → centrifugework=0 and PWM OFF");
    }
    //ตวจสอบการปิดฝาถ้าปิดฝาค่าจะเท่ากับ 0
    if (newVal == 0) {
      setLED(HIGH, LOW, HIGH);  // ม่วง
      unsigned long start = millis();
      while (apds.isGestureAvailable()) {
        apds.readGesture();
        delay(5);
        if (millis() - start > 200) break;
      }
      allowGesture = true;
    }
    val = newVal;
  }

  //การทำงานของ gesture sensor
  if (allowGesture && isr_flag == 1) {
    detachInterrupt(digitalPinToInterrupt(APDS9960_INT));
    handleGesture(); //ฟังก์ชันการอ่านท่าทาง
    isr_flag = 0;
    attachInterrupt(digitalPinToInterrupt(APDS9960_INT), interruptRoutine, FALLING);
  }

  // อ่าน PWM จาก R10 เฉพาะตอน centrifugework = 2 (สถานะปั่น)
  if (centrifugework == 2) {
    //ปั่นที่ค่า PWM สูงสุด
    int maxPWM = 255;             // ค่า PWM สูงสุดของ Arduino
    analogWrite(pwmPin, maxPWM);  // ปั่นเต็มแรงที่ PWM เท่ากับ 255
    
    //analogWrite(pwmPin, pwmValue); //ปั่นความเร็วตามค่า POT
    currentPWM = pwmValue; //เซ็ทค่า currentPWM ให้เท่ากับ pwmValue เสมอเพื่อเริ่มต้นใหม่จากการรับค่าจาก POT เป็นสัดส่วนตาม POT
    Serial.print("Running PWM = ");
    Serial.println(pwmValue);
  }

  // ถ้า centrifugework = 0 → ปิด PWM pin ไม่ให้ทำงานจะไม่เกิดการปั่น
  if (centrifugework == 0) {
    analogWrite(pwmPin, 0);   //ปิด pin ไม่ให้มอเตอร์ปั่น
  }

  // ถ้า centrifugework = 1 (สถานะหยุดปั่น)→ ลดค่า pwm ทีละ 25 ต่อจนถึง 0
  if (centrifugework == 1) {
    if (millis() - lastReduceTime >= 1000) {
      lastReduceTime = millis();
      // เริ่มลดจากค่า pwm ล่าสุดก่อนเริ่มลด
      if (currentPWM == 0) {
        currentPWM = pwmValue;  //ถ้าค่า currentPWM == 0 เมื่อจ่ายค่า PWM ลดจนมอเตอร์หยุดหมุนจะ reset ค่า currentPWM ไปเริ่มใหม่เท่ากับค่า POT จากตัวแปล pwmValue
      }
      //ลดค่าลงทีละ 25
      currentPWM -= 25;
      if (currentPWM < 0) currentPWM = 0;   //ตั้งค่าถ้าค่า currentPWM < 0 ใหกำหนดใหั currentPWM = 0
      analogWrite(pwmPin, currentPWM);  //Pin ปล่อยสัญญาณ PWM
      Serial.print("Reducing PWM = ");
      Serial.println(currentPWM);

      // ลดจนเหลือ 0 → ปิดการปั่นของเครื่อง
      if (currentPWM == 0) {
        centrifugework = 0; //ปรับค่าเป็น 0
         // สั่งบัซเซอร์ดัง 3 ครั้ง
        buzzerActive = true;
        buzzerCount = 0;
        //Servo
        myservo.write(0);     // หมุนมอเตอร์ฝาไปที่ 0°
        Serial.println("Centrifuge STOP → PWM off");
      }
    }
  }
  //ฟังก์ชัน Buzzer
  buzzerRun3Times();
  //delay(500);
}

void interruptRoutine() {
  isr_flag = 1;
}

// --- Functions ตั้งค่ากำหนด pi สี LED ---
void setLED(int r, int g, int b) {
  digitalWrite(redpin, r);
  digitalWrite(greenpin, g);
  digitalWrite(bluepin, b);
}

void handleGesture() {
  //ฟังก์ชันการทำงาน gesture sensor
  if (!apds.isGestureAvailable()) return;
  int gesture = apds.readGesture();
  switch (gesture) {
    case DIR_LEFT:
      Serial.println("LEFT");
      setLED(LOW, HIGH, LOW);  // เขียว
      //สีเขียว = ปิดการทำงานเครื่องปั่น
      // อนุญาตให้เข้าโหมดลดรอบได้เฉพาะเมื่อ centrifugework เคยเป็น 2 มาก่อน
      if (centrifugework == 2) {
        centrifugework = 1;
        Serial.println("centrifugework = 1 (start reducing speed)");
      } else {
        Serial.println("Ignore LEFT → centrifuge was not running (need =2 first)");
      }
      Serial.print("centrifugework = ");
      Serial.println(centrifugework);
      break;

    case DIR_RIGHT:
      Serial.println("RIGHT");
      //Servo
      myservo.write(180);   // หมุนมอเตอรืไปที่ 180°
      //delay(500);  //ให้ล็อกฝาก่อนเกิดการปั่น
      setLED(HIGH, LOW, LOW);  // แดง
      
      //สีแดง = เริ่มงานเครื่องปั่น
      centrifugework = 2;
      Serial.print("centrifugework = ");
      Serial.println(centrifugework);
      break;
      default:
      break;
  }
}

void buzzerRun3Times() {
  if (!buzzerActive) return;
  unsigned long now = millis();
  // --- ระยะเวลาปรับให้ยาวขึ้น ---
  unsigned long onTime = 250;   // เปิดเสียงนานขึ้น
  unsigned long offTime = 100;  // ปิดเสียงสั้นลง
  if (buzzerState == HIGH) {
    // ถ้าอยู่ในช่วงดัง
    if (now - buzzerLastTime >= onTime) {
      buzzerState = LOW;
      digitalWrite(buzzer, LOW);
      buzzerLastTime = now;
    }
  } 
  else {
    // ถ้าอยู่ในช่วงปิด
    if (now - buzzerLastTime >= offTime) {
      buzzerState = HIGH;
      digitalWrite(buzzer, HIGH);
      buzzerLastTime = now;
      buzzerCount++;   // นับครั้งเฉพาะตอนสลับมาเป็น HIGH

      if (buzzerCount >= 4) {
        // ครบ 3 ติ๊ด
        buzzerActive = false;
        buzzerState = LOW;
        digitalWrite(buzzer, LOW);
        buzzerCount = 0;
        Serial.println("Buzzer finished 3 strong beeps");
      }
    }
  }
}
