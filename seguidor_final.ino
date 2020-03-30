//Librerias
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

////// MOTORES
int ENA = 11;
int IN1 = 2;
int IN2 = 3;
int ENB = 12;
int IN3 = 7;
int IN4 = 8;
int velocidad_motor_1 = 60;
int velocidad_motor_2 = 60;
////// CNY70
int Sensor1 = 9;
int Sensor2 = 10;
int cnyValor1 = 0;
int cnyValor2 = 0;
////// HC-SR04
int Trig = 38;
int Echo = 36;

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  digitalWrite(Trig, LOW);
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  checkSettings();
}

void checkSettings() {
  Serial.println();
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:      Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ:  Serial.println("PLL with external 19.2MHz reference");break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ:  Serial.println("PLL with external 32.76kHz reference");break;
    case MPU6050_CLOCK_PLL_ZGYRO:       Serial.println("PLL with z axis gyroscope reference");break;
    case MPU6050_CLOCK_PLL_YGYRO:       Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:       Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:   Serial.println("Internal 8MHz oscillator"); break;
  }
  Serial.println(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:             Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:              Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:              Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:              Serial.println("+/- 2 g"); break;
  }
  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}
void loop() {
  
  cnyValor1 = digitalRead(Sensor1);
  cnyValor2 = digitalRead(Sensor2);
  
  long duracion;
  long distancia;
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  duracion = pulseIn(Echo, HIGH);
  distancia = (duracion/2)/29;

  Serial.println(distancia);

 /////CONTROL MOTORES 
 
    if (cnyValor1 ==0 && cnyValor2 ==0 && distancia >= 11){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, velocidad_motor_1);
  
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, velocidad_motor_2);
      
    }else if(cnyValor1 ==0 && cnyValor2 ==0 && distancia <= 10){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
  
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);      
    }
    
    if (cnyValor1 ==0 && cnyValor2 ==1){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, velocidad_motor_1);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, LOW);
        
      } 

     if (cnyValor1 ==1 && cnyValor2 ==0){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, velocidad_motor_2);

     }

     if (cnyValor1 ==1 && cnyValor2 ==1 && distancia >= 11 ){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, velocidad_motor_1);
  
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, velocidad_motor_2); 
        
     }else if(cnyValor1 ==1 && cnyValor2 ==1 && distancia <= 10){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW); 
     }

     ///ACELEROMETRO
     {
      Vector rawAccel = mpu.readRawAccel();
      Vector normAccel = mpu.readNormalizeAccel();
      Serial.println(rawAccel.ZAxis);
      Serial.print(" Xnorm = ");
      Serial.print(normAccel.XAxis);
      Serial.print(" Ynorm = ");
      Serial.print(normAccel.YAxis);
      Serial.print(" Znorm = ");
      Serial.println(normAccel.ZAxis);
      
      delay(10);
     } 
}
