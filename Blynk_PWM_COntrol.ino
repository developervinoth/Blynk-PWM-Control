#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 3    
#define DHTTYPE    DHT11   

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = "rpTa171fGR57Ii1CqobxPscoCo9kwJxj";  
char ssid[] = "Testing"; //Enter your Wifi User Name
char pass[] = "12341234"; //Enter your Wifi Password
int cmd =0;
int cmd_led =0;
#define enA 16 
#define in1 5
#define in2 4
#define in3 0
#define in4 2
#define enB 14
int motorSpeedA=0;
int motorSpeedB=0;


#define ir_in 12
#define ir_out 13

int count =0;



void IN(){
  count++;
  }
void OUT(){
  count--;
  }
  
BLYNK_WRITE(V0)
{
  motorSpeedA = (param.asInt());
analogWrite(enA, motorSpeedA);


  
}
BLYNK_WRITE(V1)

{motorSpeedB = (param.asInt());

  analogWrite(enB, motorSpeedB);

}
BLYNK_WRITE(V2)
{
  int cmd= (param.asInt());
 if(cmd == 0){
analogWrite(enA,0);
  }else{
        analogWrite(enA,1000 );
    }
}
BLYNK_WRITE(V3)
{
  int cmd_led= (param.asInt());
 if(cmd_led == 0){
analogWrite(enB,0);
  }else{
        analogWrite(enB,1000 );
    }
}


void setup() {
  
  Serial.begin(9600);
dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  
  
  delayMS = sensor.min_delay / 1000;
  Blynk.begin(auth, ssid, pass);
  delayMS = sensor.min_delay / 1000;
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

 
  // Setup a function to be called every second

}
void loop() {
    
  if(digitalRead(ir_in)){
    IN();
    }
    delay(1000);
    if(digitalRead(ir_out)){
      OUT();
      }
      if((count==1) && (cmd==0)){
        analogWrite(enA, 300);
        }
         if((count==2) && (cmd==0)){
        analogWrite(enA, 750);
        }
         if((count>=3) && (cmd==0)){
        analogWrite(enA, 1000);
        }
Serial.println(count);
  Blynk.run();



 // Send PWM signal to motor A






  
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed

delay(delayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);

    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    int temp=event.temperature;
    Serial.println(F("°C"));
 Blynk.virtualWrite(V5, temp);
 
  dht.humidity().getEvent(&event);

    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    int hum=event.relative_humidity;
     Blynk.virtualWrite(V6, hum);
  Serial.println(motorSpeedA);
  Serial.println(motorSpeedB);
 // Send PWM signal to motor B
  Blynk.virtualWrite(V4, count);
}
