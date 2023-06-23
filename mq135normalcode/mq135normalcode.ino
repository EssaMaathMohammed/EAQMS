#include "MQ135.h" 
MQ135 gasSensor = MQ135(A0); 
int val; 
int sensorPin = A0; 
int sensorValue = 0; 
void setup() { 
  SerialUSB.begin(9600);

} 

void loop() { 
  val = analogRead(A0); 
  SerialUSB.print ("raw = "); 
  SerialUSB.println (val); 
  float zero = gasSensor.getRZero(); 
  SerialUSB.print ("rzero: "); 
  SerialUSB.println (zero); 
  float ppm = gasSensor.getPPM(); 
  SerialUSB.print ("ppm: "); 
  SerialUSB.println (ppm); 
  delay(5000); 
} 