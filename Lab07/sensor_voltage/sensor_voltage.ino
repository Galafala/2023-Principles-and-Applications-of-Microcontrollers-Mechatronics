int analogPin = A0;
float sensorVal = 0;
float sensorVolt = 0;
float Vr=5.0;
float sum=0;
float k1=20.24729976;
float k2=-0.90411754;
float distance=0;
void setup() {
  Serial.begin(9600);
   
}
 
void loop() {
 
  sum=0;
  for (int i=0; i<100; i++)
  {
    sum=sum+float(analogRead(analogPin));  
  }
  sensorVal=sum/100;
  sensorVolt=sensorVal*Vr/1024;
// 
  distance = pow(sensorVolt*(1/k1), 1/k2);
  Serial.println(sensorVolt, 10);
  delay(500);
}
