int analogPin = A0;
int analogPin1 = A1;
int analogPin2 = A2;

float sensorVal = 0;
float sensorVal1 = 0;
float sensorVal2 = 0;

float sensorVolt = 0;
float Vr=5.0;

float sum=0;
float sum1=0;
float sum2=0;

float k1=20.24729976;
float k2=-0.90411754;
float distance=0;

void setup() {
  Serial.begin(9600);
   
}
 
void loop() {
 
  sum=0;
  sum1=0;
  sum2=0;
  for (int i=0; i<10; i++)
  {
    sum +=float(analogRead(analogPin));  
    sum1+=float(analogRead(analogPin1));
    sum2+=float(analogRead(analogPin2));
  }

  
  sensorVal  = sum /10;
  sensorVal1 = sum1/10;
  sensorVal2 = sum2/10;
  
//  sensorVolt=sensorVal*Vr/1024;
// 
//  distance = pow(sensorVolt*(1/k1), 1/k2);

  Serial.print(sensorVal);
  Serial.print(' ');
  Serial.print(',');
  Serial.print(' ');
  Serial.print(sensorVal1);
  Serial.print(' ');
  Serial.print(',');
  Serial.print(' ');
  Serial.println(sensorVal2);
  
  delay(500);
}
