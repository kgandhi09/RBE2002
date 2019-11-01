int outPin = 33;
  int pulse = 0;
  int pulseTime = 0;
  
  //int time = 0; 

  double Vdistance = 0;
  double Tdistance = 0;

void setup() {
  // put your setup code here, to run once:
//int trigPin = 32
//int echoPin = 36
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);

  pinMode(33, INPUT);
  pinMode(36, INPUT);
  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(32, HIGH);
 delayMicroseconds(20);
 pulse = digitalRead(outPin);
 //time = micros() + 1000; 
 while (!pulse){
  pulse = digitalRead(outPin);
 }
 pulseTime = micros();
  while (pulse){
    pulse = digitalRead(outPin);
 }
 pulseTime = micros() - pulseTime;

Vdistance = analogRead(36);
Vdistance = Vdistance*(3.3/512);

Tdistance = pulseTime/147.;

Serial.println("Hello!");
Serial.println(Vdistance);
Serial.println(Tdistance);
Serial.println();
}
