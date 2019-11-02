float rawADC = 0;
float volts = 0;
int count = 0;
void setup() {
	// put your setup code here, to run once:

	pinMode(36, INPUT);

	Serial.begin(115200);
}

void loop() {
	// put your main code here, to run repeatedly:
	while (count < 200){

		rawADC = (analogRead(36)*0.02); // sensor reading * (5/256)
		Serial.println(rawADC);
		count++;
//		Serial.print(" , ");
//		Serial.println(volts);
	}
}
