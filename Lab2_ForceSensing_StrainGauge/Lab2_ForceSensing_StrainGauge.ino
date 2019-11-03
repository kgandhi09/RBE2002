#include "Arduino.h"
#include "ESP32Servo.h"
#include "analogWrite.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <stdint.h>
#include "esp_types.h"
#include "driver/adc.h"
//#include "soc/efuse_periph.h"
#include "esp_err.h"
#include "assert.h"
#include "esp_adc_cal.h"


/*
 * Timer stuff. Do not edit!
 */
// Pointer to the timer object
hw_timer_t *timer = NULL;
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t complexHandlerTask;

/*
 * Student will edit samplingFreq
 */
uint32_t samplingFreq = 40; //Hz


double R = 4467; // ohms
double C = 0.000000106; // F
double omega = 2*3.14*samplingFreq;
double omegaRC = omega*R*C;
double vout = 0;
double dtOverRC = 0.05; //((1/double(samplingFreq))/(R*C));
int prevADC = 0;
double prevVout = 0;
int timeMillis;

/*
 * Here is where you implement your filter
 */
void studentCode()
{
	timeMillis = millis();
	int adc = local_adc1_read(0); //reads ADC1:0, which is pin 36, 12-bit conversion
	static int dac = 0;
	//	Serial.print("adc: ");
	//	Serial.println(adc);

	vout = adc - prevADC + prevVout - (dtOverRC*prevVout) + 128; // high pass



	//	Serial.print("dtOverRC: ");
	//	Serial.println(dtOverRC);
	//  int vout = adc * (1 / (1 + omegaRC));
	//	dac = vout;

	dac = (int)vout >> 4; //the dac takes an 8-bit value, so divide the adc by 16

	Serial.print("Time: ");
	Serial.print(timeMillis);
	Serial.print("\tADC: ");
	Serial.println(adc);

	prevADC = adc;
	prevVout = vout;
	//	Serial.print("prevADC: ");
	//	Serial.println(prevADC);

	analogWrite(25, (uint8_t) dac); //pin 25 is DAC1, but note that it's 8-bit
}

/*
 * Dragons below
 */

int IRAM_ATTR local_adc1_read(int channel) {
	uint16_t adc_value;
	SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
	while (SENS.sar_slave_addr1.meas_status != 0)
		;
	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;
	while (SENS.sar_meas_start1.meas1_done_sar == 0)
		;
	adc_value = SENS.sar_meas_start1.meas1_data_sar;
	return adc_value;
}

void complexHandler(void *param) {
	while (true) {
		// Sleep until the ISR gives us something to do, or for 1 second
		ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
		studentCode();
	}
}
// Place the Interrupt handeler in IRAM section of memory
void IRAM_ATTR onTimer() {
	// A mutex protects the handler from reentry (which shouldn't happen, but just in case)
	portENTER_CRITICAL_ISR(&timerMux);

	// Notify complexHandlerTask that the buffer is full.
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(complexHandlerTask, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}

	portEXIT_CRITICAL_ISR(&timerMux);

}

void setup() {
	uint32_t count = 1000000 / samplingFreq; //count is in us -- don't edit this

	xTaskCreate(complexHandler, "Handler Task", 8192, NULL, 1,
			&complexHandlerTask);
	Serial.begin(115200);
	Serial.println(
			"Start ESPMutexDemo " + String(count)
			+ " microsecond timer interrupts");
	// get a pointer to a timer to use
	timer = timerBegin(3, // Timer 3
			80, // Divider from the system clock to get to us
			true); // Count up
	timerAttachInterrupt(timer, // Timer to attach the interrupt
			&onTimer, // Interrupt handler passed to timer
			true); // Rising edge of timer
	timerAlarmWrite(timer, // The timer object to use
			count, // count, now in us
			true); // Reload after finishes, run again and again
	timerAlarmEnable(timer); // Enable timer
	analogRead(36);

#define V_REF 1100  // ADC reference voltage

	// Configure ADC
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);

	// Calculate ADC characteristics i.e. gain and offset factors
	esp_adc_cal_characteristics_t characteristics;
	esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);
	uint32_t voltage=0;
	// Read ADC and obtain result in mV
	uint32_t err = esp_adc_cal_get_voltage(ADC_CHANNEL_0, &characteristics,&voltage);
	printf("%d mV\n",voltage);

}

void loop() {
	//Serial.println("Looping");
	delay(100);
}
