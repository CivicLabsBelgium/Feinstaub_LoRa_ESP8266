/*******************************************************************************
 * Copyright (c) 2016 Maarten Westenberg
 * based on work of Thomas Telkamp, Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This sketch sends a valid LoRaWAN packet with payload a DS18B 20 temperature
 * sensor reading that will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
*  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * History:
 * 2017-02-14 rxf
 *   use an DHT22-Sensor instead of DALLAS
 *
 *  2017-01-29 rxf
 *    adopted, to use SDS011 Particulate Matter Sensor
 *	  Sends data every minute to LoRaWan
 * Jan 2016, Modified by Maarten to run on ESP8266. Running on Wemos D1-mini
 *
 *******************************************************************************/

// Use ESP declarations. This sketch does not use WiFi stack of ESP
#include <ESP8266WiFi.h>
#include <Esp.h>
#include <base64.h>

// All specific changes needed for ESP8266 need be made in hal.cpp if possible
// Include ESP environment definitions in lmic.h (lmic/limic.h) if needed
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Serial for SDS011
#include "SoftwareSerial.h"

//---------------------------------------------------------
// LoRaWAN settings (for thethingsnetwork)
//---------------------------------------------------------

// Time between transmissions to LoRa in sec
#define LORA_SEND_TIME 60
#define CONNECT_2_SINGLE_CHANNEL_GATEWAY 1


// LoRaWAN Application identifier ^ ^(AppEUI)
// Not used in this example
static const u1_t APPEUI[8]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// LoRaWAN NwkSKey, network session key
// Use this key for The Things Network
static const u1_t DEVKEY[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t ARTKEY[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x01; // <-- Change this address for every node! ESP8266 node 0x01

// **********************************************************
// ******   Above settinge have to be adopted !!! ***********
// **********************************************************



//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------
#define SDS011 1				// uses SDS011
#define S_DHT 1					// Use DHT22

#if S_DHT == 1
#include <DHT.h>
#define DHT_PIN D1
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);
#endif

//---------------------------------------------------------
// APPLICATION CALLBACKS
//---------------------------------------------------------

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

#if SDS011 == 1
//---------------------------------------------------------
// div. timings for SDS011
//---------------------------------------------------------
#define SDS_SAMPLE_TIME 1000
#define SDS_WARMUP_TIME 10
#define SDS_READ_TIME 5
#endif
//---------------------------------------------------------
// Global Variables
//---------------------------------------------------------
int debug=1;
uint8_t mydata[64];
static osjob_t sendjob;

#if SDS011 == 1
// SDS-Variables
unsigned long act_milli, prev_milli;		// Timer-Ticks to calculate 1 sec
bool is_SDS_running = true;					// true, if SDS011 is running
uint8_t timer_SDS;							// Timer with 1sec ticks for SDS011 timimg

// Variables to calculate avereage for SDS011-Data
int sds_pm10_sum = 0;
int sds_pm25_sum = 0;
int sds_val_count = 0;

// Kommands to start and stop SDS011
const byte stop_SDS_cmd[] = {0xFF, 0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
const byte start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
#endif

#if SDS011 == 1
// Pinning for SDS011
#define SDS_PIN_RX D3
#define SDS_PIN_TX D4
SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);
#endif

// variables to store data for the measurements of sensors
byte result_SDS_by[4] = {0xFF,0xFF,0xFF,0xFF};
byte result_DHT_by[2] = {0xFF,0xFF};

// Pin mapping for RFM95
lmic_pinmap pins = {
  .nss = 15,			// Make D8/GPIO15, is nSS on ESP8266
  .rxtx = 0xFF, 		// Not used, Do not connected on RFM92/RFM95
  .rst = 0xFF,  		// Not used
//  .dio = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
  .dio = {16, 4, 0xFF},	// Specify pin numbers for DIO0, 1, 2;
};
// Reset, DIO2 and RxTx are not connected AND in hal.cpp NOT initialised.



void onEvent (ev_t ev) {
    //debug_event(ev);
    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          Serial.print("Event EV_TXCOMPLETE, time: ");
          Serial.println(millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println("Data Received!");
          }
          break;
       default:
          break;
    }
}

#if SDS011
/*****************************************************************
/* read SDS011 sensor values                                     *
/*****************************************************************/
void sensorSDS() {
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is;
	int checksum_ok = 0;

	if (! is_SDS_running) {
		return;
	}

	// SDS runs: read serial buffer
	while (serialSDS.available() > 0) {
		buffer = serialSDS.read();
		value = int(buffer);
		switch (len) {
			case (0): if (value != 170) { len = -1; }; break;
			case (1): if (value != 192) { len = -1; }; break;
			case (2): pm25_serial = value; checksum_is = value; break;
			case (3): pm25_serial += (value << 8); checksum_is += value; break;
			case (4): pm10_serial = value; checksum_is += value; break;
			case (5): pm10_serial += (value << 8); checksum_is += value; break;
			case (6): checksum_is += value; break;
			case (7): checksum_is += value; break;
			case (8): if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
			case (9): if (value != 171) { len = -1; }; break;
		}
		len++;
		if ((len == 10 && checksum_ok == 1) && (timer_SDS > SDS_WARMUP_TIME)) {
			if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
				sds_pm10_sum += pm10_serial;
				sds_pm25_sum += pm25_serial;
				sds_val_count++;
			}
			len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
		}
		yield();
	}

	// Data for SDS_READTIME time is read: now calculate the average and return value
	if (timer_SDS > (SDS_WARMUP_TIME + SDS_READ_TIME)) {
    int sdsp1 = (int)(sds_pm10_sum/sds_val_count);
    int sdsp2 = (int)(sds_pm25_sum/sds_val_count);
    result_SDS_by[0] = sdsp1>>8;
    result_SDS_by[1] = sdsp1&0xFF;
    result_SDS_by[2] = sdsp2>>8;
    result_SDS_by[3] = sdsp2&0xFF;
		//clear sums and count
		sds_pm10_sum = 0; sds_pm25_sum = 0; sds_val_count = 0;
		// and STOP SDS
		serialSDS.write(stop_SDS_cmd,sizeof(stop_SDS_cmd));
		is_SDS_running = false;

    Serial.println("PM10:  "+String(sdsp1*0.1));
		Serial.println("PM2.5: "+String(sdsp2*0.1));
		Serial.println("------");
		Serial.println("SDS stopped");
	}
}

#endif

#if S_DHT == 1
/*****************************************************************
/* read DHT22 sensor values                                      *
/*****************************************************************/
void sensorDHT() {
	float h = dht.readHumidity(); //Read Humidity
	float t = dht.readTemperature(); //Read Temperature

	Serial.println("Reading DHT22");

	// Check if valid number if non NaN (not a number) will be send.
	if (isnan(t) || isnan(h)) {
		Serial.println("DHT22 couldn't be read");
	} else {
		Serial.println("Humidity    : "+String(h)+"%");
		Serial.println("Temperature : "+String(t)+" C");
	}
	Serial.println("------");
  result_DHT_by[0] = (byte)(t*2);
  result_DHT_by[1] = (byte)h ;
}

#endif


// ----------------------------------------------------
// This function prepares a message for the LoRaWAN network
// The message will be sent multiple times.
//
void do_send(osjob_t* j){

  Serial.println();
  Serial.print("Time: "); Serial.println(millis() / 1000);
  // Show TX channel (channel numbers are local to LMIC)
  Serial.print("Send, txCnhl: "); Serial.println(LMIC.txChnl);
  Serial.print("Opmode check: ");
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & (1 << 7)) {
    Serial.println("OP_TXRXPEND, not sending");
  } else {
    Serial.print("ok, ready to send: ");
    Serial.print((char *)mydata);
    Serial.println();

#if S_DHT == 1
  sensorDHT();
#endif

    byte bytsend[6];                   // !!!! MAx 10 Bytes to send !!!!
    int idx = 0;
    // Build JSON-String to send to LoRa

    for (; idx<4; idx++) {
      bytsend[idx] = result_SDS_by[idx];
      result_SDS_by[idx] = 0; // clear old values
    }
    bytsend[idx++] = result_DHT_by[0];
    bytsend[idx++] = result_DHT_by[1];

    result_DHT_by[0] = 0xFF; // clear old values
    result_DHT_by[1] = 0xFF; // clear old values

	  // prepare message
    Serial.print("Long ByteArray:");
    Serial.println(idx);

    memcpy((char *)mydata, (char *)bytsend, idx);
    int k;
    for(k=0; k<idx; k++) {
      Serial.print(mydata[k],HEX);
      Serial.print(" ");
    }

    Serial.println();

    // Prepare upstream data transmission at the next possible time.
    // LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);
    LMIC_setTxData2(1, mydata, idx, 0);
  }
  // Schedule a timed job to run at the given timestamp (absolute system time)
  os_setTimedCallback(j, os_getTime()+sec2osticks(LORA_SEND_TIME), do_send);

#if SDS011 == 1
  // Now start SDS senor
  serialSDS.write(start_SDS_cmd,sizeof(start_SDS_cmd));
	is_SDS_running = true;
	timer_SDS = 0;							// start timer
	Serial.println("SDS started");
#endif
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  // switch WiFi OFF
  WiFi.disconnect();
  WiFi.forceSleepBegin();
  delay(1);

  // LMIC init
  os_init();
  Serial.println("os_init() finished");

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  Serial.println("LMIC_reet() finished");

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (uint8_t*)DEVKEY, (uint8_t*)ARTKEY);
  Serial.println("LMIC_setSession() finished");

#if CONNECT_2_SINGLE_CHANNEL_GATEWAY == 1
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_disableChannel(1);
  LMIC_disableChannel(2);
  LMIC_disableChannel(3);
  LMIC_disableChannel(4);
  LMIC_disableChannel(5);
  LMIC_disableChannel(6);
  LMIC_disableChannel(7);
  LMIC_disableChannel(8);
#endif

  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  Serial.println("LMICsetAddrMode() finished");

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  //
  Serial.println("Init done");

}


//---------------------------------------------------------
// main loop
// Loop is simple: read sensor value and send it to the LoRaWAN
// network.
//---------------------------------------------------------

void loop() {
	Serial.println("loop: Starting");

	do_send(&sendjob);						// Put job in run queue(send mydata buffer)
	delay(10);

	while(1) {
		act_milli = millis();				// read system-tick

		if((act_milli - prev_milli) >= SDS_SAMPLE_TIME) {   // after SAMPLE_TIME (==0 1sec)
			prev_milli = act_milli;
			timer_SDS += 1;					// Count SDS-Timer
			sensorSDS();					// check (and read)  SDS011
		}
		os_runloop_once();					// Let the server run its jobs
		delay(100);
	}
}
