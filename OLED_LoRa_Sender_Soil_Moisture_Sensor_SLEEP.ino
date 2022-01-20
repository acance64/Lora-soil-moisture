/*
  This is a simple example show the Heltec.LoRa sended data in OLED.

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/
#include "heltec.h"
#include "images.h"

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
// Hardware information.
#define SENSOR_POWER 13                            // Connect the power for the soil sensor here GPIO13
#define SOIL_PIN 12                                // Connect the sensor output pin here  GPIO12
#define SLEEP_TIME_SECONDS 900                      //interval between readings and posting on LORA - IF EMERGENCY_NO_SLEEP_PIN==HIGH (i.e. not connected to ground), the LORA32 will go in deep sleep 
#define EMERGENCY_NO_SLEEP_PIN 2                   //This is for emergency, avoids sleeps and thus code can be uploaded without problems
                                                   //connect GPIO 2 to GND and won't sleep 
#define uS_TO_S_FACTOR 1000000ULL


// Global variables. 
int numMeasure = 5;                                // Number of measurements to average.
//int ADCValue = 0;                                  // Moisture sensor reading.

unsigned int counter = 0;
unsigned int soilMeasure;

String rssi = "RSSI --";
String packSize = "--";
String packet ;

void logo()
{
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}

void setup()
{

// setup pin  as a digital output pin
  
  pinMode (SENSOR_POWER, OUTPUT);
  //digitalWrite( SENSOR_POWER, HIGH );
  

  Serial.begin(115200);
   //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
 
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  logo();
  delay(1500);
  Heltec.display->clear();
  
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->display();
  delay(1000);

    pinMode(EMERGENCY_NO_SLEEP_PIN, INPUT_PULLUP);       //again, this is for emergency

   /*First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_SECONDS * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(SLEEP_TIME_SECONDS) +
  " Seconds");
Heltec.display->clear();
Heltec.display->drawString(0, 0, "Sleep " + String(SLEEP_TIME_SECONDS) +
  " Secs");
  Heltec.display->drawString(0, 15, "Now wait 10 secs");
  Heltec.display->display();
  delay(10000);
  

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
    
}

void loop()
{



  //read sensor
  
  soilMeasure = readSoil( numMeasure );

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  
  Heltec.display->drawString(0, 0, "Sending packet: ");
  Heltec.display->drawString(0, 15, String(soilMeasure));
  Heltec.display->display();
  
  // send packet
  LoRa.beginPacket();
  
/*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
*/
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  //LoRa.print("hello ");
  //LoRa.print(counter);
  LoRa.print(soilMeasure);
  LoRa.endPacket();

  Serial.print("Publish message: ");
  Serial.println(soilMeasure);



  //delay(SLEEP_TIME_SECONDS*1000);

   if (digitalRead(EMERGENCY_NO_SLEEP_PIN) == HIGH) {
    //delay( 1000 );
    Serial.println( "EMERGENCY_NO_SLEEP_PIN== HIGH - GOING TO SLEEP FOR" + String(SLEEP_TIME_SECONDS) +
  " Seconds" );
     esp_deep_sleep_start();
    }
else
    {
    
    Serial.println( "EMERGENCY_NO_SLEEP_PIN) == LOW  - just pausing for " + String(SLEEP_TIME_SECONDS) +
  " Seconds");
    delay( SLEEP_TIME_SECONDS*1000 );
     }


/*
    if (digitalRead(EMERGENCY_NO_SLEEP_PIN) == HIGH) {
    delay( 1000 );
    Serial.print( "Goodnight for "+String( SLEEP_TIME_SECONDS ) + " Seconds" );
    ESP.deepSleep( SLEEP_TIME_SECONDS * 1000000 );   //deep sleep microseconds
    }
    else
    {
    Serial.print( "No sleep, Just waiting..." +String( SLEEP_TIME_SECONDS ) + " Seconds");
     delay( SLEEP_TIME_SECONDS * 1000 );
    }
  */


/*
  counter++;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
*/
}

// This function reads the soil moisture sensor numAve times and returns the average.
long readSoil(int numAve)
{
  long ADCValue = 0;

  digitalWrite( SENSOR_POWER, HIGH);  // Turn power to device on
    
    delay(3000);    // Wait 3000 milliseconds for sensor to settle
    
  for ( int i = 0; i < numAve; i++ ) {
    
    ADCValue += analogRead( SOIL_PIN );     // Read the value from sensor
    delay(10);    // Repeat measurement after 10 millisecs


  }

    digitalWrite( SENSOR_POWER, LOW );   // Turn power to device off

  
  ADCValue = ADCValue / numAve;
  return ADCValue;                    // Return the moisture value.
}
