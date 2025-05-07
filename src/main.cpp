#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL343.h"
#include <WiFi.h>
 #include "Adafruit_MQTT.h"
#include "AdafruitIO_WiFi.h"

#define IO_KEY "YOUR_IO_KEY"
#define IO_USERNAME "YOUR_USERNAME"
#define WIFI_SSID      "oegr"
#define WIFI_PASS      "oegr1234"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

AdafruitIO_Feed *DATAREADY = io.feed("dataready");
AdafruitIO_Feed *X_ = io.feed("x");
AdafruitIO_Feed *Y_ = io.feed("y");
AdafruitIO_Feed *Z_ = io.feed("z");

bool state_=false;

#define SDA_PIN 5 // SDA pin for ESP32
#define SCL_PIN 6 // SCL pin for ESP32

Adafruit_ADXL343 accel1 = Adafruit_ADXL343(12345);

void setup(void)
{
  delay(5000);
  Serial.begin(115200);
  while (!Serial);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("*");
    delay(500);
  }
  Serial.println("WiFi connected!");

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with custom SDA and SCL pins

  // I2C Scanner
  Serial.println("Scanning for I2C devices...");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" I2C devices.");
  }
  Serial.println("");

  /* Initialise the first sensors, this uses the default address */
  if(!accel1.begin(0x53))
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 nr1 detected ... Check your wiring!");
    while(1);
  }
  
  /* Set the range and data rate to whatever is appropriate for your project */
  /* See the sensortest example for more details */
  accel1.setRange(ADXL343_RANGE_2_G);

  accel1.setDataRate(ADXL343_DATARATE_1600_HZ);

  /* Display some basic information on these sensors */
  accel1.printSensorDetails();
  Serial.println("");
  Serial.print("Connecting to Adafruit IO DASHBOARD");
  io.connect();

  // Wait for connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  state_=true;
  io.run();
  DATAREADY->save(state_);
  delay(3000);
}

void loop(void)
{
  /* Get new sensor events */
  sensors_event_t event1;
  accel1.getEvent(&event1);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: ");Serial.print(event1.acceleration.x); Serial.print(", ");
  Serial.print("Y: ");Serial.print(event1.acceleration.y); Serial.print(", ");
  Serial.print("Z: ");Serial.print(event1.acceleration.z); Serial.println(".");

  io.run();
  X_->save(event1.acceleration.x);
  Y_->save(event1.acceleration.y);
  Z_->save(event1.acceleration.z);
  delay(5000);
}