#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <ArduinoBLE.h>

BLEService weatherService("180F");
BLEStringCharacteristic weatherChar("2A19", BLERead | BLENotify, 30);
String oldSensorLevel = "0,0,0,0,0,0,0";  // last sensor readings
long previousMillis = 0;  // last time the sensor level was read, in ms

int RainSensor   = A2;

const int WindRecordTime = 3; //Measuring Time (Seconds)
const int windSensorPin  = D3;  //Interrupt Pin
int InterruptCounter;
  
int winddirection=A1;

void setup() {
  Serial.begin(9600);

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor.");
  }
  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  
  BLE.setLocalName("WeatherMonitor");
  BLE.setAdvertisedService(weatherService); // add the service UUID
  weatherService.addCharacteristic(weatherChar); // add the battery level characteristic
  BLE.addService(weatherService); // Add the battery service
  weatherChar.writeValue(oldSensorLevel); // set initial value for this characteristic

  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  int R, T, H, P, r, ws, wd;
  BLEDevice central = BLE.central();
  if (central) {
    // connected to base station
    Serial.print("Connected to central: ");

    // print the base station BT address:
    Serial.println(central.address());
  
    // check the sensor level every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the sensor level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;

        R = (int)get_colour_R();
        T = (int)get_temperature();
        H = (int)get_humidity();
        P = (int)get_pressure();
        r = (int)get_rain();
        ws = (int)get_wind();
        wd = (int)get_windDirection();

        updateSensorLevel(R, T, H, P, r, ws, wd);
        Serial.println("");
      }
    }
  }
}

void updateSensorLevel(int R, int T, int H, int P, int r, int ws, int wd) {
  String sensorLevel = String(R) + "," + String(T) + "," + 
                        String(H) + "," + String(P) + "," + 
                        String(r) + "," + String(ws) + "," +
                        String(wd);
  Serial.print("Sensor Level % is now: "); // print sensor readings
  Serial.println(sensorLevel);
  weatherChar.writeValue(sensorLevel);  // and update the weather level characteristic
}

// Get the RGB - R value. 
int get_colour_R() {
  // check if a color reading is available
  while (! APDS.colorAvailable()) {
    delay(5);
  }
  int r, g, b;
  APDS.readColor(r, g, b);

  Serial.print("RGB - R : ");
  Serial.println(r);

  return r;
}

// Get the temperature value. 
float get_temperature() {
  float temperature = HTS.readTemperature();

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");
  return temperature;

}

// Get the humidity value. 
float get_humidity() {
  float humidity    = HTS.readHumidity();

  Serial.print("Humidity    = ");
  Serial.print(humidity);
  Serial.println(" %");
  return humidity;
}

// Get the pressure value. 
float get_pressure() {
  float pressure = BARO.readPressure();

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" kPa");
  return pressure;
}

// Get the rain value. 
int get_rain() {
  int value = analogRead(RainSensor);//Reads the Value of RainSensor.
  Serial.print("Rain - RainSensor value is :");
  Serial.println(value);
  return value;
}

// Get the wind speed value. 
float get_wind() {
  //https://www.aeq-web.com/arduino-anemometer-wind-sensor/
  float WindSpeed = wind_meassure();
  Serial.print("Wind Speed: ");
  Serial.print(WindSpeed);       //Speed in km/h
  Serial.println(" km/h");
  return WindSpeed;
}


float wind_meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(windSensorPin), countup, RISING);
  delay(1000 * WindRecordTime);
  detachInterrupt(digitalPinToInterrupt(windSensorPin));
  float WindSpeed = (float)InterruptCounter / (float)WindRecordTime * 2.4;
  return WindSpeed;
}

void countup() {
  InterruptCounter++;
}

// Get the wind direction value. 
int get_windDirection() {
  int news = analogRead(winddirection);//Reads the Value of LDR(light).
  
  Serial.print("winddirection value is :");//Prints the value of LDR to Serial Monitor.
  Serial.println(news);
  return news;
}
