#include <OneWire.h>
#include <SevSeg.h>

#define private public  // Silly hack to allow redefining a function in this library
#include <DallasTemperature.h>
#define READSCRATCH     0xBE
#define SCRATCHPAD_CRC  8


////////    Config    ////////

//// Safety values ////
#define TEMP_SAFE_LOWER 10
#define TEMP_SAFE_UPPER 20
#define FLOW_SAFE_LOWER 1.0f
#define FLOW_SAFE_UPPER 2.5f
////////

//// Constants/Settings ////
// #define DEBUG        // Print debug lines to serial, comment out to disable (This may make the display flicker)
// #define PROFILE 10   // Print length of and time between long (defined by PROFILE in ms) display updates to serial, comment out to disable

#define DALLAS_RESOLUTION 10   // The resolution in bits of the DS18B20 sensor value, between 9 and 12 (lower values are maybe? read faster)

#define NTC_EXT_RESISTOR 10000.0f  // The value of the external resistor for the NTC thermistor
#define NTC_RESIST_25C   9324.0f   // The value of the NTC thermistor resistance at 25 Celsius
#define NTC_BETA         3157.0f   // The Beta Coefficient of the NTC thermistor

#define FLOW_MULTIPLIER  2.25f  // mL per pulse of the flow sensor

#define WATER_LEVEL_SENSE LOW  // Signal when water level sensor detects water

#define SENSOR_SMOOTHING_ALPHA  0.2  // Exponential smoothing factor, between 0-1, 1 being no smoothing
#define SENSOR_READ_INTERVAL_MS 500

#define DISPLAY_INTERVAL_MS     2000
#define DISPLAY_BLANK_TIME_MS   200

#define DISPLAY_TYPE       COMMON_CATHODE
#define DISPLAY_BRIGHTNESS 90      // Between -200 and 200. 0 to 100 is the standard range
////////

//// Pins ////
// Most pins can be changed, with the following notes:
//  NTC_PIN is best either A7/A6 because they can't be used as digital pins for the other sensors.
//  FLOW_PIN should be either 2/3 to use the interrupt pins.
//  0/1 are the serial pins, might be able to use them by disabling the usb serial after doing checks.
//  13 is connected to the onboard LED, but I'm using it for the red led since we've run out of other pins.
//    This has the side effect of the onboard LED lighting up on a fault as well.

// NTC Temperature Sensor
//  The simulator board already includes the extra 10k resistor to be wired in series with the sensor.
//  There is the option of using the AREF to get a reference voltage for more accurate readings,
//   but we aren't even displaying any decimal points so probably not needed.
#define NTC_PIN A7

// DS18B20 Temperature Sensor
#define DS18B20_PIN 3

// Flow Sensor
//  Check the accuracy of the flow sensor reading by using a pulse/signal generator to provide a square wave simulated input and vary the frequency.
#define FLOW_PIN 2

// Water Level Sensor
//  https://www.trumsense.com/products/xkc-y25-v-high-low-output-of-capacitive-non-contact-level-sensor-from-trumsense-precision-technology-for-water-tank-water-tower-coffee-machine-100
//  1: Brown wire(VCC) Power +5V~24V
//  2：Yellow wire(OUT) Signal output
//  3: Black wire(M)  Output level(positive output or negative output) control
//     A: Connect to high level, the output is high level when the object is sensed
//     *B: Connect to low level, the output is low level when the object is sensed
//  4: Blue wire(GND) Ground wire
#define WATER_LEVEL_PIN A5

// LEDs
#define GREEN_LED_PIN 4
#define RED_LED_PIN 13

// 7 Segment Display
//  4 Digits w/ Optional Colon (common cathode)
//  Comment out the colon pin definition to disable its use
#define SEG_COLON_PIN A3
const byte digitPins[] = {A0, A1, A2, A4};
const byte segmentPins[] = {10, 12, 6, 8, 9, 11, 5, 7};
////////

////////    End Config    ////////



// Globals
OneWire oneWire(DS18B20_PIN);
DallasTemperature dallasSensors(&oneWire);
DeviceAddress dallasDeviceAddress;

volatile int flowCount;

SevSeg sevseg;
enum DisplayState {
  DisplayBlank,
  DisplayTemp,
  DisplayFlow,
  DisplayFault
};
DisplayState displayState     = DisplayBlank;
DisplayState nextDisplayState = DisplayTemp;

long lastSensorRead         = 0;
long lastSensorReadInterval = 0;
long lastDisplayChange      = 0;

float NTCTemp     = 0.0f;
float dallasTemp  = 0.0f;
float flowLperMin = 0.0f;

bool safeTempNTC    = false;
bool safeTempDallas = false;
bool safeFlow       = false;
bool safeWaterLevel = false;

char faultMessage[5] = "";

#ifdef PROFILE
long lastDisplayRefresh = 0;
long lastDisplayDelay = 0;
#endif
////

float exponentialMovingAverage(float oldValue, float newValue) {
  return (SENSOR_SMOOTHING_ALPHA * newValue) + (1.0 - SENSOR_SMOOTHING_ALPHA) * oldValue;
}

float readNTC() {
  // Return an interger temperature read from NTC_PIN, moving averaged and rounded down
  int analogValue = analogRead(NTC_PIN);
  float voltageValue = 1023.0f / analogValue - 1;
  float celsius = 1 / (log(NTC_EXT_RESISTOR / NTC_RESIST_25C / voltageValue) / NTC_BETA + 1.0 / 298.15f) - 273.15f;
  
  celsius = exponentialMovingAverage(NTCTemp, celsius);
  celsius = constrain(celsius, 0, 99);

  return celsius;
}

float getTempC(const uint8_t* deviceAddress) {
  // Recreated from the library code to be faster since we only have the one onewire device,
  //  and allow refreshing the display between commands.
  DallasTemperature::ScratchPad scratchPad;

  sevseg.refreshDisplay();

  // send the reset command and fail fast
	int b = dallasSensors._wire->reset();
	if (b == 0)
    return DEVICE_DISCONNECTED_RAW;

  sevseg.refreshDisplay();

  // dallasSensors._wire->select(deviceAddress);
  dallasSensors._wire->skip();

  sevseg.refreshDisplay();

  dallasSensors._wire->write(READSCRATCH);

  sevseg.refreshDisplay();

  for (uint8_t i = 0; i < 9; i++) {
    scratchPad[i] = dallasSensors._wire->read();
    sevseg.refreshDisplay();
  }

  b = dallasSensors._wire->reset();

  sevseg.refreshDisplay();

	bool success = (b == 1) && !dallasSensors.isAllZeros(scratchPad) && (dallasSensors._wire->crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);

	if (success) {
    int32_t rawTemp = dallasSensors.calculateTemperature(deviceAddress, scratchPad);
	  return dallasSensors.rawToCelsius(rawTemp);
  }

	return DEVICE_DISCONNECTED_RAW;
}

float readDS18B20() {
  // Return an interger temperature read from DS18B20_PIN, moving averaged and rounded down

  // The library getTempC() takes ~13ms instead of the ~30ms getTempCByIndex() takes, mostly from searching for devices each time
  // TODO: Possibly handle a response of DEVICE_DISCONNECTED_C (-127) when the sensor gives an invalid reading?
  // TODO: getTempC() takes ~13ms in the simulator when a sensor is attached
  //   isConnected() takes ~12ms
  //     OneWire.reset() takes 1ms x 2
  //     OneWire.select() takes 5ms
  //   Potentially can run reset/select only in setup since only using the one i2c device
  //   Or we can run the display refresh in a separate scheduler loop
  // float celsius = dallasSensors.getTempC(dallasDeviceAddress);
  float celsius = getTempC(dallasDeviceAddress);

  // Request the next reading
  dallasSensors.requestTemperatures();

  celsius = exponentialMovingAverage(dallasTemp, celsius);
  celsius = constrain(celsius, 0, 99);

  return celsius;
}

void flowInterrupt() {
  // Interrupt function to run on each pulse of the flow sensor
  flowCount++;
}

float readFlowSensor(long timePassedms) {
  // Return the flow rate in Liters/minute, moving averaged

  // flowRate = flowCount * 1000.0f/timePassedms; // Pulses/second
  // flowRate = flowRate * FLOW_MULTIPLIER;       // mL/second
  // flowRate = flowRate * 60;                    // mL/minute
  // flowRate = flowRate / 1000.0f;               // Liters/minute
  // Simplifies to:
  float flowRate = (flowCount*FLOW_MULTIPLIER*60.0f) / timePassedms;

  flowRate = exponentialMovingAverage(flowLperMin, flowRate);

  flowCount = 0;
  return flowRate;
}

bool readWaterLevel() {
  return digitalRead(WATER_LEVEL_PIN) == WATER_LEVEL_SENSE;
}

void set_colon(bool state) {
#ifdef SEG_COLON_PIN
    digitalWrite(SEG_COLON_PIN, state);
#endif
}

void updateDisplay() {
  if (displayState == DisplayBlank) {
    set_colon(false);
    sevseg.blank();

  } else if (displayState == DisplayTemp) {
    set_colon(true);  // Use colon to separate numbers
    int displayNum = (int)dallasTemp*100 + (int)NTCTemp;  // Both numbers already clamped between 0-99
    sevseg.setNumber(displayNum);

  } else if (displayState == DisplayFlow) {
    set_colon(false);
    sevseg.setNumberF(flowLperMin,2);

  } else if (displayState == DisplayFault) {
    set_colon(false);
    sevseg.setChars(faultMessage);
  }
}


void setup() {
#if defined(DEBUG) || defined(PROFILE)
  Serial.begin(9600);
#endif
  
  // NTC Temperature Sensor

  // DS18B20 Temperature Sensor
  dallasSensors.begin();
  dallasSensors.setResolution(DALLAS_RESOLUTION);
  dallasSensors.setWaitForConversion(false);  // Makes the reading non-blocking, but we have to handle timing
  dallasSensors.getAddress(dallasDeviceAddress, 0);
  dallasSensors.requestTemperatures();

  // Flow sensor
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowInterrupt, RISING);

  // Water Level Sensor
  pinMode(WATER_LEVEL_PIN, INPUT);

  // LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // 7 Segment Display
  #ifdef SEG_COLON_PIN
  pinMode(SEG_COLON_PIN, OUTPUT);
  #endif
  
  byte numDigits = 4;
  bool resistorsOnSegments = false;
  bool updateWithDelays = false;
  bool leadingZeros = true;
  bool disableDecPoint = false;
  sevseg.begin(DISPLAY_TYPE, numDigits, digitPins, segmentPins, resistorsOnSegments,
               updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(DISPLAY_BRIGHTNESS);
}

void loop() {
  // Read sensors and update display
  lastSensorReadInterval = millis() - lastSensorRead;
  if (lastSensorReadInterval > SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = millis();

    safeTempNTC    = false;
    safeTempDallas = false;
    safeFlow       = false;
    safeWaterLevel = false;

    // NTC Temperature Sensor
    NTCTemp = readNTC();
    if (TEMP_SAFE_LOWER < NTCTemp && NTCTemp < TEMP_SAFE_UPPER)
      safeTempNTC = true;

    // DS18B20 Temperature Sensor
    // Only read this sensor if it's next reading is ready
    // In theory up to every 94ms for 9 bit and 750ms for 12 bit resolution
    if (dallasSensors.isConversionComplete())
      dallasTemp = readDS18B20();
    if (TEMP_SAFE_LOWER < dallasTemp && dallasTemp < TEMP_SAFE_UPPER)
      safeTempDallas = true;

    // Flow sensor
    flowLperMin = readFlowSensor(lastSensorReadInterval);
    if (FLOW_SAFE_LOWER < flowLperMin && flowLperMin < FLOW_SAFE_UPPER)
      safeFlow = true;

    // Water Level Sensor
    safeWaterLevel = readWaterLevel();

    // LEDs
    if (safeTempNTC == false || safeTempDallas == false ||
        safeFlow == false || safeWaterLevel == false) {
      // Not within safe ranges!
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
    } else {
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, LOW);
    }

    // 7 Segment Display
    updateDisplay();
    
    // Test prints
#ifdef DEBUG
      Serial.print("NTC: ");
      Serial.print(NTCTemp);
      Serial.print(" Safe:");
      Serial.print(safeTempNTC);
      Serial.print(", Dallas: ");
      Serial.print(dallasTemp);
      Serial.print(" Safe:");
      Serial.print(safeTempDallas);
      Serial.print(", Flow: ");
      Serial.print(flowLperMin);
      Serial.print(" Safe:");
      Serial.print(safeFlow);
      Serial.print(", Level safe: ");
      Serial.print(safeWaterLevel);
      Serial.println("");
#endif
  }

  // Switch display view
  if (displayState == DisplayBlank) {
    if (millis() - lastDisplayChange > DISPLAY_BLANK_TIME_MS) {
      lastDisplayChange = millis();

      displayState = nextDisplayState;
      updateDisplay();
    }

  } else {
    if (millis() - lastDisplayChange > DISPLAY_INTERVAL_MS) {
      // Serial.println(displayState);
      lastDisplayChange = millis();

      if (displayState == DisplayTemp) {
        nextDisplayState = DisplayFlow;

      } else if (displayState == DisplayFault) {
        nextDisplayState = DisplayTemp;

      } else /*if (displayState == DisplayFlow) */{
        // Check for a fault then go to err screen next if found
        nextDisplayState = DisplayFault;

        if (safeWaterLevel == false) {
          strncpy(faultMessage, " LVL", 4);
        } else if (safeFlow == false) {
          strncpy(faultMessage, " FLO", 4);
        } else if (safeTempNTC == false) {
          strncpy(faultMessage, " NTC", 4);
        } else if (safeTempDallas == false) {
          strncpy(faultMessage, " DAL", 4);
        } else {
          // No fault
          nextDisplayState = DisplayTemp;
        }
      }

      // Clear the display for a moment to indicate the view is changing
      displayState = DisplayBlank;
      updateDisplay();
    }
  }

#ifdef PROFILE
  {
    long currTime = millis();
    if (currTime - lastDisplayRefresh > PROFILE) {
      Serial.print("Long display delay: ");
      Serial.print(currTime - lastDisplayRefresh);
      Serial.print("ms, time since last delay: ");
      Serial.print(currTime - lastDisplayDelay);
      Serial.println("ms");
      lastDisplayDelay = currTime;
    }
    lastDisplayRefresh = millis();
  }
#endif
  sevseg.refreshDisplay();
}
