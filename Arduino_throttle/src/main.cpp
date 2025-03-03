#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

// arduino pins for MCP2515 CAN controller
#define CAN_CS_PIN 10  // Chip Select pin
#define CAN_INT_PIN 2  // Interrupt pin

#define GAS_SENSOR_ID 0x201  // 513 decimal (GAS_SENSOR) from dbc file
//NOTE********: upgrade this to parse dbc file directly

// devices
Adafruit_MCP4725 dac;
Adafruit_ADS1115 adc; 
MCP_CAN canBus(CAN_CS_PIN);

// variables
float inputVoltage = 0.0;     // Last read 
float outputVoltage = 0.0;
bool canInitialized = false;  // CAN flag

// debug
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;  // print status every (ms)

void setup() {
  // led for status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  Serial.println("Arduino Throttle Interface Initializing...");

  // initialize DAC
  if (!dac.begin(0x60)) {  // default for MCP4725
    Serial.println("Failed to initialize DAC");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }
  Serial.println("MCP4725 DAC initialized");

  // initialize ADC
  if (!adc.begin(0x48)) {  // default for ADS1115
    Serial.println("Failed to initialize ADC");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  // configure ADC
  adc.setGain(GAIN_ONE);  // +/-4.096V range with 16-bit res
  Serial.println("ADC initialized");

  // init CAN bus at 500 kbps
  if (canBus.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN controller initialized");
    canBus.setMode(MCP_NORMAL);
    canInitialized = true;

    // Set up interrupt for CAN message reception
    if (CAN_INT_PIN >= 0) {
      pinMode(CAN_INT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIsr, FALLING);
      Serial.println("CAN interrupt enabled");
    }
  } else {
    Serial.println(
        "CAN init failed, DAC->ADC instead");
  }

  // init output to 0v
  dac.setVoltage(0, false);

  Serial.println("throttle ready");
}

// can message received interrupt flag
volatile bool canMessageReceived = false;

// can interrupt service routine
void canIsr() { canMessageReceived = true; }

// read input voltage from ADC
float readInputVoltage() {
  int16_t adc0 = adc.readADC_SingleEnded(0);

  // convert to voltage using gain
  float voltage = adc.computeVolts(adc0);

  // stay within 0-5v
  voltage = constrain(voltage, 0.0, 5.0);

  return voltage;
}

// DAC output
void setOutputVoltage(float voltage) {
  voltage = constrain(voltage, 0.0, 5.0);

  // convert voltage to DAC value (12-bit, 0-4095)
  uint16_t dacValue = (voltage / 5.0) * 4095;
  //NOTE*********: upgrade to use const calibration factor unique to each dac
  //NOTE*********: test heavily around top and bottom limits

  // set DAC output
  dac.setVoltage(dacValue, false);

  // Update current output voltage variable
  outputVoltage = voltage;
}

// Function to extract throttle percentage from GAS_SENSOR CAN message
//NOTE*******: upgrade this to only start listening once some openpilot ACC message is sent
//  and disable if ACC is turned off OR the pedal is pressed
float extractThrottlePercentage(unsigned char* data) {
  // GAS_SENSOR signal uses: GAS_COMMAND : 7|16@0+ (0.159375,-75.555) [0|1] "" INTERCEPTOR

  // extract raw value from first two bytes (big endian)
  // left shift 8 first half bitwise OR second half
  uint16_t rawValue = (uint16_t)data[0] << 8 | data[1];

  // scaling factor and offset from DBC (0.159375,-75.555)
  float throttlePercent = (rawValue * 0.159375) - 75.555;

  // keep throttle from 0 to 100
  throttlePercent = constrain(throttlePercent, 0.0, 100.0);

  return throttlePercent;
}

void processCan() {
  if (!canInitialized) return;

  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId;

  // can polling (no interrupt)
  if (canMessageReceived ||
      (CAN_INT_PIN < 0 && canBus.checkReceive() == CAN_MSGAVAIL)) {
    canMessageReceived = false;

    // read incoming
    if (canBus.readMsgBuf(&canId, &len, buf) == CAN_OK) {
      // only process if gas sensor id
      if (canId == GAS_SENSOR_ID) {
        // blink led on message
        digitalWrite(LED_BUILTIN, HIGH);

        float throttlePercent = extractThrottlePercentage(buf);

        // throttle % to volts
        float targetVoltage = (throttlePercent / 100.0) * 5.0;
        setOutputVoltage(targetVoltage);

        // print debug
        Serial.print("GAS_SENSOR: ");
        Serial.print(throttlePercent, 1);
        Serial.print("% throttle -> ");
        Serial.print(targetVoltage, 2);
        Serial.println("V");

        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }
}

void loop() {
  // check can
  processCan();

  inputVoltage = readInputVoltage();

  // if no can, mirror input voltage
  if (!canMessageReceived) {
    setOutputVoltage(inputVoltage);
  }

  // print status every PRINT_INTERVAL
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentMillis;

    Serial.print("Status: Input=");
    Serial.print(inputVoltage, 2);
    Serial.print("V, Output=");
    Serial.print(outputVoltage, 2);
    Serial.println("V");
  }

  // delay to prevent cpu overload
  delay(10);
}