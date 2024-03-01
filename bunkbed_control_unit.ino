#include "hal_conf_custom.h"

#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L0X.h>
#include <ISM330DHCXSensor.h>
#include <PDM.h>
#include <STM32duinoBLE.h>
#include <STTS22HSensor.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <DFRobot_DF2301Q.h>

#include "tlc.h"
#include "types.h"

// Relay pins
#define STAIR_PIN A2
#define BOOKCASE_PIN A4

// RGBW LED pins
#define NOA_PIN A0
#define MAIA_PIN A1
#define NOA_LED_COUNT 20
#define MAIA_LED_COUNT 20

// Bluetooth objects
HCISharedMemTransportClass HCISharedMemTransport;
BLELocalDevice BLEObject(&HCISharedMemTransport);
BLELocalDevice& BLE = BLEObject;
BLEService mainService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 20);
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify, 20);

// Onboard I2C3 and externally available I2C1 objects
TwoWire I2C1Wire(PA10, PB8);
TwoWire I2C3Wire(PB11, PB13);

// Onboard sensors and LED control objects
STTS22HSensor Temperature(&I2C3Wire);
ISM330DHCXSensor AccGyro(&I2C3Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
tlc LED(RGB_LED);

// Touch related objects
TSC_HandleTypeDef htsc;
TSC_IOConfigTypeDef IoConfig;

// Onboard button mapping
#define BUTTON_1 PC12
#define BUTTON_2 PC13

// Onboard display
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(
  U8G2_R0, PA1, PA7, PH0, PC9, PC8
);

// STLink
// HardwareSerial STLink(PIN_SERIAL_RX, PIN_SERIAL_TX);

// DFRobot Offline Voice Recognition sensor externally attached
DFRobot_DF2301Q_I2C DF2301Q(&I2C1Wire);

// Neopixel LED bar setup
Adafruit_NeoPixel maiaLightBar(MAIA_LED_COUNT, MAIA_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel noaLightBar(NOA_LED_COUNT, NOA_PIN, NEO_GRBW + NEO_KHZ800);

// Variables to hold sensor data
float temperature = 0.0; //
int32_t accelerometer[3];
int32_t gyroscope[3];
VL53L0X_RangingMeasurementData_t measure;

// Interrupt wake notification reset during main loop
bool loxWake = false;
bool gyroWake = false;

const uint32_t screenSleepPeriod = 1000 * 60 * 5;  // 5 minute screen inactivity
const uint32_t debounceDelay = 300; // button debounce delay
const uint32_t touchThreshold = 50; // touch threshold (I noticed about an 80-100 difference on my device)
const uint32_t longTouchThresholdMs = 700; // finger hold period to consider it a 'long touch'
const uint32_t numberOfScreens = 5; // total screens
uint32_t currentScreenIndex = 0; // current screen index
uint32_t baselineTouch = 0; // calibrate normal touch level
unsigned long touchTime = 0; // track when touch began for length determination
bool isTouching = false; // whether finger is currently mid touch
bool longTouchEventTriggered = false; // whether this long touch period has already triggered an event to prevent multiple long touches
unsigned long lastButtonEventTime = 0; // tracking last button press for debouncing
unsigned long lastInterrupt = 0; // time of the last interrupt used for screen sleep

// Light state
bool noaLight = false;
uint8_t noaLightRGBIndex = 0;
uint8_t noaLightRGB[4] = {0,0,0,255};
bool maiaLight = false;
uint8_t maiaLightRGBIndex = 0;
uint8_t maiaLightRGB[4] = {0,0,0,255};
bool bookcaseLight = false;
bool stairLight = false;

// Predefined color loop and RGBW color definitions
const uint8_t rgbColorCount = 8;
uint8_t rgbColors[][4] = {
    {0, 0, 0, 255},      // WhiteColor
    {255, 0, 0, 0},      // RedColor
    {255, 165, 0, 0},    // OrangeColor
    {255, 255, 0, 0},    // YellowColor
    {0, 255, 0, 0},      // GreenColor
    {0, 255, 255, 0},    // CyanColor
    {0, 0, 255, 0},      // BlueColor
    {128, 0, 128, 0}     // PurpleColor
};
// RGB white color
uint8_t whiteColorRGB[3] = {255, 255, 255};


void toggleLight();
void toggleLightColor();
void setLightColorRGB(uint8_t rgbColor[4]);
void setLightColor(int colorIndex);
void enableLight();
void disableLight();
void updateLightState(screen_t screen, bool updateOnboardLED = true);

/// Celsius to Fahrenheit conversion
float celsiusToFahrenheit(float temperatureCelsius) {
  return (temperatureCelsius * 9.0 / 5.0) + 32.0;
}

/// ISM330DHCX callback
void gyroCallback() {
  gyroWake = true;
  lastInterrupt = millis();
}

/// VL53L0X callback
void loxCallback() {
  loxWake = true;
  lox.clearInterruptMask(false);
  lastInterrupt = millis();
}

/// TSC IRQ Handler
extern "C" {
  void TSC_IRQHandler(void) {
    HAL_TSC_IRQHandler(&htsc);
  }
}

/// BLE write event callback
void bleWriteEvent(BLEDevice central, BLECharacteristic characteristic) {
  byte value = 0;
  uint8_t packetBuffer[characteristic.valueLength()];
  characteristic.readValue(packetBuffer, characteristic.valueLength());
  
  // Command byte found
  if (packetBuffer[0] == '!') {
    characteristic.readValue(value);
    switch(packetBuffer[1]) {
      case 'C': {
        characteristic.readValue(packetBuffer, 3);

        uint8_t r = packetBuffer[2];
        uint8_t g = packetBuffer[3];
        uint8_t b = packetBuffer[4];

        if (r == 255 && g == 255 && b == 255) {
          uint8_t lightRGB[4] = {0, 0, 0, 255};
          setLightColorRGB(lightRGB);
        } else {
          uint8_t lightRGB[4] = {r, g, b, 0};
          setLightColorRGB(lightRGB);
        }
        if(r == 0 && g == 0 && b == 0) {
          disableLight();
        } else {
          enableLight();
        }
        
        screen_t currentScreen = screensArray[currentScreenIndex];
        updateLightState(currentScreen);
        break;
      }
      case 'B': {
        characteristic.readValue(packetBuffer, 2);
        uint8_t button = packetBuffer[2] - '0';
        bool pressed = packetBuffer[3] - '0';

        if (button == 1) {
          currentScreenIndex = SCREEN_NOA;
        } else if (button == 2) {
          currentScreenIndex = SCREEN_MAIA;
        } else if (button == 3) {
          currentScreenIndex = SCREEN_BOOKCASE;
        } else if (button == 4) {
          currentScreenIndex = SCREEN_STAIRS;
        }
        break;
      }
      default:
        break;
    }
  }
}

void setup() {
  // Begin STLink connection for output
  //STLink.begin(115200);

  // Start i2c at maximum frequency of 400 kHz
  // I2C1 is the user i2c bus, I2C3 is on the onboard i2c bus used by sensors
  I2C1Wire.begin();
  I2C1Wire.setClock(400000);
  I2C3Wire.begin();
  I2C3Wire.setClock(400000);

  // DF2301Q voice module begin
  if( !( DF2301Q.begin() ) ) {
    //STLink.println("Failure to initialize DF2301Q Voice Module");
  }

  // Temperature enable
  Temperature.begin();
  Temperature.Enable();

  // Configure motion detection interrupt
  AccGyro.begin();
  AccGyro.ACC_Enable();  
  AccGyro.ACC_EnableWakeUpDetection(ISM330DHCX_INT1_PIN);
  AccGyro.GYRO_Enable();
  attachInterrupt(PD2, gyroCallback, RISING);

  // Configure VL53L0X time of flight device with interrupt for wake
  lox.begin(VL53L0X_I2C_ADDR, false, &I2C3Wire);
  pinMode(PD9, INPUT_PULLUP);
  attachInterrupt(PD9, loxCallback, CHANGE);
  lox.setGpioConfig(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                    VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
                    VL53L0X_INTERRUPTPOLARITY_LOW);
  FixPoint1616_t lowThreshold = (4000 * 65536.0); // 4000mm
  FixPoint1616_t highThreshold = (4000 * 65536.0); // 4000mm
  lox.setInterruptThresholds(lowThreshold, highThreshold, false);
  lox.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, false);
  lox.startMeasurement();
  
  // Button
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // LED select and onboard RGB LED as outputs
  pinMode(LED_SELECT, OUTPUT);
  pinMode(RGB_LED, OUTPUT);

  // Start the display
	u8g2.begin();
  u8g2.clear();

  // Setup BLE elements
  if (!BLE.begin()) {
    //STLink.println("Failure to initialize bluetooth");
  }
  BLE.setDeviceName("BCU-STM32");
  BLE.setLocalName("Bunkbed Control Unit");
  BLE.setAdvertisedService(mainService);
  mainService.addCharacteristic(rxChar);
  rxChar.setEventHandler(BLEWritten, bleWriteEvent);
  mainService.addCharacteristic(txChar);
  BLE.addService(mainService);
  BLE.advertise();

  // TSC pin setup
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TSC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TSC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TSC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TSC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* TSC interrupt Init */  
  __HAL_RCC_TSC_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // TSC setup
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_1CYCLE;
  htsc.Init.CTPulseLowLength = TSC_CTPL_1CYCLE;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = 0;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = 0;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(TSC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TSC_IRQn);
  IoConfig.ChannelIOs = TSC_GROUP6_IO2;
  IoConfig.ShieldIOs = TSC_GROUP4_IO2;
  IoConfig.SamplingIOs = TSC_GROUP4_IO1|TSC_GROUP6_IO1;
  if (HAL_TSC_IOConfig(&htsc, &IoConfig) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  // Light pin definitions
  pinMode(NOA_PIN, OUTPUT);
  pinMode(MAIA_PIN, OUTPUT);
  pinMode(STAIR_PIN, OUTPUT);
  pinMode(BOOKCASE_PIN, OUTPUT);
  maiaLightBar.begin();
  noaLightBar.begin();
}

void renderHome() {
  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  
  if (stairLight) {
	  u8g2.drawGlyph(5, 35, 64 + 2);
  } else {
	  u8g2.drawGlyph(5, 35, 64 + 1);
  }

  if (bookcaseLight) {
	  u8g2.drawGlyph(5, 60, 64 + 2);
  } else {
	  u8g2.drawGlyph(5, 60, 64 + 1);
  }
  
  u8g2.setFont(u8g2_font_streamline_phone_t);
	u8g2.drawGlyph(30, 35, 48 + 8);
  u8g2.setFont(u8g2_font_streamline_content_files_t);
	u8g2.drawGlyph(30, 60, 48 + 5);

  u8g2.setFont(u8g2_font_mystery_quest_24_tr);
	u8g2.drawStr(80, 30, "N"); 
	u8g2.drawStr(80, 55, "M");

  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  if (noaLight) {
	  u8g2.drawGlyph(105, 35, 64 + 2);
  } else {
	  u8g2.drawGlyph(105, 35, 64 + 1);
  }
  if (maiaLight) {
	  u8g2.drawGlyph(105, 60, 64 + 2);
  } else {
	  u8g2.drawGlyph(105, 60, 64 + 1);
  }
}

void renderBookcase() {
  u8g2.setFont(u8g2_font_mystery_quest_24_tr);
	u8g2.drawStr(5, 30, "Bookcase"); 
  char temperatureStr[10];
  dtostrf(celsiusToFahrenheit(temperature), 3, 0, temperatureStr);
  u8g2.drawStr(2, 58, temperatureStr);
  
  u8g2.setFont(u8g2_font_streamline_weather_t);
	u8g2.drawGlyph(35, 60, 48 + 6);

  u8g2.setFont(u8g2_font_streamline_content_files_t);
	u8g2.drawGlyph(80, 60, 48 + 5);
  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  if (bookcaseLight) {
	  u8g2.drawGlyph(105, 60, 64 + 2);
  } else {
	  u8g2.drawGlyph(105, 60, 64 + 1);
  }
}

void renderStairs() {
  u8g2.setFont(u8g2_font_mystery_quest_24_tr);
	u8g2.drawStr(5, 30, "Stairs"); 
  char temperatureStr[10];
  dtostrf(celsiusToFahrenheit(temperature), 3, 0, temperatureStr);
  u8g2.drawStr(2, 58, temperatureStr);
  
  u8g2.setFont(u8g2_font_streamline_weather_t);
	u8g2.drawGlyph(35, 60, 48 + 6);

  u8g2.setFont(u8g2_font_streamline_phone_t);
	u8g2.drawGlyph(80, 60, 48 + 8);
  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  if (stairLight) {
	  u8g2.drawGlyph(105, 60, 64 + 2);
  } else {
	  u8g2.drawGlyph(105, 60, 64 + 1);
  }
}

void renderNoa() {
  u8g2.setFont(u8g2_font_mystery_quest_24_tr);
	u8g2.drawStr(5, 30, "Noa's Bed"); 
  
  u8g2.setFont(u8g2_font_streamline_pet_animals_t);
	u8g2.drawGlyph(5, 60, 48 + 10);
  u8g2.setFont(u8g2_font_streamline_entertainment_events_hobbies_t);
	u8g2.drawGlyph(30, 60, 48 + 12);
  u8g2.setFont(u8g2_font_streamline_health_beauty_t);
	u8g2.drawGlyph(55, 60, 48 + 7);

  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  if (noaLight) {
	  u8g2.drawGlyph(105, 60, 64 + 2);
  } else {
	  u8g2.drawGlyph(105, 60, 64 + 1);
  }
}

void renderMaia() {
  u8g2.setFont(u8g2_font_mystery_quest_24_tr);
	u8g2.drawStr(5, 30, "Maia's Bed");
  
  u8g2.setFont(u8g2_font_streamline_pet_animals_t);
	u8g2.drawGlyph(5, 60, 48 + 2);
  u8g2.setFont(u8g2_font_streamline_social_rewards_t);
	u8g2.drawGlyph(30, 60, 48 + 10);
  u8g2.setFont(u8g2_font_streamline_photography_t);
	u8g2.drawGlyph(55, 60, 48 + 3);

  u8g2.setFont(u8g2_font_streamline_interface_essential_other_t);
  if (maiaLight) {
    u8g2.drawGlyph(105, 60, 64 + 2);
  } else {
    u8g2.drawGlyph(105, 60, 64 + 1);
  }
}

void renderScreen() {
	u8g2.clearBuffer();
  screen_t currentScreen = screensArray[currentScreenIndex];
  switch(currentScreen) {
    case SCREEN_HOME:
      renderHome();
      break;
    case SCREEN_BOOKCASE:
	    renderBookcase();
      break;
    case SCREEN_STAIRS:
      renderStairs();
      break;
    case SCREEN_NOA:
      renderNoa();
      break;
    case SCREEN_MAIA:
      renderMaia();
      break;
  }
  u8g2.sendBuffer();
}

void fetchSensorData() {
  if (loxWake || gyroWake) {
    //STLink.print("Motion ");
    if (loxWake) {
      //STLink.print("(LOX) ");
    } else if (gyroWake) {
      //STLink.print("(GYRO) ");
    }
  } else {
    //STLink.print("No Motion ");
  }

  loxWake = false;
  gyroWake = false;

  lox.getRangingMeasurement(&measure, false); 
  if (measure.RangeStatus != 4) { // phase failures have incorrect data
    //STLink.print("Distance (mm): ");
    //STLink.println(measure.RangeMilliMeter);
  }

  AccGyro.ACC_GetAxes(accelerometer);  
  //STLink.printf("A: %d, %d, %d ", accelerometer[0], accelerometer[1], accelerometer[2]);
  AccGyro.GYRO_GetAxes(gyroscope);
  //STLink.printf("G: %d, %d, %d ", gyroscope[0], gyroscope[1], gyroscope[2]);
  Temperature.GetTemperature(&temperature);
  //STLink.printf("T: %dC %dF\n", (int) temperature, (int) celsiusToFahrenheit(temperature));
}

void updateOnboardLED(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(LED_SELECT, HIGH);
  LED.writeLed(r, g, b, true);
  digitalWrite(LED_SELECT, LOW);
}

void updateOnboardLEDState(screen_t screen) {
  switch(screen) {
    case SCREEN_HOME: {
      bool lights = bookcaseLight && stairLight && noaLight && maiaLight;
      if (lights) {
        updateOnboardLED(whiteColorRGB[0], whiteColorRGB[1], whiteColorRGB[2]);
      } else {
        updateOnboardLED(0, 0, 0);
      }
      break;
    }
    case SCREEN_BOOKCASE: {
      if (bookcaseLight) {
        updateOnboardLED(whiteColorRGB[0], whiteColorRGB[1], whiteColorRGB[2]);
      } else {
        updateOnboardLED(0, 0, 0);
      }
      break;
    }
    case SCREEN_STAIRS: {
      if (stairLight) {
        updateOnboardLED(whiteColorRGB[0], whiteColorRGB[1], whiteColorRGB[2]);
      } else {
        updateOnboardLED(0, 0, 0);
      }
      break;
    }
    case SCREEN_NOA: {
      if (noaLight) {
        if (noaLightRGB[0] == 0 && noaLightRGB[1] == 0 && noaLightRGB[2] == 0 && noaLightRGB[3] == 255) {
          // Onboard is RGB, LED strip is RGBW so alternate white approaches are needed
          updateOnboardLED(whiteColorRGB[0], whiteColorRGB[1], whiteColorRGB[2]);
        } else {   
          updateOnboardLED(noaLightRGB[0], noaLightRGB[1], noaLightRGB[2]);
        }
      } else {
        updateOnboardLED(0, 0, 0);
      }
      break;
    }
    case SCREEN_MAIA: {
      if (maiaLight) {
        if (maiaLightRGBIndex == 0) {
          // Onboard is RGB, LED strip is RGBW so alternate white approaches are needed
          updateOnboardLED(whiteColorRGB[0], whiteColorRGB[1], whiteColorRGB[2]);
        } else {   
          updateOnboardLED(maiaLightRGB[0], maiaLightRGB[1], maiaLightRGB[2]);
        }
      } else {
        updateOnboardLED(0, 0, 0);
      }
      break;
    }
  }
}

void updateLightState(screen_t screen, bool updateOnboardLED) {
  if (updateOnboardLED) {
    updateOnboardLEDState(screen);
  }
  switch(screen) {
    case SCREEN_HOME: {
      updateLightState(SCREEN_BOOKCASE, false);
      updateLightState(SCREEN_STAIRS, false);
      updateLightState(SCREEN_NOA, false);
      updateLightState(SCREEN_MAIA, false);
      break;
    }
    case SCREEN_BOOKCASE: {
      if (bookcaseLight) {
        digitalWrite(BOOKCASE_PIN, HIGH);
      } else {
        digitalWrite(BOOKCASE_PIN, LOW);
      }
      break;
    }
    case SCREEN_STAIRS: {
      if (stairLight) {
        digitalWrite(STAIR_PIN, HIGH);
      } else {
        digitalWrite(STAIR_PIN, LOW);
      }
      break;
    }
    case SCREEN_NOA: {
      uint32_t color = noaLightBar.Color(noaLightRGB[0], noaLightRGB[1], noaLightRGB[2], noaLightRGB[3]);
      noaLightBar.clear();    
      if (noaLight) {
        noaLightBar.fill(color, 0, NOA_LED_COUNT);
      }  
      noaLightBar.show();
      break;
    }
    case SCREEN_MAIA: {
      uint32_t color = maiaLightBar.Color(maiaLightRGB[0], maiaLightRGB[1], maiaLightRGB[2], maiaLightRGB[3]);
      maiaLightBar.clear();
      if (maiaLight) {
        maiaLightBar.fill(color, 0, MAIA_LED_COUNT);
      }
      maiaLightBar.show();
      break;
    }
    default:
    break;
  }
}

void toggleLight() {
  screen_t currentScreen = screensArray[currentScreenIndex];
  switch(currentScreen) {
    case SCREEN_HOME: {
      bool lights = bookcaseLight && stairLight && noaLight && maiaLight;
      bookcaseLight = !lights;
      stairLight = !lights;
      noaLight = !lights;
      maiaLight = !lights;
      break;
    }
    case SCREEN_BOOKCASE:
      bookcaseLight = !bookcaseLight;
      break;
    case SCREEN_STAIRS:
      stairLight = !stairLight;
      break;
    case SCREEN_NOA:
      noaLight = !noaLight;
      break;
    case SCREEN_MAIA:
      maiaLight = !maiaLight;
      break;
  }
}

void toggleLightColor() {
  screen_t currentScreen = screensArray[currentScreenIndex];
  if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_MAIA || currentScreen == SCREEN_HOME) {
    enableLight();

    if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_HOME) {
      noaLightRGBIndex = (noaLightRGBIndex + 1) % rgbColorCount;
      noaLightRGB[0] = rgbColors[noaLightRGBIndex][0];
      noaLightRGB[1] = rgbColors[noaLightRGBIndex][1];
      noaLightRGB[2] = rgbColors[noaLightRGBIndex][2];
      noaLightRGB[3] = rgbColors[noaLightRGBIndex][3];
    }
    if (currentScreen == SCREEN_MAIA || currentScreen == SCREEN_HOME) {
      maiaLightRGBIndex = (maiaLightRGBIndex + 1) % rgbColorCount;
      maiaLightRGB[0] = rgbColors[maiaLightRGBIndex][0];
      maiaLightRGB[1] = rgbColors[maiaLightRGBIndex][1];
      maiaLightRGB[2] = rgbColors[maiaLightRGBIndex][2];
      maiaLightRGB[3] = rgbColors[maiaLightRGBIndex][3];
    }
  }
}

void setLightColorRGB(uint8_t rgbColor[4]) {
  screen_t currentScreen = screensArray[currentScreenIndex];
  if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_MAIA || currentScreen == SCREEN_HOME) {
    if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_HOME) {
      noaLightRGB[0] = rgbColor[0];
      noaLightRGB[1] = rgbColor[1];
      noaLightRGB[2] = rgbColor[2];
      noaLightRGB[3] = rgbColor[3];
    } 
    if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_HOME) {
      maiaLightRGB[0] = rgbColor[0];
      maiaLightRGB[1] = rgbColor[1];
      maiaLightRGB[2] = rgbColor[2];
      maiaLightRGB[3] = rgbColor[3];
    }
  }
}

void setLightColor(int colorIndex) {
  screen_t currentScreen = screensArray[currentScreenIndex];
  if (currentScreen == SCREEN_NOA || currentScreen == SCREEN_MAIA) {
    enableLight();

    if (currentScreen == SCREEN_NOA) {
      noaLightRGBIndex = colorIndex;
      setLightColorRGB(rgbColors[noaLightRGBIndex]);
    } else {
      maiaLightRGBIndex = colorIndex;
      setLightColorRGB(rgbColors[maiaLightRGBIndex]);
    }
  }
}

void enableLight() {
  screen_t currentScreen = screensArray[currentScreenIndex];
  switch(currentScreen) {
    case SCREEN_HOME:
      bookcaseLight = true;
      stairLight = true;
      noaLight = true;
      maiaLight = true;
      break;
    case SCREEN_BOOKCASE:
      bookcaseLight = true;
      break;
    case SCREEN_STAIRS:
      stairLight = true;
      break;
    case SCREEN_NOA:
      noaLight = true;
      break;
    case SCREEN_MAIA:
      maiaLight = true;
      break;
  }
}

void disableLight() {
  screen_t currentScreen = screensArray[currentScreenIndex];
  switch(currentScreen) {
    case SCREEN_HOME:
      bookcaseLight = false;
      stairLight = false;
      noaLight = false;
      maiaLight = false;
      break;
    case SCREEN_BOOKCASE:
      bookcaseLight = false;
      break;
    case SCREEN_STAIRS:
      stairLight = false;
      break;
    case SCREEN_NOA:
      noaLight = false;
      break;
    case SCREEN_MAIA:
      maiaLight = false;
      break;
  }
}

void processVoiceCommand(uint8_t commandValue) {
    voice_command_t command = (voice_command_t)commandValue;

    switch (command) {
        case CustomCommand1: {
            //STLink.println("Noa mode");
            currentScreenIndex = SCREEN_NOA;
            break;
        }
        case CustomCommand2: {
            //STLink.println("Maia mode");
            currentScreenIndex = SCREEN_MAIA;
            break;
        }
        case CustomCommand3:  {
            //STLink.println("Book mode");
            currentScreenIndex = SCREEN_BOOKCASE;
            break;
        }
        case CustomCommand4:  {
            //STLink.println("Stair mode");
            currentScreenIndex = SCREEN_STAIRS;
            break;
        }
        case TurnOnTheLight: {
            //STLink.println("Light on");
            enableLight();
            screen_t currentScreen = screensArray[currentScreenIndex];
            updateLightState(currentScreen);
            break;
        }
        case TurnOffTheLight: {
            //STLink.println("Light off");
            disableLight();
            break;
        }
        case SetToRed: {
            //STLink.println("Setting light to red");
            setLightColor(1);
            break;
        }
        case SetToOrange: {
            //STLink.println("Setting light to orange");
            setLightColor(2);
            break;
        }
        case SetToYellow: {
            //STLink.println("Setting light to yellow");
            setLightColor(3);
            break;
        }
        case SetToGreen: {
            //STLink.println("Setting light to green");
            setLightColor(4);
            break;
        }
        case SetToCyan: {
            //STLink.println("Setting light to cyan");
            setLightColor(5);
            break;
        }
        case SetToBlue: {
            //STLink.println("Setting light to blue");
            setLightColor(6);
            break;
        }
        case SetToPurple: {
            //STLink.println("Setting light to purple");
            setLightColor(7);
            break;
        }
        case SetToWhite: {
            //STLink.println("Setting light to white");
            setLightColor(8);
            break;
        }
        default:
            break;
    }

    screen_t currentScreen = screensArray[currentScreenIndex];
    updateLightState(currentScreen);
}

void processInput() {
  int currentTime = millis();
  // Process input buttons for screen management
  if ((currentTime - lastButtonEventTime) > debounceDelay) {
    if (digitalRead(BUTTON_1) == LOW) {
      lastButtonEventTime = currentTime;
      currentScreenIndex = (currentScreenIndex + numberOfScreens - 1) % numberOfScreens;
      screen_t currentScreen = screensArray[currentScreenIndex];
      updateLightState(currentScreen);
    } else if (digitalRead(BUTTON_2) == LOW) {
      lastButtonEventTime = currentTime;
      currentScreenIndex = (currentScreenIndex + 1) % numberOfScreens;
      screen_t currentScreen = screensArray[currentScreenIndex];
      updateLightState(currentScreen);
    }
  }

  uint32_t tscStatus;
  uint32_t tscGroupStatus;
  uint32_t tscValue;
  tscStatus = HAL_TSC_IODischarge(&htsc,ENABLE); // DISCHARGE STATUS
  HAL_Delay(1);
  tscStatus = HAL_TSC_Start_IT(&htsc); // START STATUS
  tscStatus = HAL_TSC_PollForAcquisition(&htsc); // POLL STATUS
  tscGroupStatus = HAL_TSC_GroupGetStatus(&htsc,TSC_GROUP6_IDX); // GROUP STATUS
  tscValue = HAL_TSC_GroupGetValue(&htsc,TSC_GROUP6_IDX); // VALUE
  baselineTouch = max<uint32_t>(baselineTouch, tscValue);
  
  bool currentlyTouching = baselineTouch > 0 && tscValue <= baselineTouch - touchThreshold;
  bool longTouch = false;
  bool shortTouch = false;

  // Logic based on "isTouching" previous touch state
  if (isTouching) {
    // If touching and we pass the long touch threshold register a long touch and restart touch process
      if ((currentTime - touchTime) > longTouchThresholdMs) {
        if (!longTouchEventTriggered) {
          longTouchEventTriggered = longTouch = true;
        }
      } else {
        // Check if touch has ended
        if (!currentlyTouching) {
          // Check if this is within the debounce time and ignore event if so
          if ((currentTime - lastButtonEventTime) > debounceDelay) {
            shortTouch = true;
          }

          longTouchEventTriggered = false;
          touchTime = 0;
        }
      }
  } else {
    longTouchEventTriggered = false;
    
    // Not previously touching
    if (currentlyTouching) {
      // Touch start, set touch start and state
      touchTime = millis();
    }
  }
  isTouching = currentlyTouching;

  if (shortTouch || longTouch) {
    lastButtonEventTime = millis();
  }

  screen_t currentScreen = screensArray[currentScreenIndex];
  if (shortTouch) {
    //STLink.println("Short touch detected");
    toggleLightColor();
    updateLightState(currentScreen);
  } else if (longTouch) {
    //STLink.println("Long touch detected");
    toggleLight();
    updateLightState(currentScreen);
  }

  uint8_t voiceCommandId = DF2301Q.getCMDID();
  if(voiceCommandId != Silence) {
    //STLink.print("Voice Command = ");
    //STLink.println(voiceCommandId);
    processVoiceCommand(voiceCommandId);
  }
}

void loop() {
  processInput();
  fetchSensorData();

  // If we have not had an interrupt for 5 minutes and no lights are enabled
  // stop rendering the screen so the room can darken
  bool lights = bookcaseLight && stairLight && noaLight && maiaLight;
  int currentTime = millis();
  if ((currentTime - lastInterrupt) < screenSleepPeriod || lights) {
    renderScreen();
  } else {
    u8g2.clear();
  }

  BLE.poll();
}