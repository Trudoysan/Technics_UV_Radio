#if !defined(nullptr)
#define nullptr NULL
#endif
#include "Arduino.h"
#include <WiFi.h>
#include "Audio.h"
//#include "FS.h"
#include <Preferences.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
#include <arduinoFFT.h>
//#include <SPI.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"


//#define ADAFRUIT

#ifdef ADAFRUIT
#include <Adafruit_NeoPixel.h>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp32-hal.h"
#endif

//#define DEBUG

//BLE - ON/OFF (BLUE)
//VU -  bar level  up, down, on/off by switch, Song name ON/OFF, Station Name ON/OFF
//radio - ON/OFF(YELLOW/other), BAR(GREEN), TOP (RED), station up, down

//#define AUDIO_IN_PIN 36  // Signal in on this pin - ADC1_CHANNEL_0
#define WS_PIN 13

#define ST_UP_PIN 32     // In.  OK_OK_OK_OK
#define ST_DOWN_PIN 33   // In  OK_OK_OK_OK
#define LVL_UP_PIN 5     // In xx
#define LVL_DOWN_PIN 21  // In xx 4 nebo 5
int buttonPins[] = { ST_UP_PIN, ST_DOWN_PIN, LVL_UP_PIN, LVL_DOWN_PIN };
#define NUM_BUTTONS 4

#define SONG_PIN 15  // In  OK_OK_OK_OK
//define STATION_PIN 39  // In

#define VU_PIN 14      // In -  OK_OK_OK_OK
#define VU_LED_PIN 12  // Out -  OK_OK_OK_OK

#define TOP_PIN 18      // In -  OK_OK_OK_OK
#define TOP_LED_PIN 19  // Out -  OK_OK_OK_OK

#define BAR_PIN 22      // In  -  OK_OK_OK_OK
#define BAR_LED_PIN 23  // Out -  OK_OK_OK_OK

#define RADIO_PIN 16      // In
#define RADIO_LED_PIN 17  // Out

// Digital I/O used
#define I2S_DOUT 26  // DIN connection
#define I2S_BCLK 27  // Bit clock
#define I2S_LRC 25   // Left Right Clock

#define SAMPLES 1024  // Must be a power of 2
//#define SAMPLES 128          // Must be a power of 2
#define SAMPLING_FREQ 40000  // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
int amplitude = 10000;       // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.

Audio audio;

Preferences pref;
int infolnc = 0;         //stores channel number for tuner
int ledBrightness = 32;  //store led Brightness, maybe different for different colours?

bool bar = 1, top = 1, radioOn, VUon, songName;

#define NUM_BANDS 10  // To change this, you will need to change the bunch of if statements describing the mapping from bins to bands
#define NOISE 500     // Used as a crude noise filter, values below this are ignored

//const uint8_t kMatrixWidth = 16;                 // Matrix width
//const uint8_t kMatrixHeight = 16;                // Matrix height
const uint8_t kMatrixWidth = NUM_BANDS;          // Matrix width
const uint8_t kMatrixHeight = 18;                // Matrix height
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)  // Total number of LEDs

#define TOP (kMatrixHeight - 0)  // Don't allow the bars to go offscreen

#ifdef ADAFRUIT
Adafruit_NeoPixel pixels(NUM_LEDS / 3, WS_PIN, NEO_GRB + NEO_KHZ800);
#else

#define NR_OF_ALL_BITS 24 * (NUM_LEDS / 3)
rmt_data_t led_data[NR_OF_ALL_BITS];
rmt_obj_t *rmt_send = NULL;
#endif

// Sampling and FFT stuff
unsigned int sampling_period_us;
byte peak[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // The length of these arrays must be >= NUM_BANDS
int oldBarHeights[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime;
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);


#define SSID "****"
#define PSK "****"

#define STATIONS 15
char *stationlist[STATIONS] = {
  "http://streaming.live365.com/b92108_128mp3",
  "https://panel.retrolandigital.com/listen/oldies_internet_radio/listen",
  "http://vis.media-ice.musicradio.com/CapitalMP3",
  //"https://panel.retrolandigital.com/listen/80s_online_radio/listen",
  "https://radiorock.stream.laut.fm/radiorock",
  "https://media-ssl.musicradio.com/SmoothLondonMP3",
  "https://online.radioroks.ua/RadioROKS",
  //"http://airspectrum.cdnstream1.com:8114/1648_128",
  //"airspectrum.cdnstream1.com:8000/1261_192",
  "http://stream.1a-webradio.de/saw",
  "http://stream.antenne.de/antenne",
  "https://radiopanther.radiolebowski.com/play",
  "https://media-ssl.musicradio.com/Heart70sMP3",
  "http://1000oldies.stream.laut.fm/1000oldies",
  "http://webradio.classicfm.dk/classichorsens",
  "http://ice55.securenetsystems.net/DASH26",
  "https://samcloud.spacial.com/api/listen?sid=111415&rid=194939&f=mp3,any&br=128000,any&m=sc",
  "https://0n-80s.radionetz.de/0n-80s.mp3"
};

int currCulNo = 0;
char msg[50];

TaskHandle_t TaskAudio;
void TaskAudiocode(void *pvParameters) {
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // 0...21
  if (!radioOn)
    audio.connecttohost(stationlist[infolnc]);

  for (;;) {
    buttonCheck();
    audio.loop();
  }
}

void feedTheDog() {
  // feed dog 0
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;  // write enable
  TIMERG0.wdt_feed = 1;                        // feed dog
  TIMERG0.wdt_wprotect = 0;                    // write protect
  // feed dog 1
  TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;  // write enable
  TIMERG1.wdt_feed = 1;                        // feed dog
  TIMERG1.wdt_wprotect = 0;                    // write protect
}

int swapLED(int number) {
  // 0 +2
  // 1 nic
  // 2 -2
  switch (number % 3) {
    case 0:
      return number + 2;
      break;
    case 1:
      return number;
      break;
    case 2:
      return number - 2;
      break;
  }
}

TaskHandle_t TaskVU;
void TaskVUcode(void *pvParameters) {
  static unsigned long lastPeakTime;
  for (;;) {

    if (currCulNo) {
      //songRefresh();
      //Serial.print("currCulNo: ");
      //Serial.println(currCulNo);
      for (int i = 0; i < NUM_BANDS; i++)
        drawLineT(i, 'a', (i+3)%NUM_BANDS);
      rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      for (int k = 0; k < 5; k++) {
        for (int j = 0; j < NUM_BANDS; j++) {
          for (int i = 0; i < NUM_BANDS; i++)
            drawLineT(i, 'a', (3*i + j));
          rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }
      currCulNo = 0;
    }

    if (!VUon) {
      // Reset bandValues[]
      for (int i = 0; i < NUM_BANDS; i++) {
        bandValues[i] = 0;
      }
      //unsigned long newTime;
      // Sample the audio pin
      for (int i = 0; i < SAMPLES; i++) {
        //newTime = micros();
        // newTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        //vReal[i] = analogRead(AUDIO_IN_PIN);  // A conversion takes about 9.7uS on an ESP32
        vReal[i] = adc1_get_raw(ADC1_CHANNEL_0);
        vImag[i] = 0;
        // while ((xTaskGetTickCount() * portTICK_PERIOD_MS - newTime) < sampling_period_us) {
        //}
      }

      // Compute FFT
      FFT.DCRemoval();
      FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(FFT_FORWARD);
      FFT.ComplexToMagnitude();

      // Analyse FFT results
      for (int i = 2; i < (SAMPLES / 2); i++) {  // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency bin and its value the amplitude.
        if (vReal[i] > NOISE) {                  // Add a crude noise filter

          //10 bands 40000/512= 39
          //if (i <= 2) bandValues[0] += (int)vReal[i];               //0-0
          //if (i > 2 && i <= 4) bandValues[1] += (int)vReal[i];      //1-1
          //if (i > 4 && i <= 7) bandValues[2] += (int)vReal[i];      //2-3
          //if (i > 7 && i <= 12) bandValues[3] += (int)vReal[i];     //4-7
          //if (i > 12 && i <= 21) bandValues[4] += (int)vReal[i];    //8-15
          //if (i > 21 && i <= 38) bandValues[5] += (int)vReal[i];    //16-31
          //if (i > 38 && i <= 72) bandValues[6] += (int)vReal[i];    //32-63
          //if (i > 72 && i <= 138) bandValues[7] += (int)vReal[i];   //64-127
          //if (i > 138 && i <= 240) bandValues[8] += (int)vReal[i];  //128-255
          //if (i > 240) bandValues[9] += (int)vReal[i];              //256-512
          if (i <= 5) bandValues[0] += (int)vReal[i];
          if (i > 5 && i <= 10) bandValues[1] += (int)vReal[i];
          if (i > 10 && i <= 19) bandValues[2] += (int)vReal[i];
          if (i > 19 && i <= 35) bandValues[3] += (int)vReal[i];
          if (i > 35 && i <= 65) bandValues[4] += (int)vReal[i];
          if (i > 65 && i <= 105) bandValues[5] += (int)vReal[i];
          if (i > 105 && i <= 180) bandValues[6] += (int)vReal[i];
          if (i > 180 && i <= 240) bandValues[7] += (int)vReal[i];
          if (i > 240 && i <= 340) bandValues[8] += (int)vReal[i];
          if (i > 340) bandValues[9] += (int)vReal[i];
        }
      }

      // Process the FFT data into bar heights
      for (byte band = 0; band < NUM_BANDS; band++) {

        // Scale the bars for the display
        int barHeight = bandValues[band] / amplitude;
        if (barHeight > TOP) barHeight = TOP;

        // Small amount of averaging between frames
        barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;

        // Move peak up
        if (barHeight > peak[band]) {
          peak[band] = min(TOP, barHeight);
        }

        // Draw bars
#ifdef ADAFRUIT
        for (int i = 0; i < TOP; i += 3) {
          pixels.setPixelColor(59 - (((band * TOP) + i) / 3),
                               pixels.Color(((barHeight > i + 1 && !bar) || (peak[band] == i + 2 && !top)) ? ledBrightness : 0,
                                            ((barHeight > i && !bar) || (peak[band] == i + 1 && !top)) ? ledBrightness : 0,
                                            ((barHeight > i + 2 && !bar) || (peak[band] == i + 3 && !top)) ? ledBrightness : 0));
        }
#else
        for (int i = 0; i < TOP; i++) {
          int bit;
          int j = swapLED(179 - ((band * TOP) + i)) * 8;
          //Serial.println(j);
          if (((barHeight > i && !bar) || (peak[band] == i + 1 && !top))) {
            for (bit = 0; bit < 5; bit++) {
              led_data[j + bit].level0 = 1;
              led_data[j + bit].duration0 = 4;
              led_data[j + bit].level1 = 0;
              led_data[j + bit].duration1 = 8;
            }
            for (; bit < 8; bit++) {
              led_data[j + bit].level0 = 1;
              led_data[j + bit].duration0 = 8;
              led_data[j + bit].level1 = 0;
              led_data[j + bit].duration1 = 4;
            }
          } else {
            for (bit = 0; bit < 8; bit++) {
              led_data[j + bit].level0 = 1;
              led_data[j + bit].duration0 = 4;
              led_data[j + bit].level1 = 0;
              led_data[j + bit].duration1 = 8;
            }
          }
        }
#endif
        // Save oldBarHeights for averaging later
        oldBarHeights[band] = barHeight;
      }
#ifdef ADAFRUIT
      pixels.show();
#else
      // Send the data
      rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
#endif
      // Decay peak
      unsigned long nowMillisTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
#ifdef DEBUG
      Serial.print("nowMillisTime: ");
      Serial.print(nowMillisTime);
      Serial.print(". lastPeakTime: ");
      Serial.println(lastPeakTime);
#endif
      if (nowMillisTime - lastPeakTime > 180) {  //60
        for (byte band = 0; band < NUM_BANDS; band++)
          if (peak[band] > 0) peak[band] -= 1;
        lastPeakTime = nowMillisTime;
      }
      //vTaskDelay(10 / portTICK_PERIOD_MS);
    } else {
#ifdef ADAFRUIT
      pixels.clear();
#else
      for (int j = 0; j < 179 * 8; j++) {
        led_data[j].level0 = 1;
        led_data[j].duration0 = 4;
        led_data[j].level1 = 0;
        led_data[j].duration1 = 8;
      }
      rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
#endif
      //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    feedTheDog();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  /*ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
*/
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("WIFI Started");

  //start preferences instance
  pref.begin("radio", false);
  //set current amplitude
  if (pref.isKey("amplitude")) amplitude = pref.getUShort("amplitude");
  if (amplitude < 10) amplitude = 10;
  //set current station to saved value if available
  if (pref.isKey("station")) infolnc = pref.getUShort("station");
  if (infolnc >= STATIONS) infolnc = 0;

  /* audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // 0...21
  audio.connecttohost(stationlist[infolnc]);
*/
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));  //1.5??

  for (int i = 0; i < NUM_BUTTONS; i++)
    pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(TOP_PIN, INPUT_PULLUP);
  pinMode(BAR_PIN, INPUT_PULLUP);
  pinMode(RADIO_PIN, INPUT_PULLUP);
  pinMode(SONG_PIN, INPUT_PULLUP);
  //pinMode(STATION_PIN, INPUT_PULLUP);
  //pinMode(AUDIO_IN_PIN, INPUT);  // Signal in on this pin
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  pinMode(BAR_LED_PIN, OUTPUT);
  pinMode(TOP_LED_PIN, OUTPUT);
  pinMode(RADIO_LED_PIN, OUTPUT);
  pinMode(VU_LED_PIN, OUTPUT);

#ifdef ADAFRUIT
  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.show();
#else
  Serial.println("rmtInit");
  if ((rmt_send = rmtInit(WS_PIN, RMT_TX_MODE, RMT_MEM_64)) == NULL) {
    Serial.println("init sender failed\n");
  }
  float realTick = rmtSetTick(rmt_send, 100);
  Serial.printf("real tick set to: %fns\n", realTick);
#endif

  radioOn = -1;  // to light up LEDs
  VUon = -1;
  top = -1;
  bar = -1;
  buttonCheck();

  xTaskCreatePinnedToCore(
    TaskAudiocode,
    "TaskAudio",
    2 * 8192,
    NULL,
    8,
    &TaskAudio,
    1);

  xTaskCreatePinnedToCore(
    TaskVUcode, /* Function to implement the task */
    "TaskVU",   /* Name of the task */
    4 * 8192,   /* Stack size in words */
    NULL,       /* Task input parameter */
    8,          /* Priority of the task */
    &TaskVU,    /* Task handle. */
    0);
}


void loop() {
  //ArduinoOTA.handle();
}

int lastButtonState[NUM_BUTTONS], buttonState[NUM_BUTTONS];
unsigned long lastPlayDebounceTime[NUM_BUTTONS];
unsigned long debouncePlayDelay = 100;

void buttonCheck() {
  static unsigned long lastMillis;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis < 20)
    return;
  lastMillis = currentMillis;
  for (int i = 0; i < NUM_BUTTONS; i++) {
    int buttonRead = digitalRead(buttonPins[i]);
    if (buttonRead != lastButtonState[i]) {
      lastPlayDebounceTime[i] = currentMillis;
    }
    if ((currentMillis - lastPlayDebounceTime[i]) > debouncePlayDelay) {
      if (buttonRead != buttonState[i]) {
        buttonState[i] = buttonRead;
        if (buttonRead == LOW) {
#ifdef DEBUG
          Serial.print("Pin: ");
          Serial.println(i);
#endif
          switch (i) {
            case 0:  //ST_UP_PIN
              stationUpDown(1);
              break;
            case 1:  //ST_DOWN_PIN
              stationUpDown(-1);
              break;
              /* case 2:  //LIGHT_UP_PIN
              ledLightSetting(LIGHT_UP_PIN);
              break;
            case 3:  //LIGHT_DOWN_PIN
              ledLightSetting(LIGHT_DOWN_PIN);
              break;*/
            case 2:  //LVL_UP_PIN
              VUsetting(LVL_UP_PIN);
              break;
            case 3:
              VUsetting(LVL_DOWN_PIN);
              break;
          }
        } else {
          if ((currentMillis - lastPlayDebounceTime[i]) > 500) {
#ifdef DEBUG
            Serial.println("Long press");
#endif
          }
        }
      }
    }
    lastButtonState[i] = buttonRead;
  }

  if (radioOn != digitalRead(RADIO_PIN)) {
    if (radioOn != -1)
      radioOn = !radioOn;
    if (!radioOn) {
      digitalWrite(RADIO_LED_PIN, HIGH);
      audio.connecttohost(stationlist[infolnc]);
    } else {
      digitalWrite(RADIO_LED_PIN, LOW);
      audio.stopSong();
    }
#ifdef DEBUG
    Serial.print("radioOn de-activated: ");
    Serial.println(radioOn);
#endif
  }

  songName = digitalRead(SONG_PIN);

  if (VUon != digitalRead(VU_PIN)) {
    if (VUon != -1)
      VUon = !VUon;
    if (VUon) {
      digitalWrite(VU_LED_PIN, LOW);
      digitalWrite(BAR_LED_PIN, LOW);
      digitalWrite(TOP_LED_PIN, LOW);
    } else {
      digitalWrite(VU_LED_PIN, HIGH);
      top = 1;
      bar = 1;
    }
#ifdef DEBUG
    Serial.print("VUon de-activated: ");
    Serial.println(VUon);
#endif
  }
  if (top != digitalRead(TOP_PIN)) {
    if (top != -1)
      top = !top;
    if (top)
      digitalWrite(TOP_LED_PIN, LOW);
    else
      digitalWrite(TOP_LED_PIN, HIGH);
#ifdef DEBUG
    Serial.print("Top de-activated: ");
    Serial.println(top);
#endif
  }

  if (bar != digitalRead(BAR_PIN)) {
    if (bar != -1)
      bar = !bar;
    if (bar)
      digitalWrite(BAR_LED_PIN, LOW);
    else
      digitalWrite(BAR_LED_PIN, HIGH);
#ifdef DEBUG
    Serial.print("Bar de-activated: ");
    Serial.println(bar);
#endif
  }
}

void VUsetting(int pinNumber) {
  switch (pinNumber) {
    case LVL_UP_PIN:
      amplitude *= 1.3;
      break;
    case LVL_DOWN_PIN:
      amplitude /= 1.3;
      if (amplitude < 10)
        amplitude = 10;
      break;
  }
  pref.putUShort("amplitude", amplitude);
#ifdef DEBUG
  Serial.print("Pin activated: ");
  Serial.println(pinNumber);
#endif
}
void stationUpDown(int upDown) {
  audio.stopSong();
  infolnc += upDown;
  if (infolnc >= STATIONS)
    infolnc = 0;
  else if (infolnc < 0)
    infolnc = STATIONS - 1;
  pref.putUShort("station", infolnc);
  audio.connecttohost(stationlist[infolnc]);
#ifdef DEBUG
  Serial.println(infolnc);
#endif
}

//FONT DEFENITION
extern byte alphabets[][18] = { { B00000000, //space -everything outside of A--Z, a--z
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,  //8
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000,
                                  B00000000 },
                                { B00000000, //A
                                  B00011000,
                                  B00011000,
                                  B00011000,
                                  B00111100,
                                  B00111100,
                                  B00111100,
                                  B00100100,
                                  B00100100,  //8
                                  B01100110,
                                  B01111110,
                                  B01111110,
                                  B01100110,
                                  B01100110,
                                  B01000010,
                                  B11000011,
                                  B11000011,
                                  B00000000 }                                
                                { B00000000, //B
                                  B11111100,
                                  B11111110,
                                  B11000111,
                                  B11000011,
                                  B11000011,
                                  B11000011,
                                  B11000111,
                                  B11111100,  //8
                                  B11111100,
                                  B11000111,
                                  B11000011,
                                  B11000011,
                                  B11000011,
                                  B11000111,
                                  B11111110,
                                  B11111100,
                                  B00000000 } };


void drawLineT(int bar, char pismeno, int sloupec) {
  int alphabetIndex = toupper(pismeno) - '@';
  if (alphabetIndex < 0 || alphabetIndex > 26) alphabetIndex = 0; //space if outside of A [1] - Z [26]
  Serial.print("pismeno: ");
  Serial.print(alphabetIndex);
  Serial.print(" ");
  Serial.print((int)'@');
  Serial.print(" ");
  Serial.print((int)toupper(pismeno));
  for (int i = 0; i < TOP; i++) {
    int bit;
    int j = swapLED(179 - ((bar * TOP) + i)) * 8;
    Serial.print(" b: ");
    Serial.print(((alphabets[alphabetIndex][TOP - i - 1] >> (7 - sloupec)) & 1));
    if (sloupec < 8 && ((alphabets[alphabetIndex][TOP - i - 1] >> (7 - sloupec)) & 1)) {
      for (bit = 0; bit < 5; bit++) {
        led_data[j + bit].level0 = 1;
        led_data[j + bit].duration0 = 4;
        led_data[j + bit].level1 = 0;
        led_data[j + bit].duration1 = 8;
      }
      for (; bit < 8; bit++) {
        led_data[j + bit].level0 = 1;
        led_data[j + bit].duration0 = 8;
        led_data[j + bit].level1 = 0;
        led_data[j + bit].duration1 = 4;
      }
    } else {
      for (bit = 0; bit < 8; bit++) {
        led_data[j + bit].level0 = 1;
        led_data[j + bit].duration0 = 4;
        led_data[j + bit].level1 = 0;
        led_data[j + bit].duration1 = 8;
      }
    }
  }
  Serial.println("");
}

void songRefresh(void) {
  static int currCulNoAct = 0;
  static unsigned long songTime = millis();
  if (currCulNo) {
    if (millis() - songTime > 50) {
      // pixels.clear();
      if (currCulNo > currCulNoAct) {
        for (int i = 0; i < NUM_BANDS; i++) {
          drawLineT(i, msg[(currCulNoAct - NUM_BANDS + 1 + i) / 10], (currCulNoAct - NUM_BANDS + 1 + i) % 10);
        }
      } else {
        currCulNo = 0;
        //test, zda oz odrolovano
        //pokud ne, odroluj
      }
      //pixels.show();
      currCulNoAct++;
      songTime = millis();
    }
  }
}

// optional
/*void audio_info(const char *info) {
  Serial.print("info        ");
  Serial.println(info);
}*/
void audio_id3data(const char *info) {  //id3 metadata
  Serial.print("id3data     ");
  Serial.println(info);
}
void audio_eof_mp3(const char *info) {  //end of file
  Serial.print("eof_mp3     ");
  Serial.println(info);
}
void audio_showstation(const char *info) {
  Serial.print("station     ");
  Serial.println(info);
}
void audio_showstreaminfo(const char *info) {
  Serial.print("streaminfo  ");
  Serial.println(info);
}
void audio_showstreamtitle(const char *info) {
  Serial.print("streamtitle ");
  Serial.println(info);
  if (songName) {
    int msgLength = max(strlen(info), sizeof(msg) - 1);
    strncpy(msg, info, msgLength);
    msg[msgLength] = '\0';
    currCulNo = msgLength * 10;
    Serial.print("msg ");
    Serial.println(msg);
  }
}
void audio_bitrate(const char *info) {
  Serial.print("bitrate     ");
  Serial.println(info);
}
void audio_commercial(const char *info) {  //duration in sec
  Serial.print("commercial  ");
  Serial.println(info);
}
void audio_icyurl(const char *info) {  //homepage
  Serial.print("icyurl      ");
  Serial.println(info);
}
void audio_lasthost(const char *info) {  //stream URL played
  Serial.print("lasthost    ");
  Serial.println(info);
}
void audio_eof_speech(const char *info) {
  Serial.print("eof_speech  ");
  Serial.println(info);
}