#if !defined(nullptr)
#define nullptr NULL
#endif
#include "Arduino.h"
#include <WiFi.h>
#include "Audio.h"
#include <Preferences.h>
#include <arduinoFFT.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "Pass.h"

//#define DEBUG
//#define SONG_NAME
//#define ADAFRUIT
#define DYNAMIC_AMP

#ifdef ADAFRUIT
#include <Adafruit_NeoPixel.h>
#else  //ADAFRUIT-RMT
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp32-hal.h"
#endif  //RMT

//#define AUDIO_IN_PIN 36  // Signal in on this pin - ADC1_CHANNEL_0
#define WS_PIN 13

#define ST_UP_PIN 32     // In.  OK_OK_OK_OK
#define ST_DOWN_PIN 33   // In  OK_OK_OK_OK
#define LVL_UP_PIN 5     //
#define LVL_DOWN_PIN 21  //
int buttonPins[] = { ST_UP_PIN, ST_DOWN_PIN, LVL_UP_PIN, LVL_DOWN_PIN };
#define NUM_BUTTONS 4

#define VU_PIN 14      // In -  OK_OK_OK_OK
#define VU_LED_PIN 12  // Out -  OK_OK_OK_OK

#define TOP_PIN 18      // In -  OK_OK_OK_OK
#define TOP_LED_PIN 19  // Out -  OK_OK_OK_OK

#define BAR_PIN 22      // In  -  OK_OK_OK_OK
#define BAR_LED_PIN 23  // Out -  OK_OK_OK_OK

#define RADIO_PIN 16      // In
#define RADIO_LED_PIN 17  // Out

#define I2S_DOUT 26  // DIN connection
#define I2S_BCLK 27  // Bit clock
#define I2S_LRC 25   // Left Right Clock

#define SAMPLES 1024         // Must be a power of 2
#define SAMPLING_FREQ 40000  // Hz, must be 40000 or less due to ADC conversion time
int amplitude = 1000;        // Depending on your audio source level

Audio audio;

Preferences pref;
int infolnc = 0;  //stores channel number for tuner

bool bar = 1, top = 1, radioOn, VUon;

#ifdef SONG_NAME
#define SONG_PIN 15
bool songName;
int currCulNo = 0;
char msg[50];
#endif  //SONG_NAME

#define NUM_BANDS 10
#define TOP 18
#define NUM_LEDS (NUM_BANDS * TOP)  // Total number of LEDs

#ifdef ADAFRUIT
int ledBrightness = 32;  //store led Brightness, maybe different for different colours?
Adafruit_NeoPixel pixels(NUM_LEDS / 3, WS_PIN, NEO_GRB + NEO_KHZ800);
#else  //ADAFRUIT-RMT
#define NR_OF_ALL_BITS 24 * (NUM_LEDS / 3)
rmt_data_t led_data[NR_OF_ALL_BITS];
rmt_obj_t *rmt_send = NULL;
#endif  //RMT

// Sampling and FFT stuff
unsigned int sampling_period_us;
byte peak[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // The length of these arrays must be >= NUM_BANDS
int oldBarHeights[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandMin[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandMax[] = { 25000, 25000, 25000, 50000, 75000, 75000, 75000, 75000, 75000, 75000, 75000 };
int bandMinSample[] = { __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__, __INT_MAX__ };
int bandMaxSample[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//int totalMin = __INT_MAX__;
//int totalMax = 0;
double vReal[SAMPLES];
double vImag[SAMPLES];
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

#define STATIONS 15
char *stationlist[STATIONS] = {
  "http://streaming.live365.com/b92108_128mp3",
  "https://panel.retrolandigital.com/listen/oldies_internet_radio/listen",
  "http://vis.media-ice.musicradio.com/CapitalMP3",
  "https://radiorock.stream.laut.fm/radiorock",
  "https://media-ssl.musicradio.com/SmoothLondonMP3",
  "https://online.radioroks.ua/RadioROKS",
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

TaskHandle_t TaskAudio;
void TaskAudiocode(void *pvParameters) {
#ifdef DEBUG
  Serial.print("TaskAudio running on core ");
  Serial.println(xPortGetCoreID());
#endif  //DEBUG
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // 0...21
#ifdef DEBUG
  Serial.print("Audio task started: ");
  Serial.print(radioOn);
  Serial.print(" on station: ");
  Serial.println(infolnc);
#endif  //DEBUG
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

void setLedOn(int number) {
  int bit;
  for (bit = 0; bit < 5; bit++) {
    led_data[number + bit].level0 = 1;
    led_data[number + bit].duration0 = 4;
    led_data[number + bit].level1 = 0;
    led_data[number + bit].duration1 = 8;
  }
  for (; bit < 8; bit++) {
    led_data[number + bit].level0 = 1;
    led_data[number + bit].duration0 = 8;
    led_data[number + bit].level1 = 0;
    led_data[number + bit].duration1 = 4;
  }
}

void setLedOff(int number) {
  for (int bit = 0; bit < 8; bit++) {
    led_data[number + bit].level0 = 1;
    led_data[number + bit].duration0 = 4;
    led_data[number + bit].level1 = 0;
    led_data[number + bit].duration1 = 8;
  }
}

TaskHandle_t TaskVU;
void TaskVUcode(void *pvParameters) {
  static unsigned long lastTime;
  static int sampleNo;
  for (;;) {
#ifdef SONG_NAME
    if (currCulNo) {
      //for (int i = 0; i < NUM_BANDS; i++)
      // drawLineT(i, 'a', (i + 3) % NUM_BANDS);
      //rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
      //vTaskDelay(10000 / portTICK_PERIOD_MS);
      //strcpy(msg, "  abab  ");
      //currCulNo = 30;
      //for (int k = 0; k < 5; k++) {
      //for (int j = 0; j < currCulNo; j++) {

      unsigned long nowMillisTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
      if (nowMillisTime - lastTime > 150) {
        lastTime = nowMillisTime;
        for (int i = 0; i < NUM_BANDS; i++)
          //  drawLineT(i, 'b', j == i ? 0 : 9);
          //  drawLineT(i, msg[(j + i*3 + 1) / 10], (j + i*3 + 1) % 10);
          drawLineT(i, msg[(j + i + 1) / 5], ((j + i + 1) % 5));
        rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
        //vTaskDelay(150 / portTICK_PERIOD_MS);
        //drawingCharBar++;
        // }
        //}
        // drawLineT(i, msg[(10 + drawingCharBar - i) / 10], (10 + drawingCharBar - i) % 10);
        // drawLineT(i, msg[(10 + drawingCharBar - (i*3)) / 10], (10 + drawingCharBar - (i*3)) % 10);
        j++;
        if (j >= currCulNo) {
          j = 0;
          k++;
          if (k >= 2) {
            k = 0;
            currCulNo = 0;
          }
        }
      }
    } else
#endif  //SONG_NAME
      if (!VUon) {
        // Reset bandValues[]
        for (int i = 0; i < NUM_BANDS; i++) {
          bandValues[i] = 0;
        }
        //unsigned long newTime;
        // Sample the audio pin
        for (int i = 0; i < SAMPLES; i++) {
          //newTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
          vReal[i] = adc1_get_raw(ADC1_CHANNEL_0);
          vImag[i] = 0;
          // while ((xTaskGetTickCount() * portTICK_PERIOD_MS - newTime) < sampling_period_us) {
          //     vTaskDelay(1);
          //}
        }

        // Compute FFT
        FFT.DCRemoval();
        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(FFT_FORWARD);
        FFT.ComplexToMagnitude();

        // Analyse FFT results
        for (int i = 2; i < (SAMPLES / 2); i++) {  // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency bin and its value the amplitude.
#ifdef DYNAMIC_AMP
          if (i <= 4) bandValues[0] += (int)vReal[i];               //0-0
          if (i > 4 && i <= 6) bandValues[1] += (int)vReal[i];      //1-1
          if (i > 6 && i <= 10) bandValues[2] += (int)vReal[i];     //2-3
          if (i > 10 && i <= 21) bandValues[3] += (int)vReal[i];    //4-7
          if (i > 21 && i <= 42) bandValues[4] += (int)vReal[i];    //8-15
          if (i > 42 && i <= 100) bandValues[5] += (int)vReal[i];   //16-31
          if (i > 100 && i <= 170) bandValues[6] += (int)vReal[i];  //32-63
          if (i > 170 && i <= 280) bandValues[7] += (int)vReal[i];  //64-127
          if (i > 280 && i <= 390) bandValues[8] += (int)vReal[i];  //128-255
          if (i > 390) bandValues[9] += (int)vReal[i];              //256-512
#else                                                               //DYNAMIC_AMP
        if (vReal[i] > amplitude / 5) {  // Cut of noise
          vReal[i] = (vReal[i] - amplitude / 5);
          //10 bands 40000/512= 39
          if (i <= 2) bandValues[0] += (int)vReal[i];               //0-0
          if (i > 2 && i <= 4) bandValues[1] += (int)vReal[i];      //1-1
          if (i > 4 && i <= 7) bandValues[2] += (int)vReal[i];      //2-3
          if (i > 7 && i <= 12) bandValues[3] += (int)vReal[i];     //4-7
          if (i > 12 && i <= 21) bandValues[4] += (int)vReal[i];    //8-15
          if (i > 21 && i <= 38) bandValues[5] += (int)vReal[i];    //16-31
          if (i > 38 && i <= 72) bandValues[6] += (int)vReal[i];    //32-63
          if (i > 72 && i <= 138) bandValues[7] += (int)vReal[i];   //64-127
          if (i > 138 && i <= 240) bandValues[8] += (int)vReal[i];  //128-255
          if (i > 240) bandValues[9] += (int)vReal[i];              //256-512
          /*if (i <= 4) bandValues[0] += (int)vReal[i];
          if (i > 4 && i <= 8) bandValues[1] += (int)vReal[i];
          if (i > 8 && i <= 18) bandValues[2] += (int)vReal[i];
          if (i > 18 && i <= 37) bandValues[3] += (int)vReal[i];
          if (i > 37 && i <= 70) bandValues[4] += (int)vReal[i];
          if (i > 70 && i <= 115) bandValues[5] += (int)vReal[i];
          if (i > 115 && i <= 200) bandValues[6] += (int)vReal[i];
          if (i > 200 && i <= 240) bandValues[7] += (int)vReal[i];
          if (i > 280 && i <= 400) bandValues[8] += (int)vReal[i];
          if (i > 400) bandValues[9] += (int)vReal[i];*/
        }
#endif                                                              //DYNAMIC_AMP
        }
        sampleNo++;
        if (sampleNo == 40) sampleNo = 0;
        // Process the FFT data into bar heights
        for (byte band = 0; band < NUM_BANDS; band++) {
#ifdef DYNAMIC_AMP
          if (sampleNo) {
            /*              Serial.print(band);
            Serial.print(" ");
                        Serial.print(bandValues[band]);
            Serial.println("");*/
            if (bandMinSample[band] > bandValues[band]) {  //select max/min every second
              bandMinSample[band] = bandValues[band];
            }
            if (bandMaxSample[band] < bandValues[band]) {
              bandMaxSample[band] = bandValues[band];
            }

          } else {
            bandMin[band] = ((bandMin[band] * 9 + bandMinSample[band]) / 10);  //calculate average min/max
            bandMax[band] = ((bandMax[band] * 9 + bandMaxSample[band]) / 10);
            // totalMin = (totalMin * 9 + bandMin[band]) / 10;  //total average min/max
            // totalMax = (totalMax * 9 + bandMax[band]) / 10;

#ifdef DEBUG
            Serial.print("Band:");
            Serial.print(band);
            Serial.print("\t");
            Serial.print(bandMinSample[band]);
            Serial.print("\t");
            Serial.print(bandMaxSample[band]);
            Serial.print("\t");
            Serial.print(bandMin[band]);
            Serial.print("\t");
            Serial.print(bandMax[band]);
            //Serial.print("\t");
            //Serial.print(totalMin);
            //Serial.print("\t");
            //Serial.println(totalMax);
            Serial.println("");
#endif  //DEBUG
            bandMinSample[band] = __INT_MAX__;
            bandMaxSample[band] = 0;
          }
          // int minV = (bandMin[band] > (totalMin * 2)) ? bandMin[band] * 2 : bandMin[band];
          // int maxV = (bandMax[band] > (totalMax * 2)) ? bandMax[band] * 2 : bandMax[band];
          // if (minV < (totalMin / 2)) minV = totalMin / 2;
          // if (maxV < (totalMax / 2)) maxV = totalMax / 2;
          //int barHeight = ((TOP + 1) * (bandValues[band] - minV)) / (maxV - minV);
          int barHeight = ((TOP + 1) * (bandValues[band] - bandMin[band])) / (bandMax[band] - bandMin[band]);
          if (barHeight < 0) barHeight = 0;
#else   //DYNAMIC_AMP \
        // Scale the bars for the display
        int barHeight = bandValues[band] / amplitude;
#endif  //DYNAMIC_AMP
          if (barHeight > TOP) barHeight = TOP;

          // Small amount of averaging between frames
          barHeight = ((oldBarHeights[band] * 1) + barHeight * 2) / 3;

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
#else   //ADAFRUIT-RMT
        for (int i = 0; i < TOP; i++) {
          //int bit;
          int j = swapLED(179 - ((band * TOP) + i)) * 8;
          //Serial.println(j);
          if (((barHeight > i && !bar) || (peak[band] == i + 1 && !top))) {
            setLedOn(j);
          } else {
            setLedOff(j);
          }
        }
#endif  //RMT \
  // Save oldBarHeights for averaging later
          oldBarHeights[band] = barHeight;
        }
#ifdef ADAFRUIT
        pixels.show();
#else   //ADAFRUIT-RMT \
        // Send the data
      rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
#endif  //RMT \
  // Decay peak
        unsigned long nowMillisTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (nowMillisTime - lastTime > 180) {  //60
          for (byte band = 0; band < NUM_BANDS; band++)
            if (peak[band] > 0) peak[band] -= 1;
          lastTime = nowMillisTime;
        }
      } else {
#ifdef ADAFRUIT
        pixels.clear();
#else   //ADAFRUIT-RMT
      for (int j = 0; j < 179 * 8; j += 8)
        setLedOff(j);
      rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
#endif  //RMT
      }
    feedTheDog();
  }
}

void strtLedAnimation(void) {
  for (int k = 0; k < 9 + TOP * 2; k++) {
    for (int band = 0; band < NUM_BANDS; band++) {
      for (int i = 0; i < TOP; i++) {
        int j = swapLED(179 - ((band * TOP) + i)) * 8;
        //       if ((i <= k && k < TOP) || (k >= TOP && (TOP - i) > (2 * TOP) - k)) {
        if ((i <= k - band) && (i <= 8 + 2 * TOP - k - (NUM_BANDS - band))) {  //18, k 0-35, band 0-9
          setLedOn(j);
        } else {
          setLedOff(j);
        }
      }
    }
    rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
    vTaskDelay(30 / portTICK_PERIOD_MS);
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
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println(" - WIFI Started");

  //start preferences instance
  pref.begin("radio", false);
  //set current amplitude
  if (pref.isKey("amplitude")) amplitude = pref.getUShort("amplitude");
  if (amplitude < 10) amplitude = 10;
  //set station
  if (pref.isKey("station")) infolnc = pref.getUShort("station");
  if (infolnc >= STATIONS) infolnc = 0;

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  for (int i = 0; i < NUM_BUTTONS; i++)
    pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(TOP_PIN, INPUT_PULLUP);
  pinMode(BAR_PIN, INPUT_PULLUP);
  pinMode(RADIO_PIN, INPUT_PULLUP);
#ifdef SONG_NAME
  pinMode(SONG_PIN, INPUT_PULLUP);
#endif  //SONG_NAME
  //pinMode(AUDIO_IN_PIN, INPUT);  // Signal in on this pin
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_2_5);
  //adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  pinMode(BAR_LED_PIN, OUTPUT);
  pinMode(TOP_LED_PIN, OUTPUT);
  pinMode(RADIO_LED_PIN, OUTPUT);
  pinMode(VU_LED_PIN, OUTPUT);

#ifdef ADAFRUIT
  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.show();
#else   //ADAFRUIT-RMT
  Serial.println("rmtInit");
  if ((rmt_send = rmtInit(WS_PIN, RMT_TX_MODE, RMT_MEM_64)) == NULL) {
    Serial.println("init sender failed\n");
  }
  float realTick = rmtSetTick(rmt_send, 100);
  Serial.printf("real tick set to: %fns\n", realTick);
  strtLedAnimation();
#endif  //RMT
  Serial.println("Leds startup ends");
  radioOn = -1;  // to light up LEDs on buttons
  VUon = -1;
  top = -1;
  bar = -1;
  buttonCheck();
  Serial.println("Tasks starting\n");
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
  Serial.println("Tasks started\n");
}

void loop() {}

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
#endif  //DEBUG
          switch (i) {
            case 0:  //ST_UP_PIN
              stationUpDown(1);
              break;
            case 1:  //ST_DOWN_PIN
              stationUpDown(-1);
              break;
            case 2:  //LVL_UP_PIN
              VUsetting(LVL_UP_PIN);
              break;
            case 3:
              VUsetting(LVL_DOWN_PIN);
              break;
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
#endif  //DEBUG
  }

#ifdef SONG_NAME
  songName = digitalRead(SONG_PIN);
#endif  //SONG_NAME

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
#endif  //DEBUG
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
#endif  //DEBUG
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
#endif  //DEBUG
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
#endif  //DEBUG
}

void stationUpDown(int upDown) {
  if (!radioOn) {
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
#endif  //DEBUG
  }
}

#ifdef SONG_NAME
//FONT DEFENITION
extern byte alphabets[][18] = {

  { B00000000,  //space -everything outside of A--Z, a--z
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
  { B00000000,  //A
    B01000000,
    B01000000,
    B01000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,  //8
    B10100000,
    B10100000,
    B10100000,
    B11100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //B
    B11111000,
    B10000100,
    B10000010,
    B10000001,
    B10000001,
    B10000010,
    B10000100,
    B11111000,  //8
    B10000100,
    B10000010,
    B10000001,
    B10000001,
    B10000001,
    B10000010,
    B10000100,
    B11111000,
    B00000000 },
  { B00000000,  //C
    B01000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,  //8
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //D
    B11000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,
    B11000000,
    B00000000 },
  { B00000000,  //E
    B11100000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B11100000,  //8
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B11100000,
    B00000000 },
  { B00000000,  //F
    B11100000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B11100000,  //8
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B00000000 },
  { B00000000,  //G
    B01000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,  //8
    B10000000,
    B10000000,
    B11100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //H
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B11100000,  //8
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //I
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,  //8
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B00000000 },
  { B00000000,  //J
    B00100000,
    B00100000,
    B00100000,
    B00100000,
    B00100000,
    B00100000,
    B00100000,
    B00100000,  //8
    B00100000,
    B00100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //K
    B10100000,
    B10100000,
    B10000000,
    B10000000,
    B11000000,
    B11000000,
    B10000000,
    B10000000,  //8
    B10000000,
    B10000000,
    B11000000,
    B11000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //L
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,  //8
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B11100000,
    B00000000 },
  { B00000000,  //M
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B10100000,
    B01000000,
    B01000000,
    B01000000,
    B00000000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //N
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,
    B10000000,
    B01000000,  //8
    B01000000,
    B01000000,
    B01000000,
    B00100000,
    B00100000,
    B00100000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //O
    B01000000,
    B00000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //P
    B11000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,
    B11000000,  //8
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B00000000 },
  { B00000000,  //Q
    B01000000,
    B00000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B01100000,
    B00000000 },
  { B00000000,  //R
    B11000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B10000000,
    B10000000,
    B11000000,  //8
    B10000000,
    B10000000,
    B11000000,
    B11000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //S
    B01000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B00100000,
    B00100000,
    B01000000,  //8
    B01000000,
    B10000000,
    B10000000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //T
    B11100000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,  //8
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B00000000 },
  { B00000000,  //U
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B00000000 },
  { B00000000,  //V
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B00000000 },
  { B00000000,  //W
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,  //8
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B00000000 },
  { B00000000,  //X
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B01000000,  //8
    B01000000,
    B00000000,
    B00000000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000 },
  { B00000000,  //Y
    B10100000,
    B10100000,
    B10100000,
    B10100000,
    B00000000,
    B00000000,
    B01000000,
    B01000000,  //8
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B01000000,
    B00000000 },
  { B00000000,  //Z
    B11100000,
    B00100000,
    B00100000,
    B00100000,
    B00000000,
    B00000000,
    B01000000,
    B01000000,  //8
    B01000000,
    B00000000,
    B00000000,
    B10000000,
    B10000000,
    B10000000,
    B10000000,
    B11100000,
    B00000000 }
};


void drawLineT(int bar, char pismeno, int sloupec) {
  int alphabetIndex = toupper(pismeno) - '@';
  if (alphabetIndex < 0 || alphabetIndex > 26) alphabetIndex = 0;  //space if outside of A [1] - Z [26]
  for (int i = 0; i < TOP; i++) {
    //int bit;
    int j = swapLED(179 - ((bar * TOP) + i)) * 8;
    //Serial.print(" b: ");
    //Serial.print(((alphabets[alphabetIndex][TOP - i - 1] >> (7 - sloupec)) & 1));
    if ((sloupec < 8 && ((alphabets[alphabetIndex][TOP - i - 1] >> (7 - sloupec)) & 1))) {
      // (sloupec > 0 && sloupec < 9 && ((alphabets[alphabetIndex][TOP - i - 1] >> (8 - sloupec)) & 1))) {
      setLedOn(j);
    } else {
      setLedOff(j);
    }
    // Serial.println("");
  }
}
#endif  //SONG_NAME

// optional
void audio_info(const char *info) {
  Serial.print("info        ");
  Serial.println(info);
}
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
#ifdef SONG_NAME
  if (songName) {
    int msgLength = max(strlen(info), sizeof(msg) - 5);
    strncpy(msg, "  ", 2);


    strncpy(&msg[2], info, msgLength);
    //msgLength = 10;
    //strncpy(&msg[2], "ahoj blani", msgLength);


    msgLength += 2;
    strncpy(&msg[msgLength], "  ", 2);
    currCulNo = msgLength * 5;  //here because the last two spaces are for spare space after the text
    msgLength += 2;
    msg[msgLength] = '\0';
    Serial.print("msg ");
    Serial.println(msg);
  }
#endif  //SONG_NAME
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