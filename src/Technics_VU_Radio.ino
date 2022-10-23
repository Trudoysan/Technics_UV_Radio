#if !defined(nullptr)
#define nullptr NULL
#endif
#include "Arduino.h"
#include <WiFi.h>
#include "Audio.h"
//#include "FS.h"
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
#include <arduinoFFT.h>
//#include <SPI.h>


//#define DEBUG

//BLE - ON/OFF (BLUE)
//VU -  bar level  up, down, on/off by switch, Song name ON/OFF, Station Name ON/OFF
//radio - ON/OFF(YELLOW/other), BAR(GREEN), TOP (RED), station up, down

//#define AUDIO_IN_PIN 36  // Signal in on this pin - ADC1_CHANNEL_0
#define WS_PIN 13

#define ST_UP_PIN 32     // In
#define ST_DOWN_PIN 33   // In
#define LVL_UP_PIN 21    // In xx
#define LVL_DOWN_PIN 39  // In xx 4 nebo 5
int buttonPins[] = { ST_UP_PIN, ST_DOWN_PIN, LVL_UP_PIN, LVL_DOWN_PIN };
#define NUM_BUTTONS 4

#define SONG_PIN 15  // In
//define STATION_PIN 39  // In

#define VU_PIN 17      // In -
#define VU_LED_PIN 16  // Out -

#define TOP_PIN 18      // In -
#define TOP_LED_PIN 19  // Out -

#define BAR_PIN 22      // In  -
#define BAR_LED_PIN 23  // Out -

#define RADIO_PIN 14      // In
#define RADIO_PIN_LED 12  // Out

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

Adafruit_NeoPixel pixels(NUM_LEDS / 3, WS_PIN, NEO_GRB + NEO_KHZ800);

// Sampling and FFT stuff
unsigned int sampling_period_us;
byte peak[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // The length of these arrays must be >= NUM_BANDS
int oldBarHeights[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int bandValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime, lastPeakTime = 0L;
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);


#define SSID "Hlava"
#define PSK "hlavouni"

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
  audio.connecttohost(stationlist[infolnc]);
  for (;;) {
    audio.loop();
  }
}

TaskHandle_t TaskVU;
void TaskVUcode(void *pvParameters) {
  for (;;) {
    //Serial.println("Jedu");
    // if (VUon && !currCulNo)
    if (VUon) {
      /*  static unsigned long cas2 = millis();
      static int led = 0;
      if (millis() - cas2 > 100) {
        if (!(led % 18))
          pixels.clear();
        int ledMod3 = led % 3;
        pixels.setPixelColor(59 - (led / 3), pixels.Color((ledMod3 >= 1) ? ledBrightness : 0, ledBrightness, (ledMod3 == 2) ? ledBrightness : 0));
        //pixels.setPixelColor(led/3, pixels.Color(ledBrightness,ledBrightness,ledBrightness));

        pixels.show();
        led++;
        if (led > NUM_LEDS) led = 0;
        cas2 = millis();
        //Serial.println(59 - (led / 3));
      }*/

      // Reset bandValues[]
      for (int i = 0; i < NUM_BANDS; i++) {
        bandValues[i] = 0;
      }

      // Sample the audio pin
      for (int i = 0; i < SAMPLES; i++) {
        //newTime = micros();
        //vReal[i] = analogRead(AUDIO_IN_PIN);  // A conversion takes about 9.7uS on an ESP32
        vReal[i] = adc1_get_raw(ADC1_CHANNEL_0);
        vImag[i] = 0;
        // while ((micros() - newTime) < sampling_period_us) {
        // }
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
        for (int i = 0; i < TOP; i += 3) {
          pixels.setPixelColor(59 - (((band * TOP) + i) / 3),
                               pixels.Color(((barHeight > i + 1 && bar) || (peak[band] == i + 2 && top)) ? ledBrightness : 0,
                                            ((barHeight > i && bar) || (peak[band] == i + 1 && top)) ? ledBrightness : 0,
                                            ((barHeight > i + 2 && bar) || (peak[band] == i + 3 && top)) ? ledBrightness : 0));
          /*    Serial.print(59 - (((band * TOP) + i) / 3));
          Serial.print(" Bar height:");
          Serial.print(barHeight);
          Serial.print(" bar:");
          Serial.print(bar);
          Serial.print(" Brightness:");
          Serial.print(ledBrightness);
          Serial.print(" R:");*/
        }
        // Save oldBarHeights for averaging later
        oldBarHeights[band] = barHeight;
      }

      pixels.show();
      // Decay peak
      unsigned long nowMillisTime = millis();
      if (nowMillisTime - lastPeakTime > 120) {  //60
        for (byte band = 0; band < NUM_BANDS; band++)
          if (peak[band] > 0) peak[band] -= 1;
      }
      lastPeakTime = nowMillisTime;
    }
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
  /*  if (pref.isKey("amplitude")) amplitude = pref.getUShort("amplitude");
  if (amplitude < 10) amplitude = 10;
  //set current ledBrightness
  if (pref.isKey("ledBrightness")) ledBrightness = pref.getUShort("ledBrightness");
  if (ledBrightness > 255) ledBrightness = 255;
  if (ledBrightness < 10) ledBrightness = 10;
  //set current station to saved value if available
  if (pref.isKey("station")) infolnc = pref.getUShort("station");
  if (infolnc >= STATIONS) infolnc = 0;

*/

  xTaskCreatePinnedToCore(
    TaskAudiocode, /* Function to implement the task */
    "TaskAudio",   /* Name of the task */
    48000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    9,             /* Priority of the task */
    &TaskAudio,    /* Task handle. */
    0);            /* Core where the task should run */

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
  pinMode(RADIO_PIN_LED, OUTPUT);

  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();

  xTaskCreatePinnedToCore(
    TaskVUcode, /* Function to implement the task */
    "TaskVU",   /* Name of the task */
    48000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    15,         /* Priority of the task */
    &TaskVU,    /* Task handle. */
    1);

  //for testing
  radioOn = 0;
  VUon = 1;
  top = 0;
}


void loop() {
  //ArduinoOTA.handle();

  //  audio.loop();

  //songRefresh();
  //buttonCheck();

  /* static unsigned long cas3 = millis();
  if (millis() - cas3 > 30000) {
    stationUpDown(1);
    cas3 = millis();
    //  pixels.setPixelColor(infolnc, pixels.Color(0, 127, 255));
    //  pixels.show();
  }
  */
}

int lastButtonState[NUM_BUTTONS], buttonState[NUM_BUTTONS];
unsigned long lastPlayDebounceTime[NUM_BUTTONS];
unsigned long debouncePlayDelay = 100;

void buttonCheck() {

  unsigned long currentMillis = millis();

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
  /*
  if (radioOn != digitalRead(RADIO_PIN)) {
    radioOn = !radioOn;
    if (radioOn)
      audio.connecttohost(stationlist[infolnc]);
    else
      audio.stopSong();
  }
   if (songName != digitalRead(SONG_PIN)) {
    songName = !songName;
    if (songName)
      ;
    else
      ;
  } 
  } 

  
  if (top != digitalRead(TOP_PIN)) {
    top = !top;
    if (top)
      digitalWrite(TOP_LED_PIN, HIGH);
    else
      digitalWrite(TOP_LED_PIN, LOW);
  }
*/
  if (bar != digitalRead(BAR_PIN)) {
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
  /*
  if (VUon != digitalRead(VU_PIN)) {
    VUon = !VUon;
    if (!VUon) pixels.clear();
  }*/
}
/*
void ledLightSetting(int pinNumber) {
  switch (pinNumber) {
    case LIGHT_UP_PIN:
      ledBrightness *= 1.3;
      if (ledBrightness > 255) ledBrightness = 255;
      break;
    case LIGHT_DOWN_PIN:
      ledBrightness /= 1.3;
      if (ledBrightness < 10) ledBrightness = 10;
      break;
  }
  pref.putUShort("ledBrightness", ledBrightness);
#ifdef DEBUG
  Serial.print("Pin activated: ");
  Serial.println(pinNumber);
#endif
}
*/
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
extern byte alphabets[][16] = { { B00000000,
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
                                  B00000000 },
                                { B00011000,
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
                                  B11000011 } };

void drawLineT(int bar, char pismeno, int sloupec) {
  int alphabetIndex = msg[pismeno] - '@';
  if (alphabetIndex < 0) alphabetIndex = 0;

  for (int i = 0; i < 18 / 3; i += 3) {  //draw one band/bar  (INPUT >> N) & 1;
    pixels.setPixelColor(bar * 18 + i, pixels.Color(
                                         ((alphabets[alphabetIndex][i] >> (7 - sloupec)) & 1) ? ledBrightness : 0,
                                         ((alphabets[alphabetIndex][i + 1] >> (7 - sloupec)) & 1) ? ledBrightness : 0,
                                         ((alphabets[alphabetIndex][i + 2] >> (7 - sloupec)) & 1) ? ledBrightness : 0));
  }
}

void songRefresh(void) {
  static int currCulNoAct = 0;
  static unsigned long songTime = millis();
  if (currCulNo) {
    if (millis() - songTime > 50) {
      pixels.clear();
      if (currCulNo > currCulNoAct) {
        for (int i = 0; i < NUM_BANDS; i++) {
          drawLineT(i, msg[(currCulNoAct - NUM_BANDS + 1 + i) / 10], (currCulNoAct - NUM_BANDS + 1 + i) % 10);
        }
      } else {
        currCulNo = 0;
        //test, zda oz odrolovano
        //pokud ne, odroluj
      }
      pixels.show();
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
  int msgLength = max(strlen(info), sizeof(msg) - 1);
  strncpy(msg, info, msgLength);
  msg[msgLength] = '\0';
  currCulNo = msgLength * 10;
  Serial.print("msg ");
  Serial.println(msg);
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