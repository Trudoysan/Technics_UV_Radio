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
#include <ArduinoOTA.h>
#include <arduinoFFT.h>
//#include <SPI.h>


//#define DEBUG

//BLE - ON/OFF (BLUE)
//VU - brightness up, down, bar level  up, down, on/off by switch
//radio - ON/OFF(YELLOW/other), BAR(GREEN), TOP (RED), station up, down

#define AUDIO_IN_PIN 35  // Signal in on this pin - ADC1_CHANNEL_7

#define WS_PIN 13
//#define SONG_PIN 36
#define ST_UP_PIN 16       // In
#define ST_DOWN_PIN 17     // In
#define LIGHT_UP_PIN 18    // In
#define LIGHT_DOWN_PIN 19  // In
#define TOP_PIN 34         // In
#define BAR_PIN 33          // In
#define LVL_UP_PIN 36      // In
#define LVL_DOWN_PIN 39    // In
#define RADIO_PIN 4
int buttonPins[] = { ST_UP_PIN, ST_DOWN_PIN, LIGHT_UP_PIN, LIGHT_DOWN_PIN, LVL_UP_PIN, LVL_DOWN_PIN };
#define NUM_BUTTONS 6 

//#define SONG_LED_PIN 4  // Out
#define BAR_LED_PIN 22  // Out
#define TOP_LED_PIN 21  // Out
#define RADIO_PIN_LED 23

// Digital I/O used
#define I2S_DOUT 26  // DIN connection
#define I2S_BCLK 27  // Bit clock
#define I2S_LRC 25   // Left Right Clock

#define SAMPLES 1024         // Must be a power of 2
#define SAMPLING_FREQ 40000  // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
int amplitude = 1000;               // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.

Audio audio;

Preferences pref;
int infolnc = 0;  //stores channel number for tuner
int ledBrightness = 127; //store led Brightness, maybe different for different colours?

bool bar=1, top=1, radioOn;

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


#define SSID "****"
#define PSK "****"

#define STATIONS 15
char *stationlist[STATIONS] = {
  "http://vis.media-ice.musicradio.com/CapitalMP3",
  "https://panel.retrolandigital.com/listen/oldies_internet_radio/listen",
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
  "http://streaming.live365.com/b92108_128mp3",
  "https://samcloud.spacial.com/api/listen?sid=111415&rid=194939&f=mp3,any&br=128000,any&m=sc",
  "https://0n-80s.radionetz.de/0n-80s.mp3"
};

char msg[50];

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
  ArduinoOTA
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

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("WIFI Started");

  //start preferences instance
  pref.begin("radio", false);
  //set current amplitude
  if (pref.isKey("amplitude")) amplitude = pref.getUShort("amplitude");
  if(amplitude<10) amplitude = 10;
    //set current ledBrightness
  if (pref.isKey("ledBrightness")) ledBrightness = pref.getUShort("ledBrightness");
  if(ledBrightness>255) ledBrightness = 255;
  if(ledBrightness<10) ledBrightness = 10;
    //set current station to saved value if available
  if (pref.isKey("station")) infolnc = pref.getUShort("station");
  if (infolnc >= STATIONS) infolnc = 0;


  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(21);  // 0...21
  audio.connecttohost(stationlist[infolnc]);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));  //1.5??

  for (int i = 0; i < NUM_BUTTONS; i++)
    pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(TOP_PIN, INPUT_PULLUP);
  pinMode(BAR_PIN, INPUT_PULLUP);
  pinMode(RADIO_PIN, INPUT_PULLUP);
  pinMode(AUDIO_IN_PIN, INPUT);  // Signal in on this pin
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  pinMode(BAR_LED_PIN, OUTPUT);
  pinMode(TOP_LED_PIN, OUTPUT);
  pinMode(RADIO_PIN_LED, OUTPUT);

  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
}


void loop() {
  ArduinoOTA.handle();
  audio.loop();
  songRefresh();
  //buttonCheck();

  /* static unsigned long cas3 = millis();
  if (millis() - cas3 > 30000) {
    stationUpDown(1);
    cas3 = millis();
    //  pixels.setPixelColor(infolnc, pixels.Color(0, 127, 255));
    //  pixels.show();
  }
  */
  static unsigned long cas2 = millis();
  static int led = 0;
  if (millis() - cas2 > 100) {
    if (!(led / 18))
      pixels.clear();
    //pixels.setPixelColor(led, pixels.Color(ledBrightness, ledBrightness, ledBrightness));
    int ledMod3 = led % 3;
    pixels.setPixelColor(led / 3, pixels.Color(ledMod3 ? 0 : ledBrightness, (ledMod3 - 1) ? 0 : ledBrightness, (ledMod3 - 2) ? 0 : ledBrightness));
    pixels.show();
    led += 3;
    if (led > NUM_LEDS / 3) led = 0;
    cas2 = millis();
    //Serial.println(led);
  }

  /* static unsigned long cas4 = millis();
  static int colorDef = 0;
  if (millis() - cas4 > 1000) {
    pixels.clear();
    if (colorDef)
      pixels.fill(pixels.Color(ledBrightness, ledBrightness, ledBrightness));
    else
      pixels.fill(pixels.Color(0, 0, 0));
    pixels.show();
    colorDef++;
    if (colorDef > 1) colorDef = 0;
    cas4 = millis();
    Serial.println(colorDef);
  }*/
  /*
  // Reset bandValues[]
  for (int i = 0; i < NUM_BANDS; i++) {
    bandValues[i] = 0;
  }

  // Sample the audio pin
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN);  // A conversion takes about 9.7uS on an ESP32
    //vReal[i] = adc1_get_raw(ADC1_CHANNEL_7);
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) {
    }
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Analyse FFT results
  for (int i = 2; i < (SAMPLES / 2); i++) {  // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency bin and its value the amplitude.
    if (vReal[i] > NOISE) {                  // Add a crude noise filter

      //8 bands, 12kHz top band
      if (i <= 3) bandValues[0] += (int)vReal[i];
      if (i > 3 && i <= 6) bandValues[1] += (int)vReal[i];
      if (i > 6 && i <= 13) bandValues[2] += (int)vReal[i];
      if (i > 13 && i <= 27) bandValues[3] += (int)vReal[i];
      if (i > 27 && i <= 55) bandValues[4] += (int)vReal[i];
      if (i > 55 && i <= 112) bandValues[5] += (int)vReal[i];
      if (i > 112 && i <= 229) bandValues[6] += (int)vReal[i];
      if (i > 229) bandValues[7] += (int)vReal[i];

      //2,4,7,12,21,38,72,138,240

            //10 bands
      if (i <= 2) bandValues[0] += (int)vReal[i];
      if (i > 2 && i <= 4) bandValues[1] += (int)vReal[i];
      if (i > 4 && i <= 7) bandValues[2] += (int)vReal[i];
      if (i > 7 && i <= 12) bandValues[3] += (int)vReal[i];
      if (i > 12 && i <= 21) bandValues[4] += (int)vReal[i];
      if (i > 21 && i <= 38) bandValues[5] += (int)vReal[i];
      if (i > 38 && i <= 72) bandValues[6] += (int)vReal[i];
      if (i > 72 && i <= 138) bandValues[6] += (int)vReal[i];
      if (i > 138 && i <= 240) bandValues[6] += (int)vReal[i];
      if (i > 240) bandValues[7] += (int)vReal[i];

      //https://stackoverflow.com/questions/10160439/the-way-to-extract-10-band-equalization-information-from-mp3-format

40000/512= 39
1: 0-0
2: 1-1
3: 2-3
4: 4-7
5: 8-15
6: 16-31
7: 32-63
8: 64-127
9: 128-255
10: 256-512
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
      //if ((barHeight > i && bar) || (peak[band] == i + 1 && top))
      pixels.setPixelColor(((band * TOP) + i) / 3,
                           pixels.Color(((barHeight > i && bar) || (peak[band] == i + 1 && top)) ? ledBrightness : 0,
                                        ((barHeight > i + 1 && bar) || (peak[band] == i + 2 && top)) ? ledBrightness : 0,
                                        ((barHeight > i + 2 && bar) || (peak[band] == i + 3 && top)) ? ledBrightness : 0));
      //Serial.println(((band * TOP) + i) / 3);
      //Serial.println(((barHeight > i && bar) || (peak[band] == i + 1 && top)) ? ledBrightness : 0);
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
          switch (i) {
            case 0:  //ST_UP_PIN
              stationUpDown(1);
              break;
            case 1:  //ST_DOWN_PIN
              stationUpDown(-1);
              break;
            case 2:  //LIGHT_UP_PIN
              ledLightSetting(LIGHT_UP_PIN);
              break;
            case 3:  //LIGHT_DOWN_PIN
              ledLightSetting(LIGHT_DOWN_PIN);
              break;
            case 4:  //LVL_UP_PIN
              VUsetting(LVL_UP_PIN);
              break;
            case 5:
              VUsetting(LVL_DOWN_PIN);
              break;
          }
        } else {
          if ((currentMillis - lastPlayDebounceTime[i]) > 500) {
            Serial.print("Long press");
          }
        }
      }
    }
    lastButtonState[i] = buttonRead;
  }

  if (radioOn != digitalRead(RADIO_PIN)) {
    radioOn = !radioOn;
    if (radioOn)
      audio.connecttohost(stationlist[infolnc]);
    else
      audio.stopSong();
  }
  if (top != digitalRead(TOP_PIN)) {
    top = !top;
    if (top)
      digitalWrite(TOP_LED_PIN, HIGH);
    else
      digitalWrite(TOP_LED_PIN, LOW);
  }
  if (bar != digitalRead(BAR_PIN)) {
    bar = !bar;
    if (bar)
      digitalWrite(BAR_LED_PIN, HIGH);
    else
      digitalWrite(BAR_LED_PIN, LOW);
  }
}

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
  if (active)
    Serial.print("Pin activated: ");
  else
    Serial.print("Pin deactivated: ");
  Serial.println(pinNumber);
#endif
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
  if (active)
    Serial.print("Pin activated: ");
  else
    Serial.print("Pin deactivated: ");
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
extern byte alphabets[][16];

int msgLength = 0;
void songRefresh(void) {
  static int currTextNo = 0;
  static unsigned long songTime = millis();

  if (millis() - songTime > 50) {
    if (msgLength) {
      if (currTextNo) {
        //uz neco nakresleno, to posun a dokresli
      } else {
        // prvni sloupec
      }
    } else {
      //test, zda oz odrolovano
      //pokud ne, odroluj
    }
    songTime = millis();
  }
}

void scrollText(char *info) {
  /*int alphabetIndex = msg[charIndex] - '@';
   if (alphabetIndex < 0) alphabetIndex=0;
   */
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
  msgLength = max(strlen(info), sizeof(msg) - 1);
  strncpy(msg, info, msgLength);
  msg[msgLength] = '\0';
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
