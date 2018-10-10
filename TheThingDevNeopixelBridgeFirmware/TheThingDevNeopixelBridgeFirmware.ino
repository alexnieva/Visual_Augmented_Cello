//***************************************************************
// Visual Augmented Cello Project
// Concept: Alex Nieva (Music Technologist)
//          Juan Sebastian Delgado (Cellist)
// Code development: Alex Nieva
// Developed at the Input Devices and Music Interaction Laboratory - IDMIL
// Project funded by the Centre for Interdisciplinary Research in 
// Music Media and Technology - CIRMMT
// 2016 - 2017
// Video: https://youtu.be/zd7dEjJuMaY
//***************************************************************

// Firmware to be installed on electronic board to be put on the bridge of the cello.
// Latest update: May 2017. Alex Nieva.
// The Thing Development board by Sparkfun: https://www.sparkfun.com/products/13711
// OSC communication using the OSCMessage.h library by: https://github.com/CNMAT/OSC
// Filter bank using the filters.h library by: https://github.com/JonHub/Filters
// Using NeoPixelBus.h by: https://github.com/Makuna/NeoPixelBus
// Using MCP3002 library for the ADC based on https://github.com/mark8018/MCP3002

/*
// This sketch has two patterns that respond to accelerometer thresholds. Scanner and Fade.
// This is targeted now to the bridge lighting.
// Signal from ADC gets low pass filtered, then RMS computed on 128 samples, and then the Standard Deviation
// is evaluated to see meaningful changes in Low frequencies of Cello.
// This example will cycle between showing four pixels as Red, Green, Blue, White
// and then showing those pixels as Black.
*/

#include <NeoPixelBus.h>
#include <SPI.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include <MCP3002.h>
#include <Filters.h>

const uint16_t PixelCount = 8; // this example assumes 4 pixels, making it smaller will cause a failure
//const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

// For Esp8266, the Pin is omitted and it uses GPIO3 due to DMA hardware use.  
// There are other Esp8266 alternative methods that provide more pin options, but also have
// other side effects.
//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount);

#define colorSaturation 128
RgbColor red(colorSaturation, 0, 0);
RgbColor dimred3;

RgbColor green(0, colorSaturation, 0);
RgbColor blue(80, colorSaturation, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

// Pattern types supported:
enum  pattern { NONE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE, TILT, TWO_BY_TWO };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public NeoPixelBus<NeoGrbFeature,Neo800KbpsMethod>
{
  public:

  // Member Variables:  
  pattern  ActivePattern;  // which pattern is running
  direction Direction;     // direction to run the pattern
  
  unsigned long Interval;   // milliseconds between updates
  unsigned long lastUpdate; // last update of position
  
  //uint32_t Color1, Color2;  // What colors are in use
  RgbColor Color1, Color2; //Colors according to the library NeoPixelBus
  uint16_t TotalSteps;  // total number of steps in the pattern
  uint16_t Index;  // current step within the pattern
  uint16_t OldPix; 
  uint16_t Pix;
  uint16_t Tail;
  
  void (*OnComplete)();  // Callback on completion of pattern
  
  // Constructor - calls base-class constructor to initialize strip
  NeoPatterns(uint16_t pixels, void (*callback)())
  :NeoPixelBus(pixels)
  {
      OnComplete = callback;
  }

  // Update the pattern
  void Update()
  {
      if((millis() - lastUpdate) > Interval) // time to update
      {
          lastUpdate = millis();
          switch(ActivePattern)
          {
              case THEATER_CHASE:
                  TheaterChaseUpdate();
                  break;
              case COLOR_WIPE:
                  ColorWipeUpdate();
                  break;
              case SCANNER:
                  ScannerUpdate();
                  break;
              case FADE:
                  FadeUpdate();
                  break;
              case TILT:
                  TiltUpdate();
                  break;
              case TWO_BY_TWO:
                  TwoByTwoUpdate();
                  break;
              default:
                  break;
          }
      }
  }
  
// Increment the Index and reset at the end
  void Increment()
  {
    if (Direction == FORWARD)
    {
      Index++;
      if (ActivePattern == SCANNER)
      {
        if (Index > TotalSteps + 4)
        {
          Index = 0;
          if (OnComplete != NULL)
          {
            OnComplete(); // call the comlpetion callback
          }
        }
       }
      else
       {
          if (Index > TotalSteps)
            {
               Index = 0;
               if (OnComplete != NULL)
                  {
                      OnComplete(); // call the comlpetion callback
                  }
            }
       }
    }
    else // Direction == REVERSE
    {
          --Index;
          if (Index <= 0)
          {
              Index = TotalSteps-1;
              if (OnComplete != NULL)
              {
                  OnComplete(); // call the comlpetion callback
              }
          }
    }
  }
    
  // Reverse pattern direction
  void Reverse()
  {
      if (Direction == FORWARD)
      {
          Direction = REVERSE;
          Index = TotalSteps-1;
      }
      else
      {
          Direction = FORWARD;
          Index = 0;
      }
  }  

  // Initialize for One by One
  void TwoByTwo(RgbColor color1, direction dir = FORWARD)
  {
    ActivePattern = TWO_BY_TWO;
    TotalSteps = PixelCount()/2-1;
    Color1 = color1;
    //Index = index;
    Direction = dir;
  }

  // Update the One by One Pattern
  void TwoByTwoUpdate()
  {
    for (int i = 0; i < PixelCount(); i+2)
      {
        //RgbColor temp;
        if (i == Index)  // Scan Pixel to the right
        {
             SetPixelColor(i, Color1);
             SetPixelColor(i+1, Color1);
        }
//        else if (i == TotalSteps - Index) // Scan Pixel to the left
//        {
//             SetPixelColor(i, Color1);
//        }
        else // Turn off all the others
        {
             SetPixelColor(i, black);
             SetPixelColor(i+1, black);
        }
      }
      Show();
      if (Index/2 > TotalSteps-1)
        {
           Index = 0;
           if (OnComplete != NULL)
              {
                  OnComplete(); // call the comlpetion callback
              }
        }
      //Increment();
  }

  // Initialize for a Theater Chase
  void TheaterChase(RgbColor color1, RgbColor color2, uint8_t interval, direction dir = FORWARD)
  {
      ActivePattern = THEATER_CHASE;
      Interval = interval;
      TotalSteps = PixelCount();
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
 }
  
  // Update the Theater Chase Pattern
  void TheaterChaseUpdate()
  {
      for(int i=0; i< PixelCount(); i++)
      {
          if ((i + Index) % 3 == 0)
          {
              SetPixelColor(i, Color1);
          }
          else
          {
              SetPixelColor(i, Color2);
          }
      }
      Show();
      Increment();
  }

  // Initialize for a ColorWipe
  void ColorWipe(RgbColor color, uint8_t interval, direction dir = FORWARD) //changed to RgbColor
  {
      ActivePattern = COLOR_WIPE;
      Interval = interval;
      TotalSteps = PixelCount();
      Color1 = color;
      Index = 0;
      Direction = dir;
  }
  
  // Update the Color Wipe Pattern
  void ColorWipeUpdate()
  {
      SetPixelColor(Index, Color1);
      Show();
      Increment();
  }
  
  // Initialize for a SCANNNER
  void Scanner(RgbColor color1, uint8_t interval) //changed color1 to RgbColor
  {
      ActivePattern = SCANNER;
      Interval = interval;
//      TotalSteps = (PixelCount() - 1) * 2;
      TotalSteps = (Pix) * 2;
      Color1 = color1;
      Index = 0;
      Tail = 2;
  }

  // Update the Scanner Pattern
  void ScannerUpdate()
  { 
      for (int i = 0; i < (Pix+1); i++)
      {
        //RgbColor temp;
        if (i == Index)  // Scan Pixel to the right
        {
             SetPixelColor(i, Color1);
             //temp = GetPixelColor(i);
             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
        }
        else if (i == TotalSteps - Index) // Scan Pixel to the left
        {
             SetPixelColor(i, Color1);
             //temp = GetPixelColor(i);
             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
        }
        else // Fading tail
        {
             SetPixelColor(i, DimColor(GetPixelColor(i)));
             //temp = GetPixelColor(i);
             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
        }
      }
      Show();
      Increment();
  }

  // Tilt
  void Tilt(direction dir = FORWARD)
  {
    ActivePattern = TILT;
    Tail = 3;
    Direction = dir;
    OldPix = 0;
  }
  
  void TiltUpdate()
  {
    ColorSet(black);
    if (Direction == FORWARD){
      for (int i = OldPix+1; i<(Pix+1); i++){
        //Serial.println(i);
        ColorSet(black);
        if (i == 0)
        {
          SetPixelColor(i,white);
          SetPixelColor(i+1,dimred3);
        }
        else if (i == 7)
        {
          SetPixelColor(i,white);
          SetPixelColor(i-1,dimred3);
        }
        else
        {
          SetPixelColor(i,white);
          SetPixelColor(i+1,dimred3);
          SetPixelColor(i-1,dimred3);
        }
        delay(5);
      }
    }
    else if (Direction == REVERSE){
      for (int i = OldPix-1; i>(Pix-1); i--){
        ColorSet(black);
        if (i == 0)
        {
          SetPixelColor(i,white);
          SetPixelColor(i+1,dimred3);
        }
        else if (i == 7)
        {
          SetPixelColor(i,white);
          SetPixelColor(i-1,dimred3);
        }
        else
        {
          SetPixelColor(i,white);
          SetPixelColor(i+1,dimred3);
          SetPixelColor(i-1,dimred3);
        }
      }
      delay(10);      
    }

  Show();
  }
  
  // Initialize for a Fade
  void Fade(RgbColor color1, RgbColor color2, uint16_t steps, uint8_t interval, direction dir = FORWARD) //changed color1 and color2 to RgbColor
  {
      ActivePattern = FADE;
      Interval = interval;
      TotalSteps = steps;
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
  }
  
  // Update the Fade Pattern
  void FadeUpdate()
  {
      // Calculate linear interpolation between Color1 and Color2
      // Optimise order of operations to minimize truncation error
      RgbColor temp;
      temp.R = ((Color1.R * (TotalSteps - Index)) + (Color2.R * Index)) / TotalSteps;
      temp.G = ((Color1.G * (TotalSteps - Index)) + (Color2.G * Index)) / TotalSteps;
      temp.B = ((Color1.B * (TotalSteps - Index)) + (Color2.B * Index)) / TotalSteps;
      Serial.print("Index: "); Serial.println(Index);
      ColorSet(temp);
      Show();
      Increment();
  }
 
  // Calculate 50% dimmed version of a color (used by ScannerUpdate)
  RgbColor DimColor(RgbColor color) //changed color to RgbColor
  {
      // Shift R, G and B components one bit to the right
      RgbColor dimColor;
      dimColor.R = color.R >> Tail;
      dimColor.G = color.G >> Tail;
      dimColor.B = color.B >> Tail;
      //= Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
      return dimColor;
  }

  // Set all pixels to a color (synchronously)
  void ColorSet(RgbColor color) //changed color to RgbColor
  {
      for (int i = 0; i < PixelCount(); i++)
      {
          SetPixelColor(i, color);
      }
      Show();
  }
  
  // Input a value 0 to 255 to get a color value.
  // The colours are a transition r - g - b - back to r.
  RgbColor Wheel(byte WheelPos) //change to RgbColor
  {
      RgbColor temp;
      WheelPos = 255 - WheelPos;
      if(WheelPos < 85)
      {
        temp.R = 255-WheelPos*3;
        temp.G = 0;
        temp.B = WheelPos*3;
        return temp;
          //return temp((uint8_t)(255 - WheelPos * 3), (uint8_t)0, (uint8_t)(WheelPos * 3));
          //return ((uint32_t)(255-WheelPos*3)<<16) | ((uint32_t)(0)<<8) | (WheelPos*3);
      }
      else if(WheelPos < 170)
      {
          WheelPos -= 85;
          temp.R = 0;
          temp.G = WheelPos*3;
          temp.B = 255-WheelPos*3;
          return temp;
          //return temp(0, WheelPos * 3, 255 - WheelPos * 3);
          //return ((uint32_t)0<<16) | ((uint32_t)(WheelPos*3)<<8) | (255-WheelPos*3);
      }
      else
      {
          WheelPos -= 170;
          temp.R = WheelPos*3;
          temp.G = 255-WheelPos*3;
          temp.B = 0;
          return temp;
          //return temp(WheelPos * 3, 255 - WheelPos * 3, 0);
          //return ((uint32_t)(WheelPos*3)<<16) | ((uint32_t)(255-WheelPos*3)<<8) | (0);
      }
  }
};


void StickComplete();

// Define some NeoPatterns for the two rings and the stick
//  as well as some completion routines
NeoPatterns Stick(PixelCount, &StickComplete);


//////////////////////
// WiFi Definitions //
//////////////////////
char ssid[] = "UDP_master";               // your network SSID (name)
char pass[] = "password";                   // your network password
boolean wifiConnected = false;
IPAddress ipServer(192, 168, 4, 1);
IPAddress ipClient(192, 168, 4, 4);
IPAddress Subnet(255, 255, 255, 0);

/////////////////////
// UDP Definitions //
/////////////////////
WiFiUDP UDP;
const int portRemote = 8888;        //outgoing port
IPAddress ip(192, 168, 4, 3);       //device receiving
const int portLocal = 6448;         //incoming port
boolean udpConnected = false;
char  replyPacekt[] = "reply";  // a reply string to send back

static float bufferFromGloveFloat[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

#define DEBUG true

void initHardware(void);
boolean connectUDP(void);
boolean connectWifi(void);

long deltaRead = 30;
long previousMillis = 0;
long startMillis = 0;

boolean FLAG1 = false;
boolean FLAG2 = false;
static int eventCount = -1;

int oldval = 0; // For Tilt effect

//double RMS;
//double alex;
unsigned long intervalRMS = 33;
unsigned long previousRMS = 0;

int audioON = 0;
int oldaudioON = 0;
//float oldSigma = 0;

//**************************************
// ADC Stuff
MCP3002 adc(2); //Pin 2 for CS
//**************************************

/*
 * Filter Definitions
 */
float FrequencyLow = 100;
float FrequencyLow2= 1500;
FilterOnePole filterOneLowpass(LOWPASS, FrequencyLow);
FilterOnePole filterOneLowpass2(LOWPASS, FrequencyLow2);
float windowLength = 0.25;
RunningStatistics RMSstatsLow;


void setup()
{
    initHardware();
    while (!Serial); // wait for serial attach

    Serial.println();
    Serial.println("Initializing...");
    Serial.flush();

    wifiConnected = connectWifi();
    if(wifiConnected) udpConnected = connectUDP();
    delay(1000);

    //initialize ADC
    adc.begin();

    // this resets all the neopixels to an off state
    Stick.Begin();


    Serial.println();
    Serial.println("Running...");
    //Stick.Scanner(red, 10);
    //Stick.ColorWipe(red,55);
//    Stick.Fade(blue,black,16,1);
//    Stick.Tilt(); // Initialization for Tilt of Bow Response
//    dimred3.R = white.R>>3; dimred3.G = white.G>>3; dimred3.B = white.B>>3;
    Stick.ColorSet(black);

    yield();
}


void loop()
{ 
  uint16_t ADCval = adc.analogRead(0);
  filterOneLowpass.input((float)ADCval);
  float HighPassedADCval = (float)ADCval - filterOneLowpass.output(); //DC block
  filterOneLowpass2.input(HighPassedADCval); // 1000 Hz Low pass
  //double RMS = runningAverageRMS(HighPassedADCval); // RMS of full signal.
  double RMS = runningAverageRMS(filterOneLowpass2.output());
  
    // Schmitd Trigger
    if ((RMS >  40.) && (audioON == 0)) {
      audioON = 1;
      startMillis = millis();
      Serial.print("Audio is ON: "); Serial.print(audioON); Serial.print(" RMS: "); Serial.println(RMS);
    }
    if ((RMS < 11.) && (audioON == 1)) {
      Stick.ColorSet(black);
      Serial.print("I'm turning off: "); Serial.print(RMS); Serial.print("audioON: "); Serial.println(audioON);
      audioON = 0;
//      OSCMsgSend(RMS,(float)audioON);
    }
    if(audioON == 1){
      OSCMsgSend(RMS,(float)audioON);
    }
    else{
      OSCMsgSend(0.0,0.0);
    }
//  }

  if (audioON){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > deltaRead)
    {
      previousMillis = currentMillis;
      OSCMsgReceive();
      if (DEBUG)
      {
  //      Serial.print(bufferFromGloveFloat[0]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[1]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[2]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[3]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[4]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[5]); Serial.print(", ");
  //      Serial.print(bufferFromGloveFloat[6]); Serial.print(", ");
  //      Serial.println(bufferFromGloveFloat[7]);
      }

      // Bridge Strip response to Pizzicato Schmidt Trigger
      if (bufferFromGloveFloat[5]> 20.0 && bufferFromGloveFloat[6]> 20.0) {
        if ((RMS > 200.) && (FLAG1 == false)) {
          FLAG1 = true;
          eventCount++;
          eventCount = eventCount%4;
          Serial.print("Eventcount: "); Serial.print(eventCount); Serial.println("FLAG1 true!");
          
        }
//        if ((RMS < 50.) && (FLAG1 == true)) {
//          FLAG1 = false;
//          Serial.println("FLAG1 false!");
//        }
      }
      
      if (FLAG1) {
        if (eventCount == 0 && FLAG2 == false) {
          FLAG2 = true; Serial.println("FLAG2 true");
          int BrightnessPizzicato = (200.0/3.2)*bufferFromGloveFloat[3]; //map((long)bufferFromGloveFloat[3],0,3,0,128);
          RgbColor Pizzicato(BrightnessPizzicato); 
          int DiceValue = random(1,5); 
          //DiceValue = 4;
          Serial.print("DiceValue: "); Serial.println(DiceValue);
          switch (DiceValue) {
            case 1:
              Stick.Fade(Pizzicato,black,16,1);
              break;
            case 2:
              Stick.ColorWipe(Pizzicato,10);
              break;
            case 3:
              Stick.Scanner(Pizzicato,3);
              Stick.Pix = 8;
              Stick.TotalSteps = Stick.Pix*2;
              break;
            case 4:
              Stick.TheaterChase(Pizzicato,black,15);
              break;
            default:
              break;
          }
        }
        if (Stick.ActivePattern == TWO_BY_TWO){
          Stick.Index = eventCount*2;
        }
        Serial.println("FLAG1 working");    
        Serial.println("I'm updating");
        Stick.Update();
      }
      //Serial.println(Stick.OnComplete);
    }
  }
  else {
    FLAG1 = false;
    FLAG2 = false;
  }

  yield();
}

void initHardware()
{
   Serial.begin(115200);
}

boolean connectUDP()
{
   boolean state = false;
   if (DEBUG)
   {
      Serial.println("");
      Serial.println("Connecting to UDP");
   }
   if(UDP.begin(portLocal) == 1)
   {
      if (DEBUG)
         Serial.println("UDP Connection successful");
         Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), portLocal);
      state = true;
   }
   else
   {
      if (DEBUG)
         Serial.println("UDP Connection failed");
   }
   return state;
}

boolean connectWifi()
{
   boolean state = true;
   int i = 0;
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  WiFi.mode(WIFI_STA);
  WiFi.config(ipClient,ipServer,Subnet);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return state;
}

// Stick Completion Callback
void StickComplete()
{
    Stick.ColorSet(black);
    FLAG1 = false;
    FLAG2 = false;
    Serial.println("FLAGS OFF");
}

double runningAverageRMS (float M){
  #define LM_SIZE 64
  static float LM[LM_SIZE];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;

  //update
  sum -= LM[index];
  LM[index] = M*M;
  sum += LM[index];
  index++;
  index = index%LM_SIZE;
  if (count<LM_SIZE) count++;

  return sqrt(sum/count);
}

