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

// Firmware to be installed on electronic board to be put on the body of the cello.
// Latest update: May 2017. Alex Nieva.
// The Thing Development board by Sparkfun: https://www.sparkfun.com/products/13711
// OSC communication using the OSCMessage.h library by: https://github.com/CNMAT/OSC
// Using NeoPixelBus.h by: https://github.com/Makuna/NeoPixelBus

/*
// This script will work for the 134 strip of neopixels.
// This script creates 4 lighting patterns and creates a real-time visual effect 
// based on gaussian shaped addressing of LEDs depending on the RMS value of the sound loudness. 
*/

#include <NeoPixelBus.h>
#include <SPI.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

const uint16_t PixelCount = 134; // this example assumes 4 pixels, making it smaller will cause a failure
//const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

// For Esp8266, the Pin is omitted and it uses GPIO3 due to DMA hardware use.  
// There are other Esp8266 alternative methods that provide more pin options, but also have
// other side effects.
//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount);

#define colorSaturation 128
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(120, 128, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);
RgbColor ADCRMS(0);

// Pattern types supported:
enum  pattern { NONE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE, RANDOM };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

int LEDarray[PixelCount];

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
  int OldPix; 
  uint16_t Pix;
  uint16_t Tail = 1;
  
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
              case RANDOM:
                  RandomUpdate();
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
        if (Index >= TotalSteps + 4)
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
          if (Index >= TotalSteps)
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
      Tail = 1;

//      for (int i = 0; i < (Pix+1); i++)
//      {
//        //RgbColor temp;
//        if (i == Index)  // Scan Pixel to the right
//        {
//             SetPixelColor(i, Color1);
//             //temp = GetPixelColor(i);
//             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
//        }
//        else if (i == TotalSteps - Index) // Scan Pixel to the left
//        {
//             SetPixelColor(i, Color1);
//             //temp = GetPixelColor(i);
//             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
//        }
//        else // Fading tail
//        {
//             SetPixelColor(i, DimColor(GetPixelColor(i)));
//             //temp = GetPixelColor(i);
//             //Serial.print("R: "); Serial.print(temp.R); Serial.print("G: "); Serial.print(temp.G); Serial.print("B: "); Serial.println(temp.B); 
//        }
//      }
//      Show();
//      Increment();

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
      temp.R = ((Color1.R * (TotalSteps - Index)) + (Color2.R * Index)) / (TotalSteps+1);
      temp.G = ((Color1.G * (TotalSteps - Index)) + (Color2.G * Index)) / (TotalSteps+1);
      temp.B = ((Color1.B * (TotalSteps - Index)) + (Color2.B * Index)) / (TotalSteps+1);
      //Serial.print("Index: "); Serial.println(Index);
      ColorSet(temp);
      Show();
      Increment();
  }

  void Random(uint8_t interval,direction dir = FORWARD)
  {
     ActivePattern = RANDOM;
     Index = 0;
     Direction = dir;
     TotalSteps = PixelCount();
     
     for (int i = 0; i<PixelCount(); i++)
      {
        LEDarray[i] = i;
      }
    
    const size_t n = sizeof(LEDarray) / sizeof(LEDarray[0]);

    for (size_t i = 0; i < n - 1; i++)
    {
        size_t j = random(0, n - i);
    
        int t = LEDarray[i];
        LEDarray[i] = LEDarray[j];
        LEDarray[j] = t;
    }
  
    for (int i = 0; i < PixelCount(); i++)
    {
      SetPixelColor(i,white);
    }
    Show();
  }

  void RandomUpdate() {
  
    //for (int i = 0; i<PixelCount(); i++)
    //{
      SetPixelColor(LEDarray[Index],black);
      Show();
      Increment();
      //delay(10);
    //}
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
IPAddress ipClient(192, 168, 4, 3);
IPAddress Subnet(255, 255, 255, 0);

/////////////////////
// UDP Definitions //
/////////////////////
WiFiUDP UDP;
const int portRemote = 8888;        //outgoing port
IPAddress ip(192, 168, 4, 3);       //device receiving
const int portLocal = 6448;         //incoming port
boolean udpConnected = false;


static float bufferFromGloveFloat[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float oldBufferFromGloveFloat[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float bufferFromPiezo[1] = {0.0};

#define DEBUG true

void initHardware(void);
boolean connectUDP(void);
boolean connectWifi(void);

long deltaRead = 50;
long previousMillis = 0;

boolean FLAG1 = false;
boolean FLAG2 = false;
boolean FLAG3 = false;
boolean FLAG_AMP = false;

int Trigger1 = 0;
long previousTrigger1 = 0;
int Trigger2 = 0;
long previousTrigger2 = 0;
int countON;

float oldNormAcc = 0;
uint8_t flag1 = 0;
uint8_t output1 = 0;
uint8_t flag2 = 0;
uint8_t output2 = 0;

long deltaColorChange = 30000;
long previousColorChange = 0;
byte ColorCount = 0;

long StartTime = 0;
long TotalTime = 0;
boolean TimeFlag = false;

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
    
    // this resets all the neopixels to an off state
    Stick.Begin();


    Serial.println();
    Serial.println("Running...");
    Stick.ColorSet(black);
    yield();
}


void loop()
{
  if (TotalTime - previousColorChange > deltaColorChange){ //Slight color variation during performance
    previousColorChange = TotalTime;
    ColorCount++;
    ColorCount = ColorCount%45;
    //Serial.print("ColorCount: "); Serial.println(ColorCount);
    white.R = (128 - ColorCount);
    white.G = (128 - ColorCount);
    //Serial.print("blue.R: "); Serial.println(white.R);
    //Serial.print("blue.G: "); Serial.println(white.G);
    //blue.B = (128 - ColorCount);
    //Stick.ColorSet(blue);
  }

  
  if (!FLAG1 && !FLAG2 && !FLAG3) { // Flags that activate display patterns
    //Serial.println("Passed Flags");
    if (millis() - previousMillis > deltaRead){
      OSCMsgReceive();
      previousMillis = millis();
      if (DEBUG)
      {
        Serial.print(bufferFromGloveFloat[0]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[1]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[2]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[3]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[4]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[5]); Serial.print(", ");
        Serial.print(bufferFromGloveFloat[6]); Serial.print(", ");
        Serial.println(bufferFromGloveFloat[7]);
      }
      int val = bufferFromGloveFloat[7];
      //Serial.print("val: "); Serial.println(val);
  
      if (val) {
        if (!TimeFlag){
          StartTime = millis(); 
          TimeFlag = true;    
        }
        else{
          TotalTime = TotalTime + millis() - StartTime;
          StartTime = millis();
          //Serial.print("TotalTime: "); Serial.println(TotalTime);
        }
        
        if(bufferFromGloveFloat[6]<0) {
          int sigmaQuad = map(val,0,420,300,7);
          //Serial.print("sigmaQuad: "); Serial.println(sigmaQuad);
          float mu = 72*abs(tanh((bufferFromGloveFloat[6]-15)*PI/180/0.3));
          //Serial.print("mu: "); Serial.println(mu);
          float a = 150/sqrt(sigmaQuad)*sqrt(2*PI);
          float c = sqrt(sigmaQuad);
    
          for (int i=0; i<Stick.PixelCount(); i++) {
            int ValueForPixel = a*exp(-pow((i-mu),2)/(2*pow(c,2)));
            //Serial.print("ValueForPixel: "); Serial.println(ValueForPixel);
            Stick.SetPixelColor(i,ValueForPixel);
          }
          FLAG_AMP = true;
          Stick.Show();
          if (millis() - previousTrigger1 > 120){
            previousTrigger1 = millis();
            checkFlags();
            updateBuffer();
            if (FLAG1 || FLAG2 || FLAG3) {
              initPattern();
            }
          }
        }
      }
      if (val == 0 && FLAG_AMP == true) {
        Serial.println("I'm in val = 0");
        if (TimeFlag){
          TimeFlag = false;
//          Serial.print("TotalTime: "); Serial.println(TotalTime);
        }
        for (int p = 0; p<5; p++){
          for (int i = 0; i<Stick.PixelCount(); i++){
            Stick.SetPixelColor(i,Stick.DimColor(Stick.GetPixelColor(i)));
          }
          Stick.Show();          
        }
        Stick.ColorSet(black);
        FLAG_AMP = false;
        FLAG1 = false;
        FLAG2 = false;
        FLAG3 = false;
        UDP.flush();
      }
    }
  }
  else {
    Stick.Update();
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
    Serial.println("Complete");
    Stick.ColorSet(black);
    FLAG1 = false;
    //Serial.print("FLAG1: "); Serial.println(FLAG1);
    FLAG2 = false;
    //Serial.print("FLAG2: "); Serial.println(FLAG2);
    FLAG3 = false;
    //Serial.print("FLAG3: "); Serial.println(FLAG2);
    FLAG_AMP = false;
    for (int i = 0; i<8; i++) {
      bufferFromGloveFloat[i]=0.0;
      oldBufferFromGloveFloat[i] = 0.0;
    }
    UDP.flush();
}


