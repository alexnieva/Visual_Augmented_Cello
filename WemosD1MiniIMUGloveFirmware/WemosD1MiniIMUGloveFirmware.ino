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

// Firmware to be installed in Glove to be worn by cellist on the bowing hand (right hand).
// Latest update: May 2017. Alex Nieva.
// Microcontroller board: Wemos D1 mini - based on the ESP8266. 
// https://github.com/esp8266/Arduino/tree/master/variants/d1_mini
// LSM9DS0 Breakout IDMIL version by Ian Hattwick
// OSC communication using the OSCMessage.h library by: https://github.com/CNMAT/OSC
// Filter bank using the filters.h library by: https://github.com/JonHub/Filters

/*****************************************************************
Based on...
LSM9DS0_AHRS.ino
SFE_LSM9DS0 Library AHRS Data Fusion Example Code
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 18, 2014
https://github.com/sparkfun/LSM9DS0_Breakout

Modified by Kris Winer, April 4, 2014

The LSM9DS0 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C.

This Arduino sketch is a demo of the simple side of the
SFE_LSM9DS0 library. It'll demo the following:
* How to create a LSM9DS0 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS0 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the calcAccel(), calcGyro() and calcMag()
  functions.
* How to use the data from the LSM9DS0 to calculate orientation
  and heading.

In addition, the sketch will demo:
// Not implemented (an) * How to check for data updates using interrupts
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer anti-aliasing (low-pass) filter rate
* How to use the data from the LSM9DS0 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.


Hardware setup: This library supports communicating with the
LSM9DS0 over either I2C or SPI. If you're using I2C, these are
the only connections that need to be made:
This project uses the Wemos D1 mini microcontroller board. 
	LSM9DS0 --------- Wemos D1
	 SCL ---------- D1 (GPIO5)
	 SDA ---------- D2 (GPIO4)
	 VDD ------------- 3.3V
	 GND ------------- GND

The LSM9DS0 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.	  

*****************************************************************/

// The SFE_LSM9DS0 requires both the SPI and Wire libraries.
// Unfortunately, you'll need to include both in the Arduino
// sketch, before including the SFE_LSM9DS0 library.
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include <Filters.h>

#define DEBUG true

//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPsw[] = "password";
const char WiFiAPName[] = "UDP_master";
boolean wifiConnected = false;

WiFiUDP UDP;                               // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,4,2);        // remote IP of your computer
const IPAddress outIp2(192,168,4,3);
const IPAddress outIp3(192,168,4,4);
const unsigned int portRemote = 6448;      // remote port to receive OSC
const unsigned int portLocal = 8888;       // local port to listen for OSC packets (actually not used for sending)
boolean udpConnected = false;
//char incomingPacket[255];  // buffer for incoming packets
static float bufferFromPiezo[2] = {0.0, 0.0};

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
//#define PRINT_CALCULATED
//#define PRINT_RAW
//#define PRINT_SPEED 20 // 500 ms between prints

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float outAccel[3] = {0, 0, 0};
float outAccelBodyFrame[3] = {0, 0, 0};
float normAccel;
float outGyro[3] = {0, 0, 0};
float outMag[3] = {0, 0, 0};
float a,b,c,d,e,f,g,h,i;

uint32_t count = 0;  // used to control display output rate
uint32_t delt_t = 0; // used to control display output rate
float pitch, yaw, roll, heading;
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float temperature;

float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

uint32_t AccelDeltaRead = 5000; //microseconds
uint32_t PreviousAccelRead = 0;
uint32_t GyroDeltaRead = 5263; //microseconds
uint32_t PreviousGyroRead = 0;
uint32_t MagDeltaRead = 80000; //microseconds
uint32_t PreviousMagRead = 0;
  
// ESP8266 functions
void initHardware(void);
boolean connectUDP(void);
boolean connectWifi(void);


// Filtering setup
float filterBankFreq[2] = {2, 5};
FilterOnePole filterOneLowpass( LOWPASS, filterBankFreq[0]);
RunningStatistics inputStats;
FilterOnePole filterOneLowpassX( LOWPASS, filterBankFreq[0]);
RunningStatistics inputStatsX;
FilterOnePole filterOneLowpassY( LOWPASS, filterBankFreq[0]);
RunningStatistics inputStatsY;
FilterOnePole filterOneLowpassZ( LOWPASS, filterBankFreq[0]);
RunningStatistics inputStatsZ;

void setup()
{
  initHardware();
  
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  
  // begin() returns a 16-bit value which includes both the gyro 
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  Wire.setClock(400000L);
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();

  // Set data output ranges; choose lowest ranges for maximum resolution
  // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
  dof.setAccelScale(dof.A_SCALE_4G);
  // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
  dof.setGyroScale(dof.G_SCALE_245DPS);
  // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
  dof.setMagScale(dof.M_SCALE_2GS);
  
  // Set output data rates  
  // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
  //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
  dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 200 Hz

  
  // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
  // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
  // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
  // to get a smooth ~150 Hz filter update rate
  dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
  
  // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
  //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

  
  // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
  dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms

  
  // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
  // all subsequent measurements.
  dof.calLSM9DS0(gbias, abias);
  delay(5000);

  wifiConnected = connectWifi();
  if(wifiConnected) udpConnected = connectUDP();
//
//  while (WiFi.status() != WL_CONNECTED) {
//      delay(500);
//      Serial.print(".");
//  }
//  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Local port: ");
  Serial.println(UDP.localPort());
  delay(2000);


}

void loop()
{
  OSCMsgReceive();  // Receive data from TheThingDev board on the bridge. Info about piezo detecting
                    // vibrations of the top of the instrument.

  if ((micros() - PreviousGyroRead) > GyroDeltaRead){
    PreviousGyroRead = micros();
    printGyro();  // Print "G: gx, gy, gz"
  }
  if ((micros() - PreviousAccelRead) > AccelDeltaRead){
    PreviousAccelRead = micros();
    printAccel(); // Print "A: ax, ay, az"
  }
  if ((micros() - PreviousMagRead) > MagDeltaRead){
    PreviousMagRead = micros();
    printMag();   // Print "M: mx, my, mz"
  }
  
  //Serial.println();
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  //Serial.print("deltat = "); Serial.println(deltat,6);

  MadgwickQuaternionUpdate(outAccel[0], outAccel[1], outAccel[2], outGyro[0]*PI/180.0f, outGyro[1]*PI/180.0f, outGyro[2]*PI/180.0f, outMag[0], outMag[1], outMag[2]);
  
  //Getting rid of gravity
  //This matrix is the cosine matrix derive from the updated quaternion
  //This is its equivalent in terms of rotation.
  //a = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
  //b = 2.0f*q[1]*q[2] + 2.0f*q[0]*q[3];
  c = 2.0f*q[1]*q[3] - 2.0f*q[0]*q[2];
  //d = 2.0f*q[1]*q[2] - 2.0f*q[0]*q[3];
  //e = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
  f = 2.0f*q[2]*q[3] + 2.0f*q[0]*q[1];
  //g = 2.0f*q[1]*q[3] + 2.0f*q[0]*q[2];
  //h = 2.0f*q[2]*q[3] - 2.0f*q[0]*q[1];
  i = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  //this is substracting the acceleration due to gravity
  outAccelBodyFrame[0] = outAccel[0] - c;
  outAccelBodyFrame[1] = outAccel[1] - f;
  outAccelBodyFrame[2] = outAccel[2] - i;
  
  normAccel = sqrt(outAccelBodyFrame[0] * outAccelBodyFrame[0] + outAccelBodyFrame[1] * outAccelBodyFrame[1] + outAccelBodyFrame[2] * outAccelBodyFrame[2]);
  if (normAccel == 0.0f) return; // handle NaN

//  //Filtering............
//  RunningStatistics inputStats;
  inputStats.setWindowSecs(deltat);
  inputStatsX.setWindowSecs(deltat);
  inputStatsY.setWindowSecs(deltat);
  inputStatsZ.setWindowSecs(deltat);
  
  inputStats.input(normAccel);
  inputStatsX.input(abs(outAccelBodyFrame[0]));
  inputStatsY.input(abs(outAccelBodyFrame[1]));
//  inputStatsZ.input(abs(outAccelBodyFrame[2]));
  
  //Lowpass filter
  filterOneLowpass.input(normAccel); // 
  filterOneLowpassX.input(abs(outAccelBodyFrame[0]));
  filterOneLowpassY.input(abs(outAccelBodyFrame[1]));
  filterOneLowpassZ.input(abs(outAccelBodyFrame[2]));

  // data sending over WiFi
  delt_t = millis() - count;
  if (delt_t > 50) { // sending data at 20Hz.
    
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth. 
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination), 
    // looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      //yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      yaw -= -14.8; // Declination at Montreal, QC.-14.49
      roll  *= 180.0f / PI;

      // OSC broadcast message to all devices in network.
      OSCMessage msg("/aknow");
//      msg.add(outAccelBodyFrame[0]);	// X acceleration raw
      msg.add(filterOneLowpassX.output()); //X acceleration low pass filtered
      //msg.add(outAccelBodyFrame[1]); 	// Y acceleration raw
      msg.add(filterOneLowpassY.output()); //Y acceleration low pass filtered
//      msg.add(outAccelBodyFrame[2]); 	// Z acceleration raw
      msg.add(filterOneLowpassZ.output()); //Z acceleration low pass filtered
      msg.add(normAccel);		// Norm acceleration raw
      msg.add(filterOneLowpass.output()); // Norm acceleration low pass filtered
      msg.add(pitch);
      msg.add(roll);
      if ((int)bufferFromPiezo[1]> 0){
        msg.add(bufferFromPiezo[0]); //Serial.println(bufferFromPiezo[0]);
      }
      else{
        msg.add(0.0f); //Serial.println(bufferFromPiezo[0]);
      }
      UDP.beginPacket(outIp, portRemote);
      msg.send(UDP);
      UDP.endPacket();
      UDP.beginPacket(outIp3, portRemote);
      msg.send(UDP);
      UDP.endPacket();
      UDP.beginPacket(outIp2, portRemote);
      msg.send(UDP);
      UDP.endPacket();
      msg.empty();

      count = millis();
  }
}

float filter(float data, float filterVal, float smoothedVal){
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .7;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
//    float _2q1q2 = 2.0f * q1 * q2; //an
    float _2q1q3 = 2.0f * q1 * q3;
//    float _2q1q4 = 2.0f * q1 * q4; //an
//    float _2q2q3 = 2.0f * q2 * q3; //an
//    float _2q2q4 = 2.0f * q2 * q4; //an
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    //normAccel = norm; //an
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
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
   
   WiFi.mode(WIFI_AP_STA);
   WiFi.softAP(WiFiAPName, WiFiAPPsw);
   
   if (DEBUG)
   {
      Serial.println("");
      Serial.println("WiFi Network Created");
   }
   return state;
}
