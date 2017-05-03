#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <TeensyThreads.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <Audio.h>
#include <SPI.h>
#include <SD.h>
//#include <SerialFlash.h>

//#define ACCEL
//#define ACCEL_DEBUG
#define vib 16
#define INTERRUPT_PIN  5
#define PIN            23   // Pin on the Arduino connected to the NeoPixels
#define NUMPIXELS      8   //Number of NeoPixels attached to the Arduino

boolean shake(uint32_t ms, uint32_t timeout, int small);

MPU6050 mpu;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

AudioPlaySdWav           playSdWav1;
AudioOutputAnalog        dac1;
AudioConnection          patchCord1(playSdWav1, 0, dac1, 0);

//faces coordinates
const int facesYZ[12][2] = {
  { 0,   0}, //  0
  { 20, 50}, //  1
  { 60, -3},//  2
  { 15, -50},//  3
  {-40, -30}, //  4
  {-38,  30}, //  5
  { 50,  30}, //  6
  { 40, -30}, //  7
  {-15, -60}, //  8
  {-69,  -5}, //  9
  {-15,  60}, // 10
  { 0 ,  0},  // 11
};

typedef struct color{
  int r;
  int g;
  int b;
} color;

typedef struct face{
  boolean resPresent;
  int resID;
  uint32_t color;
  boolean isActive;
} face;

const uint32_t yellow = pixels.Color(255, 255, 0);
const uint32_t red = pixels.Color(200, 0, 0);
const uint32_t green = pixels.Color(0, 200, 0);
const uint32_t blue = pixels.Color(0, 0, 200);
const uint32_t purple = pixels.Color(150, 0, 150);
const uint32_t off = pixels.Color(0, 0, 0);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float final_euler[3];
float final_ypr[3];
volatile bool mpuInterrupt = false;

// game global vars
boolean start = false;
boolean endgame = false;
boolean faceChange = false;
uint8_t activeF = 0;
face F[12] = {0};

void maingame() {
  /* Game start */
  Serial.print("Waiting for start... ");
  Serial.println(start);
  start = shake(50, 150, 1);
  if(!start) return;

  Serial.println("START TO PLAY!!                SOUND FEEDBACK-->intro story");
  Serial.println("Start playing");
  playSdWav1.play("start1.wav");
  delay(50); // wait for library to parse WAV info
  setAllPixels(purple);
  delay(500); //Delay to make game slower

  // Set faces color to corresponding resources
  for (int i = 0; i < NUMPIXELS; i++) {
    if (F[i].isActive == 0) {
      pixels.setPixelColor(i, F[i].color);
      pixels.show();
      delay(50);
    }
  }
  
  //NOTE nonsense
  /*boolean endgame = shake(50, 120, 1);
  while (endgame) {
    Serial.print("End Game:           enter animation");
    Serial.println(endgame);
    Serial.println("Starting minigames ");
    playSdWav1.play("start1.wav");   //starting minigame sound
    delay(50); // wait for library to parse WAV info
    // "animation"
    for (int i = 0; i < 3; i++) {
      setAllPixels(purple);
      setAllPixels(off);
    }
    game();
    endgame = shake(50, 170, 1);   //coment this out to stay inside the loop
    //minigame sound play
    Serial.println("End playing");
    //playSdWav1.play("win.wav");
    //delay(50); // wait for library to parse WAV info
  }*/
  Serial.print("Face: ");
  Serial.print(activeF + 1);
  Serial.print("    ");
  Serial.print("Shake: ");
  boolean activ = shake(50, 80, 0);
  Serial.print(activ);
  Serial.println("    ");
  // Face highlighting
  if (F[activeF].isActive == 0) {
    // simple blink
    pixels.setPixelColor(activeF, F[activeF].color);
    pixels.show();
    delay(100);
    pixels.setPixelColor(activeF, off);
    pixels.show();
    Serial.print("resource = ");
    Serial.print(F[activeF].resID);
    //TODO: print resource status from color intensity
  }
  //vibration for choosing
  if (faceChange != activeF)
  {
    Serial.println("the face has changed");
    faceChange = activeF;
    digitalWrite(vib, HIGH);
    delay(150);
    digitalWrite(vib, LOW);
  }
  //Face activation sequence
  while (activ) {
    //tell me what face was activated
    Serial.print("Face nr ");
    Serial.print(activeF + 1);
    Serial.println(" has been activated");
    //change color of the activated face to purple
    pixels.setPixelColor(activeF, purple);
    pixels.show();
    F[activeF].isActive = 1;
    //face activation sound
    switch(F[activeF].resID)
    {
      case 0:
        playSdWav1.play("s1.wav");
        break;
      case 1:
        playSdWav1.play("s2.wav");
        break;
      case 2:
        playSdWav1.play("s3.wav");
        break;
      case 3:
        playSdWav1.play("s4.wav");
        break;
    }
    delay(50); // wait for library to parse WAV info
    activ = shake(50, 100, 0);
    //delay(1000);
  }
}

void arrayInit() {
  // Initialize faces array
  memset(&F, 0, 12 * sizeof(face));
  randomSeed(analogRead(A7));
  // Random assignment of resources to faces.
  for(int i = 0; i < 12; i++) {
    F[i].resID = random(4);
  }
  // Set resource color to faces based on resID
  for (int i = 0; i < 12; i++) {
    switch(F[i].resID)
    {
      case 0:
        F[i].color = blue;
        break;
      case 1:
        F[i].color = yellow;
        break;
      case 2:
        F[i].color = green;
        break;
      case 3:
        F[i].color = red;
        break;
    }
  }
}

void setAllPixels(uint32_t color) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, color);
    pixels.show();
    delay(50);
  }
}

uint8_t splitColor ( uint32_t c, char value ) {
  switch ( value ) {
    case 'r': return (uint8_t)(c >> 16);
    case 'g': return (uint8_t)(c >>  8);
    case 'b': return (uint8_t)(c >>  0);
    default:  return 0;
  }
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void accelinit() {
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

void setup() {
  /* Hardware initialization */
  pinMode(vib, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pixels.begin();
  pixels.show();  // turn off all the pixels
  Serial.begin(9600);
  SPI.begin();
  while(!(SD.begin(10))) {
    Serial.println("Unable to access the SD card");
    delay(500);
  }
  Wire.begin();
  Wire.setClock(400000);
  AudioMemory(100);
  accelinit();
  /* Data structure initialization */
  arrayInit();
  /* Register and start threads */
  //threads.setSliceMillis(200);
  //threads.addThread(maingame);
}
void loop() {
  // The main loop is dedicated to the accelerometer
  VectorInt16 aa;         //accel sensor measurements
  VectorFloat gravity;    //gravity vector
  Quaternion q;
  float euler[3];
  float ypr[3];
  if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
        // The game code is executed while the accelerometer is not working
        maingame();
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        for(int i=0; i<3; i++)
        {
          final_euler[i] = euler[i] * 180/M_PI;
          final_ypr[i] = ypr[i] * 180/M_PI;
        }
        #ifdef ACCEL_DEBUG
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        Serial.print("euler z  ");
        Serial.println(euler[2] * 180/M_PI);
        #endif
        int newF = getFace();
        if(newF != activeF) {
          activeF = newF;
          Serial.print("Active face: ");
          Serial.println(activeF);
        }
    }
}

int getFace() {
  int d, iMin = 0, dMin = 999999;
  float fY, fZ, dY, dZ;
  
  for (int i=0; i < 12; i++) { // For each face...
    fY = (float)facesYZ[i][0]; // Read y and z coordinates
    fZ = (float)facesYZ[i][1];
    dY = final_ypr[1] - fY; // Delta between accelerometer & face
    dZ = final_ypr[2] - fZ; // Delta between accelerometer & face
    d  = dY * dY + dZ * dZ; // Distance^2
    // Check if this face is the closest match so far.  Because
    // we're comparing RELATIVE distances, sqrt() can be avoided.
    if (d < dMin) { // New closest match?
      dMin = d;    // Save closest distance^2
      iMin = i;    // Save index of closest match
    }
  }
  if((iMin == 0) || (iMin == 11)) { //Top and bottom face disambiguation
    int eulerZ = (final_euler[2] > 0) ? final_euler[2] : -final_euler[2];
    iMin = (final_euler[2] > 160) ? 11 : 0;
  }
  return iMin; // Index of closest matching face*/
}

//shake detection
#ifdef ACCEL
boolean shake(uint32_t ms, uint32_t timeout, int small) {
  int d = (small)? 80 : 210; //small or big shake
  uint32_t startTime, prevTime, currentTime;
  int32_t  prevX, prevY, prevZ;
  int32_t  dX, dY, dZ;

  // Get initial orientation and time
  sensors_event_t event;
  #ifdef ACCEL
  accel.getEvent(&event);
  #endif
  prevX    = event.acceleration.x;
  prevY    = event.acceleration.y;
  prevZ    = event.acceleration.z;
  prevTime = startTime = millis();

  // Then make repeated readings until orientation settles down.
  // A normal roll should only take a second or two...if things do not
  // stabilize before timeout, probably being moved or carried.
  while (((currentTime = millis()) - startTime) < timeout) {
    if ((currentTime - prevTime) >= ms) return false; // Stable!
    sensors_event_t event;
    #ifdef ACCEL
    accel.getEvent(&event);
    #endif
    dX = event.acceleration.x - prevX; // X/Y/Z delta from last stable position
    dY = event.acceleration.y - prevY;
    dZ = event.acceleration.z - prevZ;
    // Compare distance.  sqrt() can be avoided by squaring distance
    // to be compared; about 100 units on X+Y+Z axes ((100^2)*3 = 30K)
    if ((dX * dX + dY * dY + dZ * dZ) >= d) { // Big change?
      prevX    = event.acceleration.x;    // Save new position
      prevY    = event.acceleration.y;
      prevZ    = event.acceleration.z;
      prevTime = millis(); // Reset timer
    }
  }
  return true;
}
#endif
#ifndef ACCEL
// Fake shake detection
boolean shake(uint32_t ms, uint32_t timeout, int small) {
  int time = millis();
  while(Serial.read()== -1 && (millis() - time < timeout))
  {
    delay(10);
  }
  if(millis() - time < timeout) return true;
  else return false;
}
#endif

void game(){
  setAllPixels(off);
  boolean facesLeft = true;
  int x = 0;
  while (facesLeft) {
    //get the upward face
    Serial.print("Face active: ");
    Serial.println(activeF);
    //check if there are not active faces left
    for (int i = 0; i < 12; i++) {
      x = x + F[i].resPresent;
    }
    if (x == 12) {
      facesLeft = false;
    }
    else {
      facesLeft = true;
      x = 0;
    }
    //Highlight face
    if (F[activeF].resPresent == 0) {
      uint32_t oldcolor = pixels.getPixelColor(activeF);
      int oldr = splitColor(oldcolor, 'r');
      int oldg = splitColor(oldcolor, 'g');
      int oldb = splitColor(oldcolor, 'b');
      pixels.setPixelColor(activeF, pixels.Color((oldr + 50), (oldg + 50), (oldb + 50)));
      pixels.show();
      F[activeF].resPresent = 1;
    }
  }
  //zeroing faces
  for (int i = 0; i < 12; i++) {
    F[i].resPresent = 0;
  }
  facesLeft = true;
  //minigame sound play
  Serial.println("End playing");
  playSdWav1.play("win.wav");
  delay(50); // wait for library to parse WAV info
}
