#define PIN            23   // Pin on the Arduino connected to the NeoPixels
#define NUMPIXELS      12   //Number of NeoPixels attached to the Arduino

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

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);

/* Assign a unique ID to this sensor */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

AudioPlaySdWav           playSdWav1;
AudioOutputAnalog        dac1;
AudioConnection          patchCord1(playSdWav1, 0, dac1, 0);

//faces coordinates
static const int16_t PROGMEM gtable[12][3] = {
  {  8.1, 0.4, 5.45 }, //  1
  {  4.1, 9.1,   0.2}, //  2
  {   8.3,  0.6,   -5.4}, //  3
  {  5.1,  -8.2,   -0.5}, //  4
  {  -0.3,  -5.42,  8.4}, //  5
  { -0.45,   5.3,   8.6}, //  6
  {  -5.5,  8.5,  -0.3}, //  7
  {   0.0,  5.2,  -8.9}, //  8
  {   0.2, -5.3,  -8.8}, //  9
  {  -5.1, -8.5,  -0.2}, // 10
  {  -8.8,     0,  5.1}, // 11
  {  -8.8, -0.2 , -5.7}, // 12
};

int faceChange = 1;
boolean endgame = false;
int vib1 = 16;
int vib2 = 17;
int8_t activeF = 1;

//Faces states
//TODO: turn this vector into a structure
int F [12] [6];
/* F[][0] -->  is there resource on it? 0 – no, 1 - yes
   F[][1] --> resource ID
   F[][2] --> R color value
   F[][3] --> B color value
   F[][4] --> G color value
   F[][5] --> is active? */

//Resources only described by color. For now only 4 resources
//TODO: turn this vector into a structure
int Res [4] [3];
//Res[][0] - R
//Res[][1] - G
//Res[][2] - B

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  //  delay(500);
}

void setup(void) {
  //setting pins for vibraiton
  pinMode (vib1, OUTPUT);
  pinMode (vib2, OUTPUT);
  AudioMemory(100);
  // initialize SPI:
  SPI.begin();
  // initializa serial
  Serial.begin(9600);
  while(!(SD.begin(10))) {
    Serial.println("Unable to access the SD card");
    delay(500);
  }
  //zeroing faces array
  for(int i = 0; i < 12; i++) {
    for (int j = 0; j < 6; j++) {
      F[i][j] = 0;
    }
  }
  //random assignment of resources to faces. in the end it reads amount from the json
  for(int i = 0; i < 12; i++) {
    F[i][1] = random(0, 3);
  }
  //hard set up of resource's color --> in the end this would be loaded from json file
  //resource 1 rgb(255,255,0)
  Res [0][0] = 255; //R
  Res [0][1] = 255; //G
  Res [0][2] = 0; //B
  //resource 2
  Res [1][0] = 0; //R
  Res [1][1] = 200; //G
  Res [1][2] = 0; //B
  //resource 3
  Res [2][0] = 0; //R
  Res [2][1] = 0; //G
  Res [2][2] = 200; //B
  //resource 4
  Res [3][0] = 0; //R
  Res [3][1] = 200; //G
  Res [3][2] = 0; //B

  //transfer resource color to faces
  for (int i = 0; i < 12; i++) {
    F[i][2] = Res[F[i][1]][0];
    F[i][3] = Res[F[i][1]][1];
    F[i][4] = Res[F[i][1]][2];
  }
  pixels.begin();
  Serial.println("Accelerometer Test"); Serial.println("");
  /* Initialise the sensor */
  if (!accel.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  /* Display some basic information on this sensor */
  //displaySensorDetails();

  //wipe the pixels
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }

  endgame = readRoll(50, 150);
  /*while (!endgame) {
    Serial.println("Waiting for start... ");
    Serial.println(endgame);
    //delay(1000);
    endgame = readRoll(50, 150);
  }*/
  //Delay to skip start game condition
  delay(10000);
  Serial.println("START TO PLAY!!                SOUND FEEDBACK-->intro story");
  // sound.play("1.wav");
  Serial.println("Start playing");
  playSdWav1.play("start1.wav");
  delay(50); // wait for library to parse WAV info
  // animation
  for (int i = 0; i < 12; i++) {
    pixels.setPixelColor(i, pixels.Color(150, 0, 150));
    pixels.show();
    delay(50);
  }
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  //  delay(10);

  for (int i = 0; i < 12; i++) {
    if (F[i][5] == 0) {
      pixels.setPixelColor(i, pixels.Color(F[i][2], F[i][3], F[i][4]));
    }
  }

  //long shake
  boolean endgame = readRoll(50, 120);

  while (endgame) {
    Serial.print("End Game:           enter animation");
    Serial.println(endgame);

    Serial.println("Starting minigames ");
    playSdWav1.play("start1.wav");                                              //starting minigame sound
    delay(50); // wait for library to parse WAV info
    //delay(1000);

    // animation
    for (int i = 0; i < 3; i++) {
      for (int i = 0; i < 12; i++) {
        pixels.setPixelColor(i, pixels.Color(150, 0, 150));
        pixels.show();
        delay(30);
      }
      for (int i = 0; i < 12; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        pixels.show();
        delay(30);
      }
    }
    game();
    endgame = readRoll(50, 170);   //coment this out to stay inside the loop
    //minigame sound play
    Serial.println("End playing");
    //playSdWav1.play("win.wav");
    //delay(50); // wait for library to parse WAV info
  }
  activeF = getFace();
  Serial.print("Face: ");
  Serial.print(activeF + 1);
  Serial.print("    ");
  Serial.print("Shake: ");
  boolean activ = shake(50, 80);
  Serial.print(activ);
  Serial.println("    ");
//  Serial.print("Face1: ");
//  Serial.print(F[0][5]);
//  Serial.print("    ");
//  Serial.print("Face2: ");
//  Serial.print(F[1][5]);
//  Serial.print("    ");
//  Serial.print("Face3: ");
//  Serial.println(F[2][5]);
  //Highlight
  if (F[activeF][5] == 0) {
  //  Serial.print("We're here      Face resource is:");
  //  Serial.println(F[activeF][1]);
    // simple blink
       // if (F[activeF][1] == 0) {
          pixels.setPixelColor(activeF, pixels.Color(F[activeF][2], F[activeF][3], F[activeF][4]));
           pixels.show();
           delay(100);
           pixels.setPixelColor(activeF, pixels.Color(0, 0, 0));
          Serial.print("resource = ");
          Serial.print(F[activeF][1]);
          Serial.print("            R= ");
          Serial.print(F[activeF][2]);
          Serial.print("            G= ");
          Serial.print(F[activeF][3]);
          Serial.print("            B= ");
          Serial.print(F[activeF][4]);
         // Serial.print("highlighting face nr   ");
          //Serial.println(activeF);
  }
  pixels.show();
  //vibration for choosing
  if (faceChange != activeF)
  {
    Serial.println("the face has changed ");
    faceChange = activeF;
    digitalWrite(vib1, HIGH);
    digitalWrite(vib2, HIGH);
    delay(150);
    digitalWrite(vib1, LOW);
    digitalWrite(vib2, LOW);
  }
  //activating the face sequance
  while (activ) {
    //tell me what was activated
    Serial.print("Face nr ");
    Serial.print(activeF + 1);
    Serial.println(" has been activated ");

    //change color of the activated face to
    pixels.setPixelColor(activeF, pixels.Color(150, 0, 150));
    pixels.show();
    //set face as activated
    F[activeF][5] = 1;
    //face activation sound here

    // F[][1] --> resource ID

    if (F[activeF][1] == 0) {
      playSdWav1.play("s1.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF][1] == 1) {
      playSdWav1.play("s2.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF][1] == 2) {
      playSdWav1.play("s3.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF][1] == 3) {
      playSdWav1.play("s4.wav");
      delay(50); // wait for library to parse WAV info
    }

    activ = shake(50, 100);
    //delay(1000);
  }
  //testing the game
  //game();
}

uint32_t getFace(void) {
  int32_t  dX, dY, dZ, d, dMin = 999999;
  int16_t  fX, fY, fZ;
  uint8_t  i, iMin = 0;
  uint16_t addr = 0;
  sensors_event_t event;
  accel.getEvent(&event);

  for ( i = 0; i < 12; i++) { // For each face...
    fX = pgm_read_word(&gtable[i][0]); // Read face X/Y/Z
    fY = pgm_read_word(&gtable[i][1]); // from PROGMEM
    fZ = pgm_read_word(&gtable[i][2]);
    dX = event.acceleration.x - fX; // Delta between accelerometer & face
    dY = event.acceleration.y - fY;
    dZ = event.acceleration.z - fZ;
    d  = dX * dX + dY * dY + dZ * dZ; // Distance^2
    // Check if this face is the closest match so far.  Because
    // we're comparing RELATIVE distances, sqrt() can be avoided.
    if (d < dMin) { // New closest match?
      dMin = d;    // Save closest distance^2
      iMin = i;    // Save index of closest match
    }
  }
  return iMin; // Index of closest matching face
}

//shake detection
boolean shake(uint32_t ms, uint32_t timeout) {
  uint32_t startTime, prevTime, currentTime;
  int32_t  prevX, prevY, prevZ;
  int32_t  dX, dY, dZ;

  // Get initial orientation and time
  sensors_event_t event;
  accel.getEvent(&event);
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
    accel.getEvent(&event);
    dX = event.acceleration.x - prevX; // X/Y/Z delta from last stable position
    dY = event.acceleration.y - prevY;
    dZ = event.acceleration.z - prevZ;
    // Compare distance.  sqrt() can be avoided by squaring distance
    // to be compared; about 100 units on X+Y+Z axes ((100^2)*3 = 30K)
    if ((dX * dX + dY * dY + dZ * dZ) >= 80) { // Big change?
      prevX    = event.acceleration.x;    // Save new position
      prevY    = event.acceleration.y;
      prevZ    = event.acceleration.z;
      prevTime = millis(); // Reset timer
    }
  }
  return true;
}

boolean readRoll(uint32_t ms, uint32_t timeout) {
  uint32_t startTime, prevTime, currentTime;
  int32_t  prevX, prevY, prevZ;
  int32_t  dX, dY, dZ;

  // Get initial orientation and time
  sensors_event_t event;
  accel.getEvent(&event);
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
    accel.getEvent(&event);
    dX = event.acceleration.x - prevX; // X/Y/Z delta from last stable position
    dY = event.acceleration.y - prevY;
    dZ = event.acceleration.z - prevZ;
    // Compare distance.  sqrt() can be avoided by squaring distance
    // to be compared; about 100 units on X+Y+Z axes ((100^2)*3 = 30K)
    if ((dX * dX + dY * dY + dZ * dZ) >= 210) { // Big change?
      prevX    = event.acceleration.x;    // Save new position
      prevY    = event.acceleration.y;
      prevZ    = event.acceleration.z;
      prevTime = millis(); // Reset timer
    }
  }
  return true;
}

void game() {
  //wipe the pixels
  for (int i = 0; i < NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  boolean facesLeft = true;
  int x = 0;
  while (facesLeft) {
    //get the upwards face
    activeF = getFace();
    //check if there are still not activeated faces
    for (int i = 0; i < 12; i++) {
      x = x + F[i][0];
    }
    if (x == 12) {
      facesLeft = false;
    }
    else {
      facesLeft = true;
      x = 0;
    }
    //Highlight
    if (F[activeF][0] == 0) {
      pixels.setPixelColor(activeF, pixels.Color((F[activeF][2] + 50), (F[activeF][3] + 50), (F[activeF][4] + 50)));
      F[activeF][0] = 1;
      pixels.show();

      Serial.println("We're inside highlighting if");
      Serial.println( F[activeF][0] );
    }
    //    else  if(F[activeF][0] == 1){
    //      playSdWav1.play("loose.wav");
    //      delay (50);

    //      Serial.println("We're inside else if");
    //    Serial.println( F[activeF][0] );
    ////      //zeroing faces
    ////
    ////       facesLeft = false;
    ////
    //    }
  }
  //zeroing faces
  for (int i = 0; i < 12; i++) {
    F[i][0] = 0;
  }
  facesLeft = true;
  //minigame sound play
  Serial.println("End playing");
  playSdWav1.play("win.wav");
  delay(50); // wait for library to parse WAV info
}
