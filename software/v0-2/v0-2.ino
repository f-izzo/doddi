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

typedef struct color{
  int r;
  int g;
  int b;
} color;

typedef struct face{
  boolean resPresent;
  int resID;
  color resColor;
  boolean isActive;
} face;

int faceChange = 1;
boolean endgame = false;
int vib1 = 16;
int vib2 = 17;
int8_t activeF = 1;

//Faces states
//TODO: turn this vector into a structure
face F[12] = {0};
/* F[][0] -->  is there resource on it? 0 – no, 1 - yes
   F[][1] --> resource ID
   F[][2] --> R color value
   F[][3] --> B color value
   F[][4] --> G color value
   F[][5] --> is active? */

//Resources only described by color. For now only 4 resources
color resType[4] = {{255,255,0}, //type1: yellow
                    {0,200,0},   //type2: green
                    {0,0,200},   //type3: blue
                    {0,200,0}};  //type4: green

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
  /* Hardware initialization */
  pinMode (vib1, OUTPUT);
  pinMode (vib2, OUTPUT);
  AudioMemory(100);
  SPI.begin();
  Serial.begin(9600);
  while(!(SD.begin(10))) {
    Serial.println("Unable to access the SD card");
    delay(500);
  }
  pixels.begin();
  //wipe the pixels
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  if (!accel.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  //displaySensorDetails();

  /* Data structure initialization */
  // Initialize faces array
  memset(&F, 0, 12 * sizeof(face));
  // Random assignment of resources to faces.
  // in the future it will read the amount from the json
  for(int i = 0; i < 12; i++) {
    F[i].resID = random(0, 3);
  }
  // Copy resource color to faces based on resID
  for (int i = 0; i < 12; i++) {
    F[i].resColor = resType[F[i].resID];
  }
}

void loop(void)
{
  /* Game start */
  boolean start = readRoll(50, 150);
  // Old game start condition temporarily replaced with delay
  /*while (!start) {
    Serial.println("Waiting for start... ");
    Serial.println(start);
    //delay(1000);
    start = readRoll(50, 150);
  }*/
  delay(10000);
  Serial.println("START TO PLAY!!                SOUND FEEDBACK-->intro story");
  // sound.play("1.wav");
  Serial.println("Start playing");
  playSdWav1.play("start1.wav");
  delay(50); // wait for library to parse WAV info
  // Set all pixels to purple
  for (int i = 0; i < 12; i++) {
    pixels.setPixelColor(i, pixels.Color(150, 0, 150));
    pixels.show();
    delay(50);
  }
  //TODO: Add main game loop
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  //  delay(10);

  // Set faces color to corresponding resources
  for (int i = 0; i < 12; i++) {
    if (F[i].isActive == 0) {
      pixels.setPixelColor(i, pixels.Color(F[i].resColor.r, F[i].resColor.g, F[i].resColor.b));
    }
  }

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
      // Set all pixels to purple
      for (int i = 0; i < 12; i++) {
        pixels.setPixelColor(i, pixels.Color(150, 0, 150));
        pixels.show();
        delay(30);
      } // Turn off all pixels
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
//  Serial.print(F[0].isActive);
//  Serial.print("    ");
//  Serial.print("Face2: ");
//  Serial.print(F[1].isActive);
//  Serial.print("    ");
//  Serial.print("Face3: ");
//  Serial.println(F[2].isActive);
  //Highlight[2]
  if (F[activeF].isActive == 0) {
  //  Serial.print("We're here      Face resource is:");
  //  Serial.println(F[activeF].resID);
    // simple blink
       // if (F[activeF].resID == 0) {
          pixels.setPixelColor(activeF, pixels.Color(F[activeF].resColor.r, F[activeF].resColor.g, F[activeF].resColor.b));
           pixels.show();
           delay(100);
           pixels.setPixelColor(activeF, pixels.Color(0, 0, 0));
          Serial.print("resource = ");
          Serial.print(F[activeF].resID);
          Serial.print("            R= ");
          Serial.print(F[activeF].resColor.r);
          Serial.print("            G= ");
          Serial.print(F[activeF].resColor.g);
          Serial.print("            B= ");
          Serial.print(F[activeF].resColor.b);
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
  //activating the face sequence
  while (activ) {
    //tell me what was activated
    Serial.print("Face nr ");
    Serial.print(activeF + 1);
    Serial.println(" has been activated ");

    //change color of the activated face to purple
    pixels.setPixelColor(activeF, pixels.Color(150, 0, 150));
    pixels.show();
    //set face as activated
    F[activeF].isActive = 1;
    //face activation sound here

    // F[].resID --> resource ID

    if (F[activeF].resID == 0) {
      playSdWav1.play("s1.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF].resID == 1) {
      playSdWav1.play("s2.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF].resID == 2) {
      playSdWav1.play("s3.wav");
      delay(50); // wait for library to parse WAV info
    }
    else if (F[activeF].resID == 3) {
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
  //turn off the pixels
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  boolean facesLeft = true;
  int x = 0;
  while (facesLeft) {
    //get the upward face
    activeF = getFace();
    //check if there are still not activeated faces
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
    //Highlight
    if (F[activeF].resPresent == 0) {
      pixels.setPixelColor(activeF, pixels.Color((F[activeF].resColor.r + 50), (F[activeF].resColor.g + 50), (F[activeF].resColor.b + 50)));
      F[activeF].resPresent = 1;
      pixels.show();

      Serial.println("We're inside highlighting if");
      Serial.println( F[activeF].resPresent );
    }
    //    else  if(F[activeF].resPresent == 1){
    //      playSdWav1.play("loose.wav");
    //      delay (50);

    //      Serial.println("We're inside else if");
    //    Serial.println( F[activeF].resPresent );
    ////      //zeroing faces
    ////
    ////       facesLeft = false;
    ////
    //    }
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
