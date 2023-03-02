#include <Adafruit_NeoPixel.h>

#define PIN D1	 // input pin Neopixel is attached to

#define NUMPIXELS      58 // number of neopixels in strip

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int add = 0;

int DarkPix = 12;

int storedRed = 60;
int storedGreen = 112;
int storedBlue = 203;

void setup() {
  // Initialize the NeoPixel library.
  pixels.begin();
  
  Serial.begin(115200);
  
  pinMode(D1, OUTPUT);
}

void loop() {
  setColor(add, DarkPix);
  delay(50);
  add = (add + 1) % DarkPix;
  }
    

void setColor(int add, int DarkPix) {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 'p') { //purple
      storedRed = 128;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'y') { //yellow
      storedRed = 255;
      storedGreen = 182;
      storedBlue = 0;
    } else if (input == 'l') { //AEMLIGHT
      storedRed = 60;
      storedGreen = 112;
      storedBlue = 203;
    } else if (input == 'd') { //AEMDARK
      storedRed = 16;
      storedGreen = 84;
      storedBlue = 162;
    } else if (input == 'r') {//red
      storedRed = 255;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == 'b') { //blue
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'g') { //green
      storedRed = 0; 
      storedGreen = 255;
      storedBlue = 0;
    } else if (input == 'o') { //orange
      storedRed = 255;
      storedGreen = 65;
      storedBlue = 0;
    } else if (input == '0') { // black
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 0;
    }
  }

  int red;
  int green;
  int blue;
  int SubtractValR;
  int SubtractValG;
  int SubtractValB;

  for (int i=0; i < NUMPIXELS; i++) {
    SubtractValR = map(((i+add)%DarkPix),0,(DarkPix),0,storedRed);
    SubtractValG = map(((i+add)%DarkPix),0,(DarkPix),0,storedGreen);
    SubtractValB = map(((i+add)%DarkPix),0,(DarkPix),0,storedBlue);
    red = round(storedRed-SubtractValR);
    green = round(storedGreen-SubtractValG);
    blue = round(storedBlue-SubtractValB);
    pixels.setPixelColor(i, pixels.Color(red,green,blue));
  }
    // This sends the updated pixel color to the hardware.
  pixels.show();
    
  
}

