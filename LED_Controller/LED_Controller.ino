#include <Adafruit_NeoPixel.h>

// input pin Neopixel is attached to
#define PIN D1
#define PIN2 D5

#define NUMPIXELS 58  // number of neopixels in strip

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);


int shift = 0;

int SetWidth = 12;

int storedRed = 60;
int storedGreen = 112;
int storedBlue = 203;

int speed = 10;

bool loopq = false;

void setup() {
  // Initialize the NeoPixel library.
  pixels.begin();

  Serial.begin(115200);

  pinMode(D1, OUTPUT);
  pinMode(D5, OUTPUT);

  turnOn(SetWidth, 60, 112, 203);
  speed = 45;
}

int modulo(int a, int n) {
  int mod = a % n;
  if (mod < 0) {
    mod += n;
  }
  return mod;
}

void loop() {
  getColor();
  if (storedRed == 0 && storedGreen == 0 && storedBlue == 0 && !loopq) {
    shift = 0;
  } else if (storedRed == 0 && storedGreen == 0 && storedBlue == 0 && loopq) {
    turnOff();
  } else if (!loopq) {
    turnOn(SetWidth, storedRed, storedGreen, storedBlue);
  } else if (loopq) {
    setColor(shift, SetWidth);
    delay(speed);
    shift = modulo(shift - 1, SetWidth);
  }
}


void getColor() {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 'p') {  //purple
      storedRed = 128;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'y') {  //yellow
      storedRed = 255;
      storedGreen = 182;
      storedBlue = 0;
    } else if (input == 'l') {  //AEMLIGHT
      storedRed = 60;
      storedGreen = 112;
      storedBlue = 203;
    } else if (input == 'd') {  //AEMDARK
      storedRed = 16;
      storedGreen = 84;
      storedBlue = 162;
    } else if (input == 'r') {  //red
      storedRed = 255;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == 'b') {  //blue
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'g') {  //green
      storedRed = 0;
      storedGreen = 255;
      storedBlue = 0;
    } else if (input == 'o') {  //orange
      storedRed = 255;
      storedGreen = 65;
      storedBlue = 0;
    } else if (input == '0') {  // black
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == '1') { // regular
      speed = 45;
    } else if (input == '2') { // fast
      speed = 10;
    }
  }
}

void setColor(int shift, int SetWidth) {

  int red;
  int green;
  int blue;
  int SubtractValR;
  int SubtractValG;
  int SubtractValB;

  for (int i = 0; i < NUMPIXELS; i++) {
    SubtractValR = map(modulo(i + shift, SetWidth), 0, (SetWidth), 0, storedRed);
    SubtractValG = map(modulo(i + shift, SetWidth), 0, (SetWidth), 0, storedGreen);
    SubtractValB = map(modulo(i + shift, SetWidth), 0, (SetWidth), 0, storedBlue);
    red = round(storedRed - SubtractValR);
    green = round(storedGreen - SubtractValG);
    blue = round(storedBlue - SubtractValB);
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels2.setPixelColor(i, pixels2.Color(red, green, blue));
  }
  // This sends the updated pixel color to the hardware.
  pixels.show();
  pixels2.show();
}

void turnOn(int SetWidth, int InRed, int InGreen, int InBlue) {
  int red;
  int green;
  int blue;
  int SubtractValR;
  int SubtractValG;
  int SubtractValB;

  for (int i = 0; i < NUMPIXELS; i++) {
    SubtractValR = map(modulo(i, SetWidth), 0, (SetWidth), 0, InRed);
    SubtractValG = map(modulo(i, SetWidth), 0, (SetWidth), 0, InGreen);
    SubtractValB = map(modulo(i, SetWidth), 0, (SetWidth), 0, InBlue);
    red = round(InRed - SubtractValR);
    green = round(InGreen - SubtractValG);
    blue = round(InBlue - SubtractValB);
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels2.setPixelColor(i, pixels2.Color(red, green, blue));
    pixels.show();
    pixels2.show();
    delay(10);
  }
  loopq = true;
}

void turnOff() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0,0,0));
    pixels2.setPixelColor(i, pixels2.Color(0, 0, 0));
    pixels.show();
    pixels2.show();
    delay(10);
  }
  loopq = false;
}

