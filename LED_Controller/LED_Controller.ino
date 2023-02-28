#include <Adafruit_NeoPixel.h>

#define PIN D1	 // input pin Neopixel is attached to

#define NUMPIXELS      58 // number of neopixels in strip

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int add = 0;

int DarkPix = 6;

int storedRed = 60;
int storedGreen = 112;
int storedBlue = 203;

int doRainbow = 0;

void setup() {
  // Initialize the NeoPixel library.
  pixels.begin();
  
  Serial.begin(115200);
  
  //pinMode(3, INPUT);
  //pinMode(4, INPUT);
  //pinMode(5, INPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D1, OUTPUT);
}

void loop() {
  if (doRainbow == 0){
    setColor(add, DarkPix);
  } else if (doRainbow == 1) {
    rainbow(add);
  }
  delay(100);
  add = (add + 1) % DarkPix;
  }
    
  //delay(1000);
void rainbow(int add) {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 113) {
      doRainbow = 0;
    }
  }

  int red[3] = {255,0,0};
  int orange[3] = {255,65,0};
  int yellow[3] = {255,182,0};
  int green[3] = {0,255,0};
  int blue[3] = {0,0,255};
  int purple[3] = {128,0,255};
  int colors[6] = {red[3],orange[3],yellow[3],green[3],blue[3],purple[3]};
  for (int i=0; i < NUMPIXELS; i++) {
    int index = (i+add)%6;
    int selColor[3] = {colors[index]};    
    int red = selColor[0];
    int green = selColor[1];
    int blue = selColor[2];
    pixels.setPixelColor(i, pixels.Color(red,green,blue));
    //Serial.print(red);
    //Serial.print(green);
    //Serial.println(blue);
    //Serial.println("RED:");
    //Serial.println((abs(red/2)+(red/2)));
    //Serial.println((abs(green/2)+(green/2)));
    //Serial.println((abs(blue/2)+(blue/2)));

    // This sends the updated pixel color to the hardware.
    pixels.show();

    // Delay for a period of time (in milliseconds).
    //delay(delayval);
  }
}

void setColor(int add, int DarkPix) {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 112) { //purple
      storedRed = 128;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 121) { //yellow
      storedRed = 255;
      storedGreen = 182;
      storedBlue = 0;
    } else if (input == 108) { //AEMLIGHT
      storedRed = 60;
      storedGreen = 112;
      storedBlue = 203;
    } else if (input == 100) { //AEMDARK
      storedRed = 16;
      storedGreen = 84;
      storedBlue = 162;
    } else if (input == 114) {//red
      storedRed = 255;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == 98) { //blue
       storedRed = 0;
       storedGreen = 0;
       storedBlue = 255;
    } else if (input == 103) { //green
      storedRed = 0; 
      storedGreen = 255;
      storedBlue = 0;
    } else if (input == 111) { //orange
      storedRed = 255;
      storedGreen = 65;
      storedBlue = 0;
    } else if (input == 48) {
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == 113) {
      doRainbow = 1;
    }
  }
  //int color[3] = {255,0,0};//red
  //int color[3] = {0,0,255};//blue
  //int color[3] = {255,255,0};//yellow
  //int color[3] = {255,0,255};//purple
  //int color[3] = {0,255,0};//green bc Zack

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
    //Serial.print(red);
    //Serial.print(green);
    //Serial.println(blue);
    //Serial.println("RED:");
    //Serial.println((abs(red/2)+(red/2)));
    //Serial.println((abs(green/2)+(green/2)));
    //Serial.println((abs(blue/2)+(blue/2)));

    // This sends the updated pixel color to the hardware.
    pixels.show();

    // Delay for a period of time (in milliseconds).
    //delay(delayval);
  }
    
  
}

