#include <Adafruit_NeoPixel.h>

#define PIN D1	 // output pin Neopixel is attached to
#define PIN2 D5

#define NUMPIXELS      58 // number of neopixels in strip

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);

int shift = 0;

int storedRed = 255;
int storedGreen = 0;
int storedBlue = 0;

int storedHue = 0;
int storedSat = 100;
int storedVal = 100;

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
  pinMode(D5, OUTPUT);
}

void loop() {
  setColor();
  delay(100);
  shift = shift + 1;
  if (shift > 100) {
    shift = 0;
  }
}
    

void setColor() {
  detectInput();
  //int color[3] = {255,0,0};//red
  //int color[3] = {0,0,255};//blue
  //int color[3] = {255,255,0};//yellow
  //int color[3] = {255,0,255};//purple
  //int color[3] = {0,255,0};//green bc Zack

  int red;
  int green;
  int blue;

  for (int i=0; i < NUMPIXELS; i++) {
    hsv_to_rgb(storedHue, storedSat, (storedVal-((shift+i)%100)));
    red = storedRed;
    green = storedGreen;
    blue = storedBlue;
    pixels.setPixelColor(i, pixels.Color(red,green,blue));
    pixels2.setPixelColor(i, pixels2.Color(red,green,blue));
    //Serial.print(red);
    //Serial.print(green);
    //Serial.println(blue);
    //Serial.println("RED:");
    //Serial.println((abs(red/2)+(red/2)));
    //Serial.println((abs(green/2)+(green/2)));
    //Serial.println((abs(blue/2)+(blue/2)));

    // This sends the updated pixel color to the hardware.
    pixels.show();
    pixels2.show();

    // Delay for a period of time (in milliseconds).
    //delay(delayval);
  }
  
}

void detectInput() {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 112) { //purple
      storedHue = 285;
      storedSat = 100;
      storedVal = 100;
    } else if (input == 121) { //yellow
      storedHue = 75;
      storedSat = 100;
      storedVal = 100;
    } else if (input == 108) { //AEMLIGHT
      storedHue = 218;
      storedSat = 70;
      storedVal = 80;
    } else if (input == 100) { //AEMDARK
      storedHue = 212;
      storedSat = 90;
      storedVal = 64;
    } else if (input == 114) {//red
      storedHue = 0;
      storedSat = 100;
      storedVal = 100;
    } else if (input == 98) { //blue
       storedHue = 240;
       storedSat = 100;
       storedVal =100;
    } else if (input == 103) { //green
      storedHue = 120; 
      storedSat = 100;
      storedVal = 100;
    } else if (input == 111) { //orange
      storedHue = 15;
      storedSat = 100;
      storedVal = 100;
    } else if (input == 48) { //off
      storedHue = 0;
      storedSat = 0;
      storedVal = 0;
    } else if (input == 119) { //white
      storedHue = 0;
      storedSat = 0;
      storedVal = 100;
    }
  }
}


void hsv_to_rgb(int hue, int sat, int val) {
  sat = sat/100;
  val = val/100;
  double C = val*sat;
  double X = C*(1-abs((hue/60)%2-1));
  double m = val-C;
  double Rp, Gp, Bp;
  if ((0 <= hue && hue < 60) || hue == 360) {
    Rp, Gp, Bp = C, X, 0;
  } else if (60 <= hue && hue < 120) {
    Rp, Gp, Bp = X, C, 0;
  } else if (120 <= hue && hue < 180) {
    Rp, Gp, Bp = 0, C, X;
  } else if (180 <= hue && hue < 240) {
    Rp, Gp, Bp = 0, X, C;
  } else if (240 <= hue && hue < 300) {
    Rp, Gp, Bp = X, 0, C;
  } else if (300 <= hue && hue < 360) {
    Rp, Gp, Bp = C, 0, X;
  }

  double R = round(((Rp+m)*255));
  double G = round(((Gp+m)*255));
  double B = round(((Bp+m)*255));

  storedRed = R;
  storedGreen = G;
  storedBlue = B;

  Serial.println(storedVal);
}