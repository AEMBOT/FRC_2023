// Declare that we are using the neopixel library
#include <Adafruit_NeoPixel.h>

// Input pin Neopixel is attached to (D1 and D5 for the microcontroller we are using)
#define PIN D1
#define PIN2 D5

#define NUMPIXELS 58  // Number of neopixels in the strip

// Makes an LED strip called 'pixels' and 'pixels2' which are used later in changeing the color
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);

// Tells the LEDs when to shift, should never be bigger then 'setWidth'
int shift = 0;

// The width of each light-to-dark set
int setWidth = 12;

// The stored RGB values, default is currently AEMLIGHT
int storedRed = 60;
int storedGreen = 112;
int storedBlue = 203;

// How fast the LEDs shift
int speed = 10;

// Tells the void loop() whether to shift or not, depending if there is a color, or the LEDs are off
bool loopq = false;

// Setup function to run once
void setup() {
  // Initialize the NeoPixel library
  pixels.begin();

  // Initialize serial communications
  Serial.begin(115200);

  // Set the pinmodes on the microcontroller
  pinMode(D1, OUTPUT);
  pinMode(D5, OUTPUT);

  // Turns the LEDs on with a cool function in the stored color
  turnOn(setWidth, storedRed, storedGreen, storedBlue);

  // Sets the shifting speed to a slower speed
  speed = 45;
}

// Mod function with no negative numbers
int modulo(int a, int n) {
  int mod = a % n;
  if (mod < 0) {
    mod += n;
  }
  return mod;
}

// loop function to repeat constantly
void loop() {
  // Function to read the serial and set the color
  getColor();

  // Checks if there is no color, if there is not, it resets the shift variable back to 0
  if (storedRed == 0 && storedGreen == 0 && storedBlue == 0 && !loopq) {
    shift = 0;

  // Checks if the LEDs were just turned off, and if so, runs the turn off function
  } else if (storedRed == 0 && storedGreen == 0 && storedBlue == 0 && loopq) {
    turnOff();

  // Checks if the LEDs were just turned on, and if so, runs the turn on function
  } else if (!loopq) {
    turnOn(setWidth, storedRed, storedGreen, storedBlue);

  // Checks if the LEDs should just be shifting like normal, and shifts them
  } else if (loopq) {
    setColor(shift, setWidth);

    // Delay for [speed] ms 
    delay(speed);

    // Shift the LEDs and make sure if is always below setWidth
    shift = modulo(shift - 1, setWidth);
  }
}

// Function to read the serial and set the color
void getColor() {
  
  // Checks if there is a serial input
  if (Serial.available()) {

    // Reads the input
    int input = Serial.read();
    if (input == 'p') {// Sets the color to purple
      storedRed = 128;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'y') {// Sets the color to yellow
      storedRed = 255;
      storedGreen = 182;
      storedBlue = 0;
    } else if (input == 'l') {// Sets the color to AEMLIGHT
      storedRed = 60;
      storedGreen = 112;
      storedBlue = 203;
    } else if (input == 'd') {// Sets the color to AEMDARK
      storedRed = 16;
      storedGreen = 84;
      storedBlue = 162;
    } else if (input == 'r') {// Sets the color to red
      storedRed = 255;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == 'b') {// Sets the color toblue
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 255;
    } else if (input == 'g') {// Sets the color to green
      storedRed = 0;
      storedGreen = 255;
      storedBlue = 0;
    } else if (input == 'o') {// Sets the color to orange
      storedRed = 255;
      storedGreen = 65;
      storedBlue = 0;
    } else if (input == '0') {// Sets the color to black
      storedRed = 0;
      storedGreen = 0;
      storedBlue = 0;
    } else if (input == '1') {// Regular speed (Auto & Teleop)
      speed = 45;
    } else if (input == '2') { // Fast speed (Endgame)
      speed = 10;
    }
  }
}

// Set the color (shifting support)
void setColor(int shift, int setWidth) {

  // Declare the variables that stores the final red, green, and blue values
  int red;
  int green;
  int blue;

  // Declare the variables that store the amount subtracted from the stored red, green, and blue values
  int SubtractValR;
  int SubtractValG;
  int SubtractValB;

  // Loops through all of the pixels on the strip, (NUMPIXELS variable declared above)
  for (int i = 0; i < NUMPIXELS; i++) {

    // Calculate the SubtractVal values
    SubtractValR = map(modulo(i + shift, setWidth), 0, (setWidth), 0, storedRed);
    SubtractValG = map(modulo(i + shift, setWidth), 0, (setWidth), 0, storedGreen);
    SubtractValB = map(modulo(i + shift, setWidth), 0, (setWidth), 0, storedBlue);

    // Set red, green, or and blue to the corresponding stored value - the corresponding subtract value
    red = round(storedRed - SubtractValR);
    green = round(storedGreen - SubtractValG);
    blue = round(storedBlue - SubtractValB);

    // Set the pixel colors on both the strips
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels2.setPixelColor(i, pixels2.Color(red, green, blue));
  }
  // Update the pixels so that the color set above shows
  pixels.show();
  pixels2.show();
}

// Turn on function
void turnOn(int setWidth, int InRed, int InGreen, int InBlue) {

  // Declare the variables that stores the final red, green, and blue values
  int red;
  int green;
  int blue;

  // Declare the variables that store the amount subtracted from the stored red, green, and blue values
  int SubtractValR;
  int SubtractValG;
  int SubtractValB;

  // Loops through all of the pixels on the strip, (NUMPIXELS variable declared above)
  for (int i = 0; i < NUMPIXELS; i++) {

    // Calculate the SubtractVal values
    SubtractValR = map(modulo(i, setWidth), 0, (setWidth), 0, InRed);
    SubtractValG = map(modulo(i, setWidth), 0, (setWidth), 0, InGreen);
    SubtractValB = map(modulo(i, setWidth), 0, (setWidth), 0, InBlue);

    // Set red, green, or and blue to the corresponding stored value - the corresponding subtract value
    red = round(InRed - SubtractValR);
    green = round(InGreen - SubtractValG);
    blue = round(InBlue - SubtractValB);

    // Set the pixel colors on both the strips
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels2.setPixelColor(i, pixels2.Color(red, green, blue));

    // Update the pixels so the color set above shows (inside the loop this time so there is a slight delay between LEDs updating)
    pixels.show();
    pixels2.show();

    // Delay for 10 ms or 1/100 of a second
    delay(10);
  }

  // Tell the loop in loop() to loop
  loopq = true;
}

// Turn off function
void turnOff() {

  // Loops through all of the pixels on the strip, (NUMPIXELS variable declared above)
  for (int i = 0; i < NUMPIXELS; i++) {

    // Sets the pixel color to 0
    pixels.setPixelColor(i, pixels.Color(0,0,0));
    pixels2.setPixelColor(i, pixels2.Color(0, 0, 0));

    // Update the pixels so the color set above shows (inside the loop this time so there is a slight delay between LEDs updating)
    pixels.show();
    pixels2.show();

    // Delay for 10 ms or 1/100 of a second
    delay(10);
  }

  // Tell the loop in loop() to not loop
  loopq = false;
}