#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

#define TRIGGER_PIN_0 3
#define ECHO_PIN_0 4
#define TRIGGER_PIN_1 6
#define ECHO_PIN_1 5
#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 10

#define NUM_LEDS 120

Adafruit_NeoPixel strip(NUM_LEDS, 13, NEO_GRB + NEO_KHZ800);

NewPing sonar[3] = {
  NewPing(TRIGGER_PIN_0, ECHO_PIN_0),
  NewPing(TRIGGER_PIN_1, ECHO_PIN_1),
  NewPing(TRIGGER_PIN_2, ECHO_PIN_2)
};

unsigned long lastDistanceUpdateTime = 0;
unsigned long distanceUpdateInterval = 1000; 

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < 120; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  delay(wait);
}

void rainbow1(int wait) {
  int firstPixelHue = 0;
  for (int a = 0; a < 30; a++) {
    for (int b = 0; b < 3; b++) {
      strip.clear();
      for (int c = b; c < strip.numPixels(); c += 3) {
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue));
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);
      firstPixelHue += 65536 / 90;
    }
  }
}

void rainbow2(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 4096; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDistanceUpdateTime >= distanceUpdateInterval) {
    lastDistanceUpdateTime = currentMillis;
    float distance = Distance();
    Serial.println(distance);
  }

  // Đọc dữ liệu từ cổng Serial
  while (Serial.available() > 0) {
    char color = Serial.read();
    HandleSerialInput(color);
  }
}

float Distance() {
  unsigned long duration;
  int distance247;
  digitalWrite(TRIGGER_PIN_0, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_0, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN_0, LOW);
  duration = pulseIn(ECHO_PIN_0, HIGH);
  distance247 = int(duration / 2 / 29.412);
  return distance247;
}

void HandleSerialInput(char color) {
  switch (color) {
    case '1':
      colorWipe(strip.Color(255, 0, 0), 1);
      break;
    case '2':
      colorWipe(strip.Color(0, 255, 0), 1);
      break;
    case '3':
      colorWipe(strip.Color(0, 0, 255), 1);
      break;
    case '4':
      colorWipe(strip.Color(255, 0, 255), 1);
      break;
    case '5':
      colorWipe(strip.Color(255, 255, 0), 1);
      break;
    case '6':
      colorWipe(strip.Color(255, 255, 255), 1);
      break;
    case '7':
      rainbow1(1);
      break;
    case '8':
      rainbow2(1);
      break;
    default:
      colorWipe(strip.Color(0, 0, 0), 1);
      break;
  }
}
