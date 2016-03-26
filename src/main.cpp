//We always have to include the library
#include "LedControl.h"
#include "Delay.h"

#define	MATRIX_TOP	1
#define	MATRIX_BOTTOM	0

// Boundaries are 260/330/400
#define ACC_THRESHOLD_LOW 300
#define ACC_THRESHOLD_HIGH 360

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn
 pin 11 is connected to the CLK
 pin 10 is connected to LOAD
 We have only a single MAX72XX.
 */
LedControl lc=LedControl(2,4,3,2);
NonBlockDelay d;

void printmatrix() {
  Serial.println(" 0123-4567 ");
  for (int y = 0; y<8; y++) {
    if (y == 4) {
      Serial.println("|----|----|");
    }
    Serial.print(y);
    for (int x = 0; x<8; x++) {
      if (x == 4) {
        Serial.print("|");
      }
      Serial.print(lc.getXY(0,x,y) ? "X" :" ");
    }
    Serial.println("|");
  }
  Serial.println("-----------");
}

coord getDown(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y+1;
  return xy;
}

coord getLeft(int x, int y) {
  coord xy;
  xy.x = x-1;
  xy.y = y;
  return xy;
}

coord getRight(int x, int y) {
  coord xy;
  xy.x = x;
  xy.y = y+1;
  return xy;
}

bool canGoLeft(int addr, int x, int y) {
  if (x == 0) return false; // not available
  return !lc.getXY(addr, getLeft(x, y)); // you can go there if this is empty
}

bool canGoRight(int addr, int x, int y) {
  if (y == 7) return false; // not available
  return !lc.getXY(addr, getRight(x, y)); // you can go there if this is empty
}

bool canGoDown(int addr, int x, int y) {
  if (y == 7) return false; // not available
  if (x == 0) return false; // not available
  if (!canGoLeft(addr, x, y)) return false;
  if (!canGoRight(addr, x, y)) return false;
  return !lc.getXY(addr, getDown(x, y)); // you can go there if this is empty
}

void goDown(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getDown(x,y), true);
}

void goLeft(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getLeft(x,y), true);
}

void goRight(int addr, int x, int y) {
  lc.setXY(addr, x, y, false);
  lc.setXY(addr, getRight(x,y), true);
}


bool move_particle(int addr, int x, int y) {
    if (!lc.getXY(addr,x,y)) {
        return false;
    }

    bool can_GoLeft = canGoLeft(addr, x, y);
    bool can_GoRight = canGoRight(addr, x, y);

    if (!can_GoLeft && !can_GoRight) {
      return false; // we're stuck
    }

    bool can_GoDown = canGoDown(addr, x, y);

    if (can_GoDown) {
      goDown(addr, x, y);
    } else if (can_GoLeft&& !can_GoRight) {
      goLeft(addr, x, y);
    } else if (can_GoRight && !can_GoLeft) {
      goRight(addr, x, y);
    } else if (random(2) == 1) { // we can go left and right, but not down
      goLeft(addr, x, y);
    } else {
      goRight(addr, x, y);
    }
    return true;
}

void fill(int addr, int maxcount) {
  int n = 8;
  byte x,y;
  int count = 0;
  for (byte slice = 0; slice < 2*n-1; ++slice) {
    byte z = slice<n ? 0 : slice-n + 1;
    for (byte j = z; j <= slice-z; ++j) {
      y = 7-j;
      x = (slice-j);
      if (++count <= maxcount) {
        lc.setXY(addr, x, y, true);
        //lc.setXY(addr, x, y, random(3) == 1);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  // return;

  // VCC for the accelerometer
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);



    randomSeed(analogRead(0));



    // reset displays
    for (byte i=0; i<2; i++) {
      lc.shutdown(i,false);
      lc.setIntensity(i,0);
      lc.clearDisplay(i);
    }

    // fill top part
    fill(MATRIX_TOP, 60);
}

int _analogRead(int pin, int samples=10, int d=100) {
  delay(d);
  int result;
  result = analogRead(pin); // discard first reading
  for(int i = 0; i < samples; i++) { // Average 10 readings for accurate reading
    delay(d);
    result += analogRead(pin);
  }
  result = round(result/samples); // average

  return result;
}

int getGravity() {
  int x = analogRead(2);
  int y = analogRead(1);

  if (x < ACC_THRESHOLD_LOW) {
    return 0;
  }
  if (y < ACC_THRESHOLD_LOW) {
    return 270;
    //return 90;
  }
  if (x > ACC_THRESHOLD_HIGH) {
    return 180;
  }
  if (y >ACC_THRESHOLD_HIGH) {
    return 90;
    // return 270;
  }
}

void loop() {

  int gravity = getGravity();
  lc.setRotation((90 + gravity) % 360);

  Serial.println(getGravity());

  int n = 8;
  byte x,y;
  bool direction;

  for (byte slice = 0; slice < 2*n-1; ++slice) {
      direction = (random(2) == 1); // randomize if we scan from left to right or from right to left, so the grain doesn't always fall the same direction
      byte z = slice<n ? 0 : slice-n + 1;
      for (byte j = z; j <= slice-z; ++j) {
        y = direction ? (7-j) : (7-(slice-j));
        x = direction ? (slice-j) : j;
          // for (byte d=0; d<2; d++) { lc.invertXY(0, x, y); delay(50); }
          move_particle(0, x, y);
          move_particle(1, x, y);
      }
  }
  if (d.Timeout()) {
    if (gravity == 0 || gravity == 180) {
      if ((lc.getRawXY(MATRIX_TOP, 0, 0) && !lc.getRawXY(MATRIX_BOTTOM, 7, 7)) ||
          (!lc.getRawXY(MATRIX_TOP, 0, 0) && lc.getRawXY(MATRIX_BOTTOM, 7, 7))
      ) {
        // for (byte d=0; d<8; d++) { lc.invertXY(0, 0, 7); delay(50); }
        lc.invertRawXY(MATRIX_TOP, 0, 0);
        lc.invertRawXY(MATRIX_BOTTOM, 7, 7);
      }
    }
    d.Delay(1*1000);
  }
  delay(200);
}
