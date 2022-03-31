int a = 1;

int x = 1023;
int y = 0;

int xd = -1;
int yd = -1;

int inThrottle;
int inSteering;

int left;
int right;

#define xPin 0
#define yPin 1

#define xDeadZone 25
#define yDeadZone 25
#define joyMax 1000

long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  x = analogRead(xPin);
  y = analogRead(yPin);

  x = map(x,0,1023,-1023,1023);
  y = map(y,0,1023,1023,-1023);

  if (x<=xDeadZone && x>=-xDeadZone) {
    x = 0;
  }

  if (y<=yDeadZone && y>=-yDeadZone) {
    y = 0;
  }

  x = constrain(x, -joyMax, joyMax);
  y = constrain(y, -joyMax, joyMax);
  
  left = constrain(x - y,-990,990);
  right = constrain(x + y,-990,990);

  //Serial.print(a);
  //Serial.write("\t");
  Serial.print(left);
  Serial.write("\t");
  Serial.print(right);
  Serial.write("\t");
  Serial.print(x);
  Serial.write("\t");
  Serial.println(y);

  /*x = x + xd;
  y = y + yd;
  a++;*/
}

/*void CalculateTankDrive(float x, float y)
{
  // first Compute the angle in deg
  // First hypotenuse
  float z = sqrt(x * x + y * y);
  
  // angle in radians
  float rad = acos(abs(x) / z);

  // Cataer for NaN values
  if (isnan(rad)) {
    rad=0;
  }
  
  // and in degrees
  float angle = rad * 180 / PI;
  
  // Now angle indicates the measure of turn
  // Along a straight line, with an angle o, the turn co-efficient is same
  // this applies for angles between 0-90, with angle 0 the co-eff is -1
  // with angle 45, the co-efficient is 0 and with angle 90, it is 1

  float tcoeff = -1 + (angle / 90) * 2;
  float turn = tcoeff * abs(abs(y) - abs(x));
  turn = round(turn * 100) / 100;
  
  // And max of y or x is the movement
  float mov = max(abs(y), abs(x));
  
  // First and third quadrant
  if ((x >= 0 && y >= 0) || (x < 0 && y < 0)) {
    rawLeft = mov; rawRight = turn;
  } else {
    rawRight = mov; rawLeft = turn;
  }

  // Reverse polarity
  /*if (y < 0) {
    rawLeft = 0 - rawLeft; rawRight = 0 - rawRight;
  }*/

  // Update the values
  /*RawLeft = rawLeft;
  RawRight = rawRight;
  
  // Map the values onto the defined rang
  ValLeft = map(rawLeft, MinJoy, MaxJoy, MinValue, MaxValue);
  ValRight = map(rawRight, MinJoy, MaxJoy, MinValue, MaxValue);
  
  // Cater for inverse of direction if needed
  /*if (invXL) {
    RawLeft *= -1; ValLeft = MaxValue - ValLeft;
  }
  if (invXR) {
    RawRight *= -1; ValRight = MaxValue - ValRight;
  }*/
//}

