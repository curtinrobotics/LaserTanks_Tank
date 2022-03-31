//TODO ir
//     evrything else

#include <SoftwareSerial.h>
#include <IRremote.h>
#include <DRV8835MotorShield.h>

#define TANK_NUM 1
#define TANK_CODE_1 2701
#define TANK_CODE_2 2702
#define TANK_CODE_3 2703
#define TANK_CODE_4 2704

#if TANK_NUM == 1
  #define TANK_CODE TANK_CODE_1
#endif

#if TANK_NUM == 2
  #define TANK_CODE TANK_CODE_2
#endif

#if TANK_NUM == 3
  #define TANK_CODE TANK_CODE_3
#endif

#if TANK_NUM == 4
  #define TANK_CODE TANK_CODE_4
#endif

#define SOUND_SERIAL_PIN A0

#define SOUND_BUSY_PIN A1

#define IR_RECEIVE_PIN A4

#define LASER_PIN A5

//#define LEFTM_INVERTED 1
//#define RIGHTM_INVERTED 1
// uncomment the relevant #define to invert the motor phase

#define LEFTM_PHASE_PIN 7
#define RIGHTM_PHASE_PIN 8

#define TURRET_ZERO_VALUE 2900//3900
// turret zero value is the OCR3A value for which no rotation occurs. Upper and lower limits will be this value +-1000
// if it is difficult to make it stay still, we could shut off the pin when it is stationary.

#define BARREL_JOYSTICK_NEUTRAL_VAL 512
#define BARREL_CONTROL_DEADBAND 16
int BARREL_MINVAL = map(-10,-90,90,1050,4550);
int BARREL_MAXVAL = map(30,-90,90,1050,4550);
int BARREL_NEUTRALPOS = map(0,-90,90,1050,4550);
//#define BARREL_INVERT_DIR 1

#define TANK_SWITCH_1 A3
#define TANK_SWITCH_2 A2

#define RGB_RED_PIN 3
#define RGB_GREEN_PIN 6
#define RGB_BLUE_PIN 11

SoftwareSerial soundSerial(NULL, SOUND_SERIAL_PIN);

IRrecv irrecv(IR_RECEIVE_PIN);

IRsend irsend;

DRV8835MotorShield motors;

void ledColour(int, int, int);

void moveTurret(int);
void moveBarrel(int);
void motorControl(int, int);

void initializeTimers();

void playSound(int);

void shoot();

byte soundComand[6];
byte arg1;
byte arg2;

int hit = 0;
int powerUpUse = 0;

int toGM[2] = {hit, powerUpUse};

int left;
int right;
int turret;
int barrel;
int red;
int green;
int blue;
int fire = 0;
int sound;
byte fromCont[2] = {0,0};

int soundBusy;

unsigned long lastShot = millis();

byte first;
byte second;
byte receve[2];

int shotBy = 0;

decode_results IRresults;

void prntBits(byte b, char lnEnd='');

void setup() {
  initializeTimers();

  //usb serial
  Serial.begin(9600);

  //wifi serial
  Serial1.begin(9600);

  //sounds
  soundSerial.begin(9600);
  pinMode(SOUND_BUSY_PIN, INPUT_PULLUP);

  //laser
  pinMode(LASER_PIN, OUTPUT);

  irrecv.enableIRIn();

  pinMode(A3, OUTPUT);
}

void loop() {
  //reset sound
  sound = 0;

  //reset laser
  if (millis()-lastShot>150) {
    digitalWrite(LASER_PIN, LOW);
  }

  //test if sound is busy
  soundBusy = !digitalRead(SOUND_BUSY_PIN);

  //receve info from controller

  if (Serial1.available()) {
    /*first = Serial1.read();
    prntBits(first, true);*/
    Serial1.readBytes(receve,2);
    first = receve[0];
    second = receve[1];
    prntBits(first, '\t');
    //Serial.write("\t");
    prntBits(second, '\n');
  }

  //decode binary values
  //byte first = B11101110;
  
  byte leftB = (first)&(B11100000);
  int left = leftB >> 5;
  
  byte leftD = (first)&(B00010000);
  leftD = leftD >> 4;
    
  if (leftD) {
    left *= -1;
  }

  byte rightB = (first)&(B00001110);
  int right = rightB >> 1;
  
  byte rightD = (first)&(B00000001);
    
  if (rightD) {
    right *= -1;
  }

  byte turretB = (second)&(B11100000);
  int turret = turretB >> 5;
  
  byte turretD = (second)&(B00010000);
  turretD = turretD >> 4;
    
  if (turretD) {
    turret *= -1;
  }
  
  byte fire = (second)&(B00001000);

  /*Serial.print(left);
  Serial.write(",");
  Serial.print(right);
  Serial.write(",");
  Serial.print(turret);
  Serial.write(",");
  Serial.print(barrel);
  Serial.write(",");
  Serial.print(red);
  Serial.write(",");
  Serial.print(green);
  Serial.write(",");
  Serial.print(blue);
  Serial.write(",");
  Serial.println(fire);*/

  /*Serial.print(left);
  Serial.print(",");
  Serial.print(right);*/

  /*Serial.write("\t");*/
  //Serial.print(left);
  //Serial.print(",");
  //Serial.println(right);
  
  //turret and barrel movment
  moveTurret(turret);
  moveBarrel(barrel);

  //drive moter control
  motorControl(left, right);

  //rgb led colour
  //ledColour(red,green,blue);

  //play sounds
  //playSound(sound);

  //fire the gun
  if (fire) {
    shoot();
  }

  if (irrecv.decode(&IRresults)) {
    if (IRresults.value != TANK_CODE) {
      if (IRresults.value == TANK_CODE_1) {
        shotBy = 1;
      } else if (IRresults.value == TANK_CODE_2) {
        shotBy = 2;
      } else if (IRresults.value == TANK_CODE_3) {
        shotBy = 3;
      } else if (IRresults.value == TANK_CODE_4) {
        shotBy = 4;
      }
      if (shotBy != 0) {
        Serial.print("You were shot by Tank ");
        Serial.println(shotBy);
        digitalWrite(A3, HIGH);
        delay(15);
        digitalWrite(A3, LOW);
        shotBy = 0;
      }
    }
    irrecv.resume(); // Receive the next value
  }

  ledColour(5,5,5);

  //test if hit
  //hit

  //send hits and use powerup to game master
  
  //delay(15);
} 

//#define TURRET_PIN 5
//#define BARREL_PIN 4
// barrel servo is interrupt-driven
// what does 'turning off' the servo control signal (pin) do?

// functions: ledColour, moveTurret, moveBarrel, motors

// 0-100%
void ledColour(int rDuty, int gDuty, int bDuty)
{
  // Red pin is PB5
  // Grn pin is PB6
  // Blu pin is PB7
  rDuty = map(rDuty,0,100,0,255);
  gDuty = map(gDuty,0,100,0,255);
  bDuty = map(bDuty,0,100,0,255);
  
  /*analogWrite(RGB_RED_PIN, rDuty);
  analogWrite(RGB_GREEN_PIN, gDuty);
  analogWrite(RGB_BLUE_PIN, bDuty);*/
}

void moveTurret(int tSpeed)
{
  tSpeed = map(tSpeed, -7, 7, -1000, 1000);
  OCR3B = TURRET_ZERO_VALUE + tSpeed;
}

void moveBarrel(int bInput)
{
  static int bPosition = BARREL_NEUTRALPOS;
  int change = map(bInput, 0, 1023, -25, 25);
  if ((change > 5) || (change < -5))
  {
    bPosition += change;
    bPosition = constrain(bPosition, BARREL_MINVAL, BARREL_MAXVAL);
  }
  OCR3A = bPosition;
}

// so... inputs from -7 to 7 (just 'cause)
void motorControl(int l_Speed, int r_Speed)
{
  left = map(l_Speed,-7,7,-400,400);
  right = map(r_Speed,-7,7,-400,400);

  //Serial.print(left);
  //Serial.print(",");
  //Serial.println(right);
  
  motors.setM1Speed(left);
  motors.setM2Speed(right);
}

/* This code is to set up the timers and pin directions for all PWM pins on the Arduino Micro.
 * The output compare registers should really only be accessed from within other functions that 
 * provide better abstraction, eg "void moveBarrel(int angle)" rather than writing the raw pulse length
 * directly to OCR3B.
 */

/* Timer assignments:
 * Timer 3 - Interrupt mode, interrupts control servo pins  --- expect   50  Hz
 */
 
/* PIN ASSIGNMENTS - Arduino Pin Number - (access register) [range]
 * Servo 1: D5 (OCR3A) [0-39999]; half-microSeconds
 * Servo 2: D4 (OCR3B) [0-39999]; half-microSeconds; ***software (interrupt) -based
 */
 
void initializeTimers()
{
  /* Rev 1 - code adjusted for unscaled system clock (16MHz) */
   
  /* enable global interrupts */
  sei();
   
  /* Initialize Timer 3 for a 20ms period, with direct servo control output on OC3A and indirect servo
   * control via interrupt generated by OCR3B compare match */
  OCR3A = 0x00; /* IMPORTANT - initialize OCRs to 0 to prevent weirdness */
  OCR3B = 0x00;
  ICR3 = 39999; /* this sets the counter period to 20ms */
   
  TCCR3A = 0x00; /* clear all bits */   
  TCCR3B = 0x00; /* clear all bits */   
  TCCR3C = 0x00; /* clear all bits */   
  TIMSK3 = 0x00; /* clear all bits */
   
  TCCR3A |= (1<<COM3A1) + (1<<WGM31); /* clears OC3A (dig. pin 5) on compare match */
    DDRC |= _BV(6); /* Set dig. pin 5 (PC6) to output mode */

  TCCR3B |= (1<<WGM33) + (1<<WGM32) + (1<<CS31); /* FPWM mode with period defined by ICR3; prescaler = 8 */

  TIMSK3 |= (1<<ICIE3) + (1<<OCIE3B); /* enable interrupts for count==ICR3 and count==OCR3B */
    DDRD |= _BV(4); /* Set dig. pin 4 (PD4) to output mode*/
  /* End Timer 3 */

 /* End Timer Initializations */   
}
 
ISR(TIMER3_CAPT_vect)
{
  /* FOR SERVO
   * This interrupt is triggered when Timer3 reaches the value in ICR3 (39,999 in this case).
   * We set the output pin (Dig. pin 4) high at the end (aka beginning) of each timer cycle. */
  PORTD |= _BV(4); /* Set Dig. pin 4 (PD4) HIGH */
}
 
ISR(TIMER3_COMPB_vect)
{
  /* FOR SERVO
   * This interrupt is triggered when Timer3 reaches the value in OCR3B.
   * This time we clear the output pin (Dig. pin 4) at the end of each pulse. */
  PORTD &= ~(_BV(4)); /* Set Dig. pin 4 (PD4) LOW */
}

void playSound(int sound) {
  if (sound && soundBusy) {
    //Serial.println(sound);
    arg1 = (sound>>8) & 0xFF;
    arg2 = sound & (byte)0xFF;
  
    soundComand[0] = (byte)0x7E;
    soundComand[1] = 4;
    soundComand[2] = (byte)0x03;
    soundComand[3] = arg1;
    soundComand[4] = arg2;
    soundComand[5] = (byte)0xEF;
    
    soundSerial.write(soundComand,6);
  }
}

void shoot() {
  if ((millis()-lastShot)>1000) {
    digitalWrite(LASER_PIN, HIGH);
    irsend.sendSony(TANK_CODE, 12);
    irrecv.enableIRIn();
    playSound(1);
    lastShot = millis();
  }
}

void prntBits(byte b, char lnEnd='')
{
  for(int i = 7; i >= 0; i--)
    Serial.print(bitRead(b,i));
  Serial.write(lnEnd);
}
