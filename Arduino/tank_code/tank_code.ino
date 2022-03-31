//TODO ir
//     evrything else

#include <SoftwareSerial.h>

SoftwareSerial soundSerial(NULL, A0);

void ledColour(uint8_t, uint8_t, uint8_t);

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
byte receve[1];

#define LASER A5

#define IR 12

#define SOUND_BUSY_PIN A1

//#define LEFTM_INVERTED 1
//#define RIGHTM_INVERTED 1
// uncomment the relevant #define to invert the motor phase

#define LEFTM_PHASE_PIN 7
#define RIGHTM_PHASE_PIN 8

#define TURRET_ZERO_VALUE 3900//2900
// turret zero value is the OCR3A value for which no rotation occurs. Upper and lower limits will be this value +-1000
// if it is difficult to make it stay still, we could shut off the pin when it is stationary.

#define BARREL_JOYSTICK_NEUTRAL_VAL 512
#define BARREL_CONTROL_DEADBAND 16
int BARREL_MINVAL = map(-10,-90,90,1050,4550);
int BARREL_MAXVAL = map(30,-90,90,1050,4550);
int BARREL_NEUTRALPOS = map(0,-90,90,1050,4550);
//#define BARREL_INVERT_DIR 1

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
  pinMode(LASER, OUTPUT);

  //ir transmitter
  pinMode(IR, OUTPUT);
  digitalWrite(IR, HIGH);
}

void loop() {
  //reset sound
  sound = 0;

  //reset laser
  if (millis()-lastShot>150) {
    digitalWrite(LASER, LOW);
  }

  //reset ir
  digitalWrite(IR, HIGH);

  //test if sound is busy
  soundBusy = !digitalRead(SOUND_BUSY_PIN);

  //receve info from controller

  if (Serial1.available()) {
    Serial1.readBytes(receve,1);
    Serial.write(receve[0]);
    //Serial.write("\t");
  }

  first = receve[0];

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

  Serial.print(left);
  Serial.print(",");
  Serial.print(right);

  left = map(left,-7,7,-1023,1023);
  right = map(right,-7,7,-1023,1023);

  Serial.write("\t");
  Serial.print(left);
  Serial.print(",");
  Serial.println(right);
  
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

void ledColour(uint8_t rDuty, uint8_t gDuty, uint8_t bDuty)
{
  // Red pin is PB5
  // Grn pin is PB6
  // Blu pin is PB7
  
  if (rDuty) { // if rDuty != 0
    DDRB |= _BV(PB5); // enable output on red pin
    OCR1A = rDuty; // set pwm duty cycle
  } else {
    DDRB &= ~(_BV(PB5)); // disable output
  }
  
  if (gDuty) { // if gDuty != 0
    DDRB |= _BV(PB6); // enable output on grn pin
    OCR1B = gDuty; // set pwm duty cycle
  } else {
    DDRB &= ~(_BV(PB6));
  }
  
  if (bDuty) { // if bDuty != 0
    DDRB |= _BV(PB7); // enable output on blu pin
    OCR1C = bDuty; // set pwm duty cycle
  } else {
    DDRB &= ~(_BV(PB7)); // disable output
  }
}

void moveTurret(int tSpeed)
{
  tSpeed = map(tSpeed, 0, 1023, -1000, 1000);
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

// so... inputs from -1023 to 1023 (just 'cause)
void motorControl(int l_Speed, int r_Speed)
{
#ifdef LEFTM_INVERTED
  l_Speed = -l_Speed;
#endif
#ifdef RIGHTM_INVERTED
  r_Speed = -r_Speed;
#endif 

  if (l_Speed < 0)
  {
    digitalWrite(LEFTM_PHASE_PIN, LOW);
    l_Speed = -l_Speed;
  }
  else digitalWrite(LEFTM_PHASE_PIN, HIGH);
  
  OCR4A = write_10bit_register(constrain(l_Speed,0,1023));
  
  if (r_Speed < 0)
  {
    digitalWrite(RIGHTM_PHASE_PIN, LOW);
    r_Speed = -r_Speed;
  }
  else digitalWrite(RIGHTM_PHASE_PIN, HIGH);
  
  OCR4D = write_10bit_register(constrain(r_Speed,0,1023));
}

/* This code is to set up the timers and pin directions for all PWM pins on the Arduino Micro.
 * The output compare registers should really only be accessed from within other functions that 
 * provide better abstraction, eg "void moveBarrel(int angle)" rather than writing the raw pulse length
 * directly to OCR3B.
 */

/* Timer assignments:
 * Timer 1 - PWM mode (8-bit) - RGB leds           --- expect 7.81 kHz
 * Timer 3 - Interrupt mode, interrupts control servo pins  --- expect   50  Hz
 * Timer 4 - PWM mode (OC4A & OC4D)             --- expect 23.4 kHz
 */
 
/* PIN ASSIGNMENTS - Arduino Pin Number - (access register) [range]
 * Motor 1: D13 (OCR4A) [0-1023]; LEFT/A; 'Phase' pin is D7
 * Motor 2: D6  (OCR4D) [0-1023]; RIGHT/B; 'Phase' pin is D8
 * Servo 1: D5 (OCR3A) [0-39999]; half-microSeconds
 * Servo 2: D4 (OCR3B) [0-39999]; half-microSeconds; ***software (interrupt) -based
 * RGB Red: D9  (OCR1A) [0-255]
 * RGB Grn: D10 (OCR1B) [0-255]
 * RGB Blu: D11 (OCR1C) [0-255]
 *
 * If possible we should use something like:
 * #define MOTOR1_SPEED OCR4A
 * 
 * IMPORTANT: OCR4A and OCR4D can ONLY be written with a call to write_10bit_register()
 */
 
 void initializeTimers()
 {
  /* Rev 1 - code adjusted for unscaled system clock (16MHz) */
   
  /* enable global interrupts */
  sei();
     
   
  /* Initialize Timer 1 for 8-bit PWM */
  OCR1A = 0x0080; /* initialize all OCRs to 50% duty cycle (0x0080) */
  OCR1B = 0x0080;
  OCR1C = 0x0080;
   
  TCCR1A = 0x00; /* clear all bits */   
  TCCR1B = 0x00; /* clear all bits */
  
  TCCR1A |= (1<<COM1A1) + (1<<COM1B1) + (1<<COM1C1) + (1<<WGM10); /* clear on compare match; 8-bitFPWM */
    DDRB |= _BV(5) + _BV(6) + _BV(7); /* set dig. pins 9,10,11 (PD5,PD6,PD7) to output mode */

  TCCR1B |= (1<<WGM12) + (1<<CS11); /* 8-bitFastPWM; prescaler = 8 (expected freq = 7.8kHz) */
  /* End Timer 1 */ 
   
   
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
   
   
  /* Initialize Hi-speed Timer 4 for 10-bit PWM on OC4A (Dig. pin 13) & OC4D (Dig. pin 6) */
  OCR4C = write_10bit_register(1023); /* OCR4C defines the "TOP" value for Timer/Counter 4 */
  OCR4A = write_10bit_register(512);
  OCR4D = write_10bit_register(512);

  TCCR4A = 0x00; /* clear all bits */
  TCCR4B = 0x00; /* clear all bits */
  TCCR4C = 0x00; /* clear all bits */
  TCCR4D = 0x00; /* clear all bits */
  TCCR4E = 0x00; /* clear all bits */
  TIMSK4 = 0x00; /* clear all bits */

  TCCR4A |= (1<<COM4A1) + (1<<PWM4A); /* clear OC4A (Dig. pin 13) on compare match; enable OC4A PWM */
    DDRC |= _BV(7); /* Set dig. pin 13 (PC7) to output mode*/     

  TCCR4B |= (1<<CS40); /* prescaler = 1 */
   
  TCCR4C |= (1<<COM4D1) + (1<<PWM4D); /* clear OC4D (Dig. pin 6) on compare match; enable OC4D PWM */
    DDRD |= _BV(7); /* Set dig. pin 6 (PD7) to output mode*/

  // set up phase pins for motors (pins 7 and 8)
  DDRB |= _BV(4);
  DDRE |= _BV(6);
   
  /* PLL control --- This may or may not affect USB functionality... */
  /* Connecting or disconnecting USB might screw up Timer4 operation until the next reset.
   * If USB is connected and enabled, the PLL will need no further configuration. Otherwise it will need
   * to be set up as follows: */
  if ((USBCON & (1<<USBE)) == 0)
  {
    /* if USB is disabled (i.e. USBE bit in USBCON is 0), enable the PLL */
    PLLFRQ = 0x00;  /* clear all bits */
      PLLFRQ |= (1<<PLLTM1) + (1<<PDIV2);
      /* divides frequency sent to Timer4 by 1.5; Sets raw frequency to 48MHz */
     
    /* CODE COPIED FROM "USBCore.cpp" */
    PLLCSR |= (1<<PINDIV); /* divide 16MHz cpu clock by 2 for input into PLL */
    PLLCSR |= (1<<PLLE); /* enable PhaseLockedLoop */
    while (!(PLLCSR & (1<<PLOCK)))    // wait for lock pll
    {
    }

    /* Some tests on specific versions of macosx (10.7.3), reported some
    // strange behaviors when the board is reset using the serial
    // port touch at 1200 bps. This delay fixes this behavior. */
    delay(1);
    /* END COPIED CODE */    
  }
  /* End Timer 4 */

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
 
uint8_t write_10bit_register(uint16_t i)
 {
  TC4H = (i >> 8);
  return (uint8_t)(i & 0xff);
 }

void playSound(int sound) {
  if (sound && soundBusy) {
    Serial.println(sound);
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
    digitalWrite(LASER, HIGH);
    digitalWrite(IR, LOW);
    playSound(1);
    lastShot = millis();
  }
}
