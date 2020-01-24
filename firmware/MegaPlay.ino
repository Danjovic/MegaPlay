/*
  MegaPlay adapter for Atari 5200 - Danjovic 2020 - danjovic@hotmail.com - https://hackaday.io/danjovic

  24 January 2020

  This adapter lets you play Atari 5200 using a 6 button Sega Genesis controller yet providing full keypad control.
  The circuit is built around an Arduino Nano and two analog multiplexers.
  Interface with analog joystick is borrowed from "Low Priced MasterPlay clone adapter" and uses two resistors and
  a capacitor on each axis to convert the digital directionals into equivalent charging time for minimum, medium and 
  maximum position on each axis.

  Keypad presses are emulated by activating a pair of analog multiplexers. Only 1 (one) keypress can be simulated at a time. 
  When there is no key pressed the "output" multiplexer is inhibited. Buttons are mapped as follows:

                         +--------------------+ 
                         |      6 buttons     | 
    +--------+-----------+------MODE key------+ 
    | Button | 3 buttons | Released | Pressed | 
    +--------+ ----------+----------+---------+ 
    | UP     |    UP     |    UP    |    8    | 
    | DOWN   |   DOWN    |  DOWN    |  Reset  | 
    | LEFT   |   LEFT    |  LEFT    |    7    | 
    | RIGHT  |   RIGHT   |  RIGHT   |    9    | 
    |  A     |    Top    |   Top    |    4    | 
    |  B     |   Bottom  |  Bottom  |    5    | 
    |  C     |    Top    |   Top    |    6    | 
    | START  |   Pause   |  Pause   |  Start  | 
    +--------+-----------+----------+---------+ 
    |  X     |     -     |    *     |    1    | 
    |  Y     |     -     |    0     |    2    | 
    |  Z     |     -     |    #     |    3    | 
    +--------+-----------+----------+---------+  

  CAV Voltage on PIN 9 is connected to INT1 IRQ pin and is used to promptly turn off the timing pins whenever CAV drops to zero. 
  A zener diode as added to protect the AVR input, as the CAV voltage can reach up to 6.4Volts.


/*
     
      +------- Output Mux -MSbits-----+
Input |   7       6       5       8
 Mux  |  0 0  |  0 1  |  1 0  |  1 1  |
 B A  +-------+-------+-------+-------+ 
 0 0  |   1   |   4   |   7   |   *   | 3
 0 1  |   2   |   5   |   8   |   0   | 2
 1 0  |   3   |   6   |   9   |   #   | 1
 1 1  | start | pause | reset |   -   | 4
      +-------+-------+-------+-------+
*/

#define inputMuxA       6
#define inputMuxB       7
#define outputMuxA      0
#define outputMuxB      1
#define outputMuxInhibt 2

#define _key1        (0<<3)|(0<<2)|(0<<1)|(0<<0) // 0
#define _key2        (0<<3)|(0<<2)|(0<<1)|(1<<0) // 1
#define _key3        (0<<3)|(0<<2)|(1<<1)|(0<<0) // 2
#define _keyStart    (0<<3)|(0<<2)|(1<<1)|(1<<0) // 3
#define _key4        (0<<3)|(1<<2)|(0<<1)|(0<<0) // 4
#define _key5        (0<<3)|(1<<2)|(0<<1)|(1<<0) // 5
#define _key6        (0<<3)|(1<<2)|(1<<1)|(0<<0) // 6
#define _keyPause    (0<<3)|(1<<2)|(1<<1)|(1<<0) // 7
#define _key7        (1<<3)|(0<<2)|(0<<1)|(0<<0) // 8
#define _key8        (1<<3)|(0<<2)|(0<<1)|(1<<0) // 9
#define _key9        (1<<3)|(0<<2)|(1<<1)|(0<<0) // 10
#define _keyReset    (1<<3)|(0<<2)|(1<<1)|(1<<0) // 11
#define _keyAsterisk (1<<3)|(1<<2)|(0<<1)|(0<<0) // 12
#define _key0        (1<<3)|(1<<2)|(0<<1)|(1<<0) // 13
#define _keyHash     (1<<3)|(1<<2)|(1<<1)|(0<<0) // 14
#define _keyNone     (1<<3)|(1<<2)|(1<<1)|(1<<0) // 15

#define inputMuxA       6
#define inputMuxB       7
#define outputMuxA      0
#define outputMuxB      1
#define outputMuxInhibt 2

#define genesisUp       A0
#define genesisDown     A1
#define genesisLeft     A2
#define genesisRigth    A3
#define genesisB        A4
#define genesisC        A5
#define genesisSelect   13

#define potXfull        11
#define potXhalf        12

#define potYfull        10
#define potYhalf        9

#define fireTop         4
#define fireBottom      5

#define _6Button 0
#define _3Button 1

#define buttonUp     (1<<0)
#define buttonDown   (1<<1)
#define buttonLeft   (1<<2)
#define buttonRight  (1<<3)
#define buttonA      (1<<4)
#define buttonB      (1<<5)
#define buttonC      (1<<6)
#define buttonX      (1<<7)
#define buttonY      (1<<8)
#define buttonZ      (1<<9)

#define buttonStart  (1<<10)
#define buttonMode   (1<<11)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////





uint8_t ScanGenesis(void);
void assertTopFire(void);
void releaseTopFire(void);
void assertBottomFire(void);
void releaseBottomFire(void);
void setMinimumY(void);
void setMaximumY(void);
void setMiddleY(void); 
void setMinimumX(void);
void setMaximumX(void);
void setMiddleX(void); 


static volatile uint8_t CAVon;
uint16_t combinedButtons = 0;


TODO: // ISR para desativar CAV e variavel CAVon
         Programar interrupcoes para pin change ou externa



void setup() {
  // put your setup code here, to run once:
  pinMode(genesisUp    ,INPUT_PULLUP);
  pinMode(genesisDown  ,INPUT_PULLUP);
  pinMode(genesisLeft  ,INPUT_PULLUP);
  pinMode(genesisRigth ,INPUT_PULLUP);
  pinMode(genesisB     ,INPUT_PULLUP);
  pinMode(genesisC     ,INPUT_PULLUP);
  pinMode(genesisSelect,OUTPUT);
  digitalWrite(genesisSelect,LOW);
  delay(5); // wait for genesis internal logic timeout
}

void loop() {
  uint8_t controllerType = ScanGenesis(); 
  if ( controllerType == _6Button)  {  // do 6 Button Stuff
    if (combinedButtons & buttonMode) {
      combinedButtons &= ~buttonMode;  // clear Mode button bit 
      switch(combinedButtons) {
        case buttonUp:    // keypad 8
           setKeypad (_key8);
           break;
        case buttonDown:  // Reset
           setKeypad (_keyReset);
           break;
        case buttonLeft:  // keypad 7
           setKeypad (_key7);
           break;
        case buttonRight: // keypad 6
           setKeypad (_key6);
           break;
        case buttonA:     // keypad 4
           setKeypad (_key4);
           break; 
        case buttonB:     // keypad 5
           setKeypad (_key5);
           break;
        case buttonC:     // keypad 6
           setKeypad (_key6);
           break;  
        case buttonX:     // keypad 1
           setKeypad (_key1);
           break;
        case buttonY:     // keypad 2
           setKeypad (_key2);
           break;    
        case buttonZ:     // keypad 3
           setKeypad (_key3);
           break;
        case buttonStart: // Start
           setKeypad (_keyStart);
           break;
        default:       // None or multiple
           setKeypad (_keyNone);    
      }
      
  
    } else { // 
      switch(combinedButtons) {
        case buttonX:     // keypad * 
           setKeypad (_keyAsterisk);
           break;    
        case buttonY:     // keypad 0
           setKeypad (_key0);
           break;    
        case buttonZ:     // keypad #
           setKeypad (_keyHash);
           break;
        case buttonStart: // Pause
           setKeypad (_keyPause);
           break;
        default:       // None or multiple
           setKeypad (_keyNone);
      }
      
      // Take care of buttons
      if (combinedButtons & (buttonA | buttonC) ) assertTopFire(); else releaseTopFire();
      if (combinedButtons &  buttonB ) assertBottomFire(); else releaseBottomFire();
      
      // Take care of directionals
      if ( (combinedButtons & (buttonUp)) && !(combinedButtons & (buttonDown) ) ) { // minimum Y
        setMinimumY();
      } else if (!buttonUp && buttonDown) { // maximum Y
        setMaximumY();
      } else { // Y at middle
        setMiddleY();   
      } 
  
      if (buttonLeft && !buttonRight) { // minimum X
        setMinimumX();
      } else if (!buttonLeft && buttonRight) { // maximum X
        setMaximumX();
      } else { // X at middle
        setMiddleX();   
      } 
  
    }
  } else if (controllerType == _3Button ) { // work with 3 button, limited functionality
    
      switch(combinedButtons) {
        case buttonStart: // Pause
           setKeypad (_keyPause);
           break;
        default:       // None or multiple
           setKeypad (_keyNone);
      }
      
      // Take care of buttons
      if (combinedButtons & (buttonA | buttonC) ) assertTopFire(); else releaseTopFire();
      if (combinedButtons &  buttonB ) assertBottomFire(); else releaseBottomFire();
      
      // Take care of directionals
      if ( (combinedButtons & (buttonUp)) && !(combinedButtons & (buttonDown) ) ) { // minimum Y
        setMinimumY();
      } else if (!buttonUp && buttonDown) { // maximum Y
        setMaximumY();
      } else { // Y at middle
        setMiddleY();   
      } 
  
      if (buttonLeft && !buttonRight) { // minimum X
        setMinimumX();
      } else if (!buttonLeft && buttonRight) { // maximum X
        setMaximumX();
      } else { // X at middle
        setMiddleX();   
      }
      
  } else { // No controller detected, 
    setKeypad (_keyNone);
    setMiddleX();
    setMiddleY();
    releaseBottomFire();
    releaseTopFire();
    delay(100);
  }
  delay(8); // 8 milliseconds between readings.

}


void setKeypad ( uint8_t keyCode) {

  if (keyCode & (1<<3) ) digitalWrite(outputMuxB,HIGH); else digitalWrite(outputMuxB,LOW);
  if (keyCode & (1<<2) ) digitalWrite(outputMuxA,HIGH); else digitalWrite(outputMuxA,LOW); 
  
  if (keyCode & (1<<1) ) digitalWrite(inputMuxB,HIGH); else digitalWrite(inputMuxB,LOW);
  if (keyCode & (1<<0) ) digitalWrite(inputMuxA,HIGH); else digitalWrite(inputMuxA,LOW);
  
  if (keyCode==_keyNone ) {
    digitalWrite(outputMuxInhibt,HIGH);  // inhibt output 
  } else {
    digitalWrite(outputMuxInhibt,LOW);    
  }
  
}



void setMinimumX(void) {
  if (CAVon) {
    digitalWrite(potXfull,HIGH);
    pinMode(potXfull,OUTPUT);
  }
}
void setMaximumX(void) {
  if (CAVon) {
    pinMode(potXhalf,INPUT);
    digitalWrite(potXhalf,LOW); // turn off pullup
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup
  } 
}
void setMiddleX(void){
  if (CAVon) {
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup   
    digitalWrite(potXhalf,HIGH);    
    pinMode(potXhalf,OUTPUT);
  } 
}

void setMinimumY(void) {
  if (CAVon) {
    digitalWrite(potYfull,HIGH);
    pinMode(potYfull,OUTPUT);
  }
}
void setMaximumY(void) {
  if (CAVon) {
    pinMode(potYhalf,INPUT);
    digitalWrite(potYhalf,LOW); // turn off pullup
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup
  } 
}
void setMiddleY(void){
  if (CAVon) {
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup   
    digitalWrite(potYhalf,HIGH);    
    pinMode(potYhalf,OUTPUT);
  } 
}

void assertTopFire(void) {
  digitalWrite(fireTop,LOW); // turn off pullup
  pinMode(fireTop,OUTPUT);   
}
void releaseTopFire(void) {
  pinMode(fireTop,INPUT);
  digitalWrite(fireTop,HIGH); // turn on pullup
}

void assertBottomFire(void) {
  digitalWrite(fireBottom,LOW); // turn off pullup
  pinMode(fireBottom,OUTPUT);   
}
void releaseBottomFire(void) {
  pinMode(fireBottom,INPUT);
  digitalWrite(fireBottom,HIGH); // turn on pullup
}

uint8_t ScanGenesis(void) {

  uint8_t sample[7]; 
  uint8_t type;  
  combinedButtons = 0;

  
//  delayMicroseconds(10);           //  5  4  3  2  1  0  -> BIT

  sample[0] = PINC & 0x3f;           //  ST A  0  0  DW UP
  
  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10); 
  sample[1] = PINC & 0x3f;           //  C  B  RG LF DW UP

  digitalWrite(genesisSelect,LOW);   //  
  delayMicroseconds(10);
  sample[2] = PINC & 0x3f;           //  ST A  0  0  DW UP 
 
  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10);
  sample[3] = PINC & 0x3f;           //  C  B  RG LF DW UP 

  digitalWrite(genesisSelect,LOW);   // 
  delayMicroseconds(10);
  sample[4] = PINC & 0x3f;           //   ST A  0  0  0  0  -> 3 button: ST A  0  0  DW  UP 

  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10);
  sample[5] = PINC & 0x3f;           //  1  1  MD X  Y  Z  

  digitalWrite(genesisSelect,LOW);   //
  delayMicroseconds(10);
  sample[6] = PINC & 0x3f;            //  ST A  1  1  1  1 -> 3 button:  ST A  0  0  DW  UP 
    

  // check for 3 or 6 buttons
  if ( ((sample[4] & 0x03) == 0) && ((sample[6] & 0x0f)==0x0f) ) {
  type = _6Button;  
  } else if  ( (sample[6] & 0x0c) == 0)  {
  type = _3Button;
  } else
    return 0xff; // unknown

  // now populate combinedButtons variable accordingly
                                                            // 11 10 9  8  7  6  5  4  3  2  1  0
  combinedButtons = (uint16_t)sample[1];                    // 0  0  0  0  0  0  C  B  RG LF DW UP
  combinedButtons |= ((uint16_t)(sample[0]<<2)) & 0xc0;     // 0  0  0  0  ST A  C  B  RG LF DW UP

  if (type == _6Button) 
     combinedButtons |= ((uint16_t)(sample[5]<<8)) & 0xf00; // MD X  Y  Z  0  ST A  C  B  RG LF DW UP

  return type;
}
