/*******************************************************************************
       __  __               ___ _           
      |  \/  |___ __ _ __ _| _ \ |__ _ _  _ 
      | |\/| / -_) _` / _` |  _/ / _` | || |
      |_|  |_\___\__, \__,_|_| |_\__,_|\_, |
                 |___/                 |__/ 

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




/*******************************************************************************
 *         _      __ _      _ _   _             
 *      __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
 *     / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
 *     \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
 *                                              
 */
#define _key1        ((0<<3)|(0<<2)|(0<<1)|(0<<0)) // 0
#define _key2        ((0<<3)|(0<<2)|(0<<1)|(1<<0)) // 1
#define _key3        ((0<<3)|(0<<2)|(1<<1)|(0<<0)) // 2
#define _keyStart    ((0<<3)|(0<<2)|(1<<1)|(1<<0)) // 3
#define _key4        ((0<<3)|(1<<2)|(0<<1)|(0<<0)) // 4
#define _key5        ((0<<3)|(1<<2)|(0<<1)|(1<<0)) // 5
#define _key6        ((0<<3)|(1<<2)|(1<<1)|(0<<0)) // 6
#define _keyPause    ((0<<3)|(1<<2)|(1<<1)|(1<<0)) // 7
#define _key7        ((1<<3)|(0<<2)|(0<<1)|(0<<0)) // 8
#define _key8        ((1<<3)|(0<<2)|(0<<1)|(1<<0)) // 9
#define _key9        ((1<<3)|(0<<2)|(1<<1)|(0<<0)) // 10
#define _keyReset    ((1<<3)|(0<<2)|(1<<1)|(1<<0)) // 11
#define _keyAsterisk ((1<<3)|(1<<2)|(0<<1)|(0<<0)) // 12
#define _key0        ((1<<3)|(1<<2)|(0<<1)|(1<<0)) // 13
#define _keyHash     ((1<<3)|(1<<2)|(1<<1)|(0<<0)) // 14
#define _keyNone     ((1<<3)|(1<<2)|(1<<1)|(1<<0)) // 15

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

enum controllerType {
  _6Button=0,
  _3Button,
  _unKnown =0xff
  };

#define buttonUp     (1<<0)
#define buttonDown   (1<<1)
#define buttonLeft   (1<<2)
#define buttonRight  (1<<3)
#define buttonB      (1<<4)
#define buttonC      (1<<5)
#define buttonA      (1<<6)
#define buttonStart  (1<<7)
#define buttonZ      (1<<8)
#define buttonY      (1<<9)
#define buttonX      (1<<10)
#define buttonMode   (1<<11)

/*******************************************************************************
 *                   _      _    _        
 *     __ ____ _ _ _(_)__ _| |__| |___ ___
 *     \ V / _` | '_| / _` | '_ \ / -_|_-<
 *      \_/\__,_|_| |_\__,_|_.__/_\___/__/
 *                                        
 */

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

static volatile uint8_t CAVoff;
uint16_t combinedButtons = 0;



/*******************************************************************************
 *      _     _                         _      
 *     (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
 *     | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
 *     |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
 *                                |_|          
 */

// External interrupt, triggers every time CAV voltage changes state.
ISR (INT1_vect, ISR_NAKED) { 
  asm volatile (
    "push __tmp_reg__\n\t"         //
    "in __tmp_reg__,__SREG__\n\t"  //  Save Context
    "push __tmp_reg__\n\t"         //
    
    "clr __tmp_reg__\n\t"         // temp_reg=0 CAVoff = false

    "sbrc %[PORTIN],3 \n\t"       // CAV has dropped?
    "rjmp _STORE\n\t"             // no update value
    "cbi %[PORTPOTS],2\n\t"       // yes, force pin Y to zero
    "cbi %[DDRPOTS],2\n\t"
    "cbi %[PORTPOTS],3\n\t"       // yes, force pin X to zero
    "cbi %[DDRPOTS],3\n\t"
    "inc __tmp_reg__\n\t"         // temp_reg=1 CAVoff = true                        
    "_STORE:\n\t"
    "sts __tmp_reg__,%[flagCAVoff]\n\t" // store CAVoff flag value


    "pop __tmp_reg__\n\t"           //
    "out __SREG__,__tmp_reg__\n\t"  // Restore Context
    "pop __tmp_reg__\n\t"           //
    "reti \n\t"
    :[flagCAVoff]"=r"(CAVoff)
    :[PORTIN]"I" (_SFR_IO_ADDR(PIND)),[DDRPOTS]"I" (_SFR_IO_ADDR(DDRB)) ,[PORTPOTS]"I" (_SFR_IO_ADDR(PORTB)) 
    );  
  }



/*******************************************************************************
 *      ___      _             
 *     / __| ___| |_ _  _ _ __ 
 *     \__ \/ -_)  _| || | '_ \
 *     |___/\___|\__|\_,_| .__/
 *                       |_|   
 */
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
  pinMode(outputMuxInhibt,OUTPUT);
  digitalWrite(outputMuxInhibt,HIGH);
  delay(5); // wait for genesis internal logic timeout

  // Setup interrupts
  EICRA = (0<<ISC11)| (1<<ISC10); //  Any logical change on INT1 generates an interrupt request.
  EIMSK = (1<<INT1);              //  enable external interrupts on pin INT1 (D3)
  EIFR =  (1<<INT1); // clear any pending interrupt
//  sei();  // enable interrupts
Serial.begin(9600);
}




/*******************************************************************************
 *      __  __      _        _                  
 *     |  \/  |__ _(_)_ _   | |   ___  ___ _ __ 
 *     | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
 *     |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
 *                                        |_|   
 */
void loop() {
  uint8_t controllerType; // = ScanGenesis(); 

 //  for (;;) {
 
    controllerType = ScanGenesis();
    
    Serial.print("Type:");
    switch (controllerType) {
      case _6Button:
          Serial.print ("6 button ");
          break;
      case _3Button:
          Serial.print ("3 button ");
          break;
      default:
          Serial.print ("Master/unknown ");
          break;                    

      }

     Serial.print  ("buttons:");
//     combinedButtons |= (1<<15);
     Serial.print   ((combinedButtons | (1<<15)),BIN);

 //   Serial.println(); delay(8); // 8 milliseconds between readings.  
    
//    } // for (;;)


controllerType = _3Button;  // TOD REMOVE 
Serial.print ("faking 3 button ");
  
  if ( controllerType == _6Button)   {  // do 6 Button Stuff
    if (combinedButtons & buttonMode) {
      Serial.print (" m ");
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
        case buttonRight: // keypad 9
           setKeypad (_key9);
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
      
  
    } else { // Mode button released
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
    

    if (combinedButtons & buttonC )  {  // Modifier key pressed  
	   if (combinedButtons & buttonB) {  // Second modifier key pressed  
	      if (combinedButtons & buttonA) { // Third modifier key pressed (C+B+A)
		  // C+B+A pressed
		  combinedButtons &= ~(buttonC | buttonB | buttonA); // clear modifier buttons bits 
				switch (combinedButtons) {
					case buttonLeft:  // keypad 7
					   setKeypad (_key7);
					   break;		
					case buttonUp:    // keypad 8
					   setKeypad (_key8);
					   break;		
					case buttonRight: // keypad 9
					   setKeypad (_key9);
					   break;		
//			case buttonDown:  // keypad * 
//			   setKeypad (_keyAsterisk);
//			   break;
					case buttonStart:     // Reset
					   setKeypad (_keyReset);
					   break; 
//			case buttonA:     // keypad #
//			   setKeypad (_keyHash);
//			   break;
					default:       // None or multiple
					   setKeypad (_keyNone);		   
				} // switch			
		  } else { // C+B modifier keys pressed
		  combinedButtons &= ~(buttonC | buttonB); // clear modifier buttons bits 
				switch (combinedButtons) {
					case buttonLeft:  // keypad 4
					   setKeypad (_key4);
					   break;		
					case buttonUp:    // keypad 5
					   setKeypad (_key5);
					   break;		
					case buttonRight: // keypad 6
					   setKeypad (_key6);
					   break;		
//			case buttonDown:  // keypad * 
//			   setKeypad (_keyAsterisk);
//			   break;
					case buttonStart:     // Start
					   setKeypad (_keyStart);
					   break; 
//			case buttonA:     // keypad #
//			   setKeypad (_keyHash);
//			   break;
					default:       // None or multiple
					   setKeypad (_keyNone);		   
				} // switch	
		  } // else
		   
	} else { // Only C modifier key pressed
	combinedButtons &= ~(buttonC); // clear modifier buttons bits 		
		switch (combinedButtons) {
			case buttonLeft:  // keypad 1
			   setKeypad (_key1);
			   break;		
			case buttonUp:    // keypad 2
			   setKeypad (_key2);
			   break;		
			case buttonRight: // keypad 3
			   setKeypad (_key3);
			   break;		
			case buttonDown:  // keypad * 
			   setKeypad (_keyAsterisk);
			   break;
			case buttonStart: // keypad 0
			   setKeypad (_key0);
			   break; 
			case buttonA:     // keypad #
			   setKeypad (_keyHash);
			   break;
			default:       // None or multiple
			   setKeypad (_keyNone);		   
		} // switch	
	} // else
		
	} else { // No modifier key pressed
	
      // Take care of Start Button		
      if (combinedButtons & buttonStart) setKeypad (_keyPause); else setKeypad (_keyNone);


	  
      // Take care of buttons
      if (combinedButtons & (buttonA ) ) assertTopFire(); else releaseTopFire();
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
	  
	} // else

      
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



/*******************************************************************************
 *       __              _   _             
 *      / _|_  _ _ _  __| |_(_)___ _ _  ___
 *     |  _| || | ' \/ _|  _| / _ \ ' \(_-<
 *     |_|  \_,_|_||_\__|\__|_\___/_||_/__/
 *                                         
 */

void setKeypad ( uint8_t keyCode) {
  char key[16]={"123S456P789R*0#N"};
  Serial.print (" Key:");
  Serial.print (keyCode);
  Serial.print ("->");  
  Serial.println (key[keyCode]);

  if (keyCode & (1<<3) ) digitalWrite(outputMuxB,HIGH); else digitalWrite(outputMuxB,LOW);
  if (keyCode & (1<<2) ) digitalWrite(outputMuxA,HIGH); else digitalWrite(outputMuxA,LOW); 
  
  if (keyCode & (1<<1) ) digitalWrite(inputMuxB,HIGH); else digitalWrite(inputMuxB,LOW);
  if (keyCode & (1<<0) ) digitalWrite(inputMuxA,HIGH); else digitalWrite(inputMuxA,LOW);

  if ( keyCode == _keyNone ) {
    digitalWrite(outputMuxInhibt,HIGH);  // inhibt output 
  } else {
    digitalWrite(outputMuxInhibt,LOW);   
  }
}



//
//
void setMinimumX(void) {
  if (!CAVoff) {
    digitalWrite(potXfull,HIGH);
    pinMode(potXfull,OUTPUT);
  }
}

//
//
void setMaximumX(void) {
  if (!CAVoff) {
    pinMode(potXhalf,INPUT);
    digitalWrite(potXhalf,LOW); // turn off pullup
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup
  } 
}

//
//
void setMiddleX(void){
  if (!CAVoff) {
    pinMode(potXfull,INPUT);
    digitalWrite(potXfull,LOW);  // turn off pullup   
    digitalWrite(potXhalf,HIGH);    
    pinMode(potXhalf,OUTPUT);
  } 
}

//
//
void setMinimumY(void) {
  if (!CAVoff) {
    digitalWrite(potYfull,HIGH);
    pinMode(potYfull,OUTPUT);
  }
}

//
//
void setMaximumY(void) {
  if (!CAVoff) {
    pinMode(potYhalf,INPUT);
    digitalWrite(potYhalf,LOW); // turn off pullup
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup
  } 
}

//
//
void setMiddleY(void){
  if (!CAVoff) {
    pinMode(potYfull,INPUT);
    digitalWrite(potYfull,LOW);  // turn off pullup   
    digitalWrite(potYhalf,HIGH);    
    pinMode(potYhalf,OUTPUT);
  } 
}

//
//
void assertTopFire(void) {
  digitalWrite(fireTop,LOW); // turn off pullup
  pinMode(fireTop,OUTPUT);   
}

//
//
void releaseTopFire(void) {
  pinMode(fireTop,INPUT);
  digitalWrite(fireTop,HIGH); // turn on pullup
}

//
//
void assertBottomFire(void) {
  digitalWrite(fireBottom,LOW); // turn off pullup
  pinMode(fireBottom,OUTPUT);   
}

//
//
void releaseBottomFire(void) {
  pinMode(fireBottom,INPUT);
  digitalWrite(fireBottom,HIGH); // turn on pullup
}

//
//
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
  sample[5] =  PINC & 0x3f;           //  1  1  MD X  Y  Z  

  digitalWrite(genesisSelect,LOW);   //
  delayMicroseconds(10);
  sample[6] = PINC & 0x3f;            //  ST A  1  1  1  1 -> 3 button:  ST A  0  0  DW  UP 
    

  // check for 3 or 6 buttons
  if ( ((sample[4] & 0x03) == 0) && ((sample[6] & 0x0f)==0x0f) ) {
  type = _6Button;  
  } else if  ( (sample[6] & 0x0c) == 0)  {
  type = _3Button;
  } else
    return _unKnown; // unknown

 
  // now populate combinedButtons variable accordingly       // 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  combinedButtons = (uint16_t)sample[1];                     // 0  0  0  0  0  0  0  0  0  0  C  B  RG LF DW UP
  combinedButtons |= ((uint16_t)(sample[0]<<2)) & 0xc0;      // 0  0  0  0  0  0  0  0  ST A  C  B  RG LF DW UP
  combinedButtons |= ((uint16_t)(sample[5]<<8)) & 0xf00;     // 0  0  0  0  MD X  Y  Z  ST A  C  B  RG LF DW UP

  // invert bits. Make '1' the active state 
  combinedButtons = ~combinedButtons;
  switch (type) {
    case _6Button:
      combinedButtons &= 0x0fff;
      break;

    case _3Button:
      combinedButtons &= 0x00ff;

    default:
      combinedButtons &= 0x003f;    
    } // case

  

  return type;
}