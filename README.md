# MegaPlay
Sega Genesis/Megadrive controller adapter with keypad emulation for Atari 5200

 This adapter lets you play Atari 5200 using a 6 button Sega Genesis controller yet providing full keypad control.
 
  The circuit is built around an Arduino Nano and two analog multiplexers.
  
  Interface with analog joystick is borrowed from "Low Priced MasterPlay clone adapter" and uses two resistors and a capacitor on each axis to convert the digital directionals into equivalent charging time for minimum, medium and maximum position on each axis.

  Keypad presses are emulated by activating a pair of analog multiplexers. Only 1 (one) keypress can be simulated at a time. 
  When there is no key pressed the "output" multiplexer is inhibited. 
  

