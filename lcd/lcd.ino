/*
  LiquidCrystal Library - Serial Input

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch displays text sent over the serial port
 (e.g. from the Serial Monitor) on an attached LCD.

 The circuit:
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://arduino.cc/en/Tutorial/LiquidCrystalSerial
 */

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define btnRIGHT  'R'
#define btnUP     'U'
#define btnDOWN   'D'
#define btnLEFT   'L'
#define btnSELECT 'S'
#define btnNONE   0

int select;

/**************************************************************/
/*関数名：read_LCD_buttons*/
/*動作；A0ポートの電圧値から押されたボタンを判別*/
/**************************************************************/
char read_LCD_button(void)
{
   int adc_key_in;
   
   adc_key_in = analogRead(0);
   if (adc_key_in < 50)   return btnRIGHT;  //0   , 戻り値0, 0V
   if (adc_key_in < 250)  return btnUP;     //144 , 戻り値1, 0.70V
   if (adc_key_in < 450)  return btnDOWN;   //329 , 戻り値2, 1.61V
   if (adc_key_in < 650)  return btnLEFT;   //504 , 戻り値3, 2.47V
   if (adc_key_in < 850)  return btnSELECT; //741 , 戻り値4, 3.62V
 
   return btnNONE;                          
}

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // initialize the serial communications:
  Serial.begin(9600);
  select = 1;
}

void loop()
{
  char c;
  
  // when characters arrive over the serial port...
  if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // read all the available characters
    while (Serial.available() > 0) {
      // display each character to the LCD
      c = Serial.read();
      switch (c) {
        case 0:
          lcd.clear();
        case 1:
          lcd.setCursor(0, 0);
          break;
        case 2:
          lcd.setCursor(0, 1);
          break;
        case 3:
          select = 1;
          break;
        default:
          lcd.write(c);
      }
    }
  }
  c = read_LCD_button();
  if ((c != btnNONE) && (select)) {
    select = 0;
    Serial.write(c);
  }
}
