/*
 * Transmitter file is designed to transmit a packet of 3 bytes.
 * -------------------------------------------------------------
 * The Packet which is to be sent is hard-coded.
 *
 * Pattern for the Packet is:
 * 1st Byte = Length of Packet
 * 2nd Byte = Your Data
 * 3rd Byte = More Data
 * .
 * .
 * Nth Byte = Last Byte of Data
 *
 * -------------------------------------------------------------
 *
 * Hard-coded Packet for this sample program is:
 * 1st Byte = 3;    // Length of Packet
 * 2nd Byte = 0x09;
 * 3rd Byte = 0x01;
 *
 * Receiver will get the Packet in the above Pattern.
 *
 * -------------------------------------------------------------
 *
 * To run this program properly, do the following steps:
 *
 * 1. Connect the GDO0-Pin of CC2500 With Arduino's Pin-4
 * 2. Connect a Push Button(Active-High) with Arduino's Pin-2
 * 3. Transmitter will continously trransmit the Hard-coded Packet Until You Release the Push-Button to Low
 *
 * -------------------------------------------------------------
 *
 * Precautions:
 * - Do appropriate current and voltage conversion between your microcontroller and CC2500 module.
 * - High voltage or High current may damage your CC2500 Module.
 */

#include <cc2500_REG.h>
#include <cc2500_VAL.h>
#include <Arduino.h>
#include <SPI.h>

// Define direction bits
#define TOP_BIT    (1 << 0)
#define BOTTOM_BIT (1 << 5)
#define RIGHT_BIT  (1 << 2)
#define LEFT_BIT   (1 << 3)
#define STOP_BIT   (1 << 4)

#define CC2500_IDLE 0x36 // Exit RX / TX, turn
#define CC2500_TX 0x35   // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX 0x34   // Enable RX. Perform calibration if enabled
#define CC2500_FTX 0x3B  // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX 0x3A  // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO 0x3F
#define CC2500_RXFIFO 0x3F

#define No_of_Bytes 2

const int X_PIN = 36;      // Connect VRx of the joystick to GPIO 34
const int Y_PIN = 39;      // Connect VRy of the joystick to GPIO 35
const int BUTTON_PIN = 32; // Connect SW of the joystick to GPIO 32

// const int buttonPin = 2;     // the number of the pushbutton pin
// int buttonState = 0;         // variable for reading the pushbutton status
const int touchPin = 27;
const int ledPin = 2;

const int threshold = 30; // set the threshold

const int GDO0_PIN = 4;   // the number of the GDO0_PIN pin
int GDO0_State = 0;       // variable for reading the pushbutton status
int ledState = LOW;       // the current state of the output pin
int touchState;           // the current reading from the input pin
int lastTouchState = LOW; // the previous reading from the input pin
int StopFlag = 0;
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output

char TxData[2];

void TxData_RF(unsigned char length);
void WriteReg(char addr, char value);
char ReadReg(char addr);
char SendStrobe(char strobe);
void init_CC2500();
void Read_Config_Regs(void);
void intToHex(int num, char hexString[]);
uint8_t createDirectionData(int top, int bottom, int right, int left, int stop);

void setup()
{
  Serial.begin(9600);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SS, OUTPUT);
  SPI.begin();
  digitalWrite(SS, HIGH);
  // initialize the pushbutton pin as an input:
  // pinMode(buttonPin, INPUT);
  pinMode(GDO0_PIN, INPUT);

  pinMode(ledPin, OUTPUT);

  // set initial LED state
  digitalWrite(ledPin, ledState);

  Serial.println("Starting Transmiter..");
  init_CC2500();

  /* This function is to make sure that cc2500 is successfully configured.
   * This function read values of some registers from CC2500.
   * To use this function, you must read the register values from
   * -> Arduino-CC2500-Library / CC2500_Library / cc2500_VAL.h
   * Then compare the values from result of this function..
   */
  Read_Config_Regs();
}

void loop()
{
  int xValue = analogRead(X_PIN);
  int yValue = analogRead(Y_PIN);
  int buttonState = digitalRead(BUTTON_PIN);

  // Map analog values to a range (0-1023) to a smaller range, if necessary
  int mappedX = map(xValue, 0, 4095, 0, 100);
  int mappedY = map(yValue, 0, 4095, 0, 100);
  // intToHex(mappedX, hexMappedX);
  // intToHex(mappedY, hexMappedY);

  // Serial.print("X-axis: ");
  // Serial.print(mappedX);
  // Serial.print(hexMappedX[1]);
  // Serial.print("\r\n");
  // Serial.print("\r\n");
  // Serial.print("Y-axis: ");
  // Serial.print(mappedY);
  // Serial.print(hexMappedY[1]);
  // Serial.print("\r\n");
  // Serial.print("\r\n");
  // Serial.print("hex, Button: ");
  // Serial.println(buttonState);
  // buttonState = digitalRead(buttonPin);
  // int reading = touchRead(touchPin);
  // Serial.println(buttonState);

  // binarize touch reading for easy operation
  // if (reading < threshold)
  // {
  //   reading = HIGH;
  // }
  // else
  // {
  //   reading = LOW;
  // }

  // If the pin is touched:
  // if (reading != lastTouchState)
  // {
  //   // reset the debouncing timer
  //   lastDebounceTime = millis();
  // }

  // if ((millis() - lastDebounceTime) > debounceDelay)
  // {
  //   // whatever the reading is at, it's been there for longer than the debounce
  //   // delay, so take it as the actual current state:

  //   // if the touch state has changed:
  //   if (reading != touchState)
  //   {
  //     touchState = reading;

  //     // only toggle the LED if the new touch state is HIGH
  //     if (touchState == HIGH)
  //     {
  //       ledState = !ledState;
  //     }
  //   }
  // }

  // set the LED:
  // digitalWrite(ledPin, 1);

  // save the reading. Next time through the loop, it'll be the lastTouchState:
  // lastTouchState = reading;
  //  To start transmission
  // while (buttonState)
  // while (touchState)

  {
    // read the state of the pushbutton value:
    // buttonState = digitalRead(buttonPin);
    // int reading = touchRead(touchPin);
    // Serial.println(buttonState);

    // binarize touch reading for easy operation
    // if (reading < threshold)
    // {
    //   reading = HIGH;
    // }
    // else
    // {
    //   reading = LOW;
    // }

    // If the pin is touched:
    // if (reading != lastTouchState)
    // {
    //   // reset the debouncing timer
    //   lastDebounceTime = millis();
    // }

    // if ((millis() - lastDebounceTime) > debounceDelay)
    // {
    //   // whatever the reading is at, it's been there for longer than the debounce
    //   // delay, so take it as the actual current state:

    //   // if the touch state has changed:
    //   if (reading != touchState)
    //   {
    //     touchState = reading;

    //     // only toggle the LED if the new touch state is HIGH
    //     if (touchState == HIGH)
    //     {
    //       ledState = !ledState;
    //     }
    //   }
    // }

    // set the LED:
    // digitalWrite(ledPin, ledState);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*top
    x = 4095 /100
    Y = 1906 /46

    Bottom
    X = 0
    Y = 1968 /46

    Left
    X = 2018 /49
    Y = 0

    Right
    X = 2015 /44
    Y = 4095 /100

    Center
    X = 49
    Y = 46*/
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // save the reading. Next time through the loop, it'll be the lastTouchState:
    // lastTouchState = reading;

    if (!((mappedX <= 55 && mappedX >= 40) && (mappedY <= 55 && mappedY >= 40)))
    {
      if (((mappedX <= 100 && mappedX >= 90) && (mappedY <= 50 && mappedY >= 40)))
      {
        Serial.println("Transmitting TOP signal...");
        digitalWrite(ledPin, HIGH);
        delay(10);
        TxData[0] = createDirectionData(1, 0, 0, 0, 0);
        // TxData[0] = 0x1;
        // TxData[1] = 0x0;
        // TxData[2] = 0x0;
        // TxData[3] = 0x0;
        TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
        Serial.println("Transmission is over");
        Serial.print("\r\n");
        digitalWrite(ledPin, LOW);
        memset(TxData, 0, sizeof(TxData));
        delay(100);
        digitalWrite(ledPin, 0);
      }
      else if (((mappedX <= 10 && mappedX >= 0) && (mappedY <= 55 && mappedY >= 40)))
      {
        Serial.println("Transmitting BOTTOM signal...");
        digitalWrite(ledPin, HIGH);
        delay(10);
        TxData[0] = createDirectionData(0, 1, 0, 0, 0);
        // TxData[0] = 0x0;
        // TxData[1] = 0x1;
        // TxData[2] = 0x0;
        // TxData[3] = 0x0;
        TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
        Serial.println("Transmission is over");
        Serial.print("\r\n");
        digitalWrite(ledPin, LOW);
        memset(TxData, 0, sizeof(TxData));
       delay(100);
        digitalWrite(ledPin, 0);
      }
      else if (((mappedX <= 55 && mappedX >= 40) && (mappedY <= 10 && mappedY >= 0)))
      {
        Serial.println("Transmitting LEFT signal...");
        digitalWrite(ledPin, HIGH);
        TxData[0] = createDirectionData(0, 0, 0, 1, 0);
        delay(10);
        // TxData[0] = 0x0;
        // TxData[1] = 0x0;
        // TxData[2] = 0x1;
        // TxData[3] = 0x0;
        TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
        Serial.println("Transmission is over");
        Serial.print("\r\n");
        digitalWrite(ledPin, LOW);
        memset(TxData, 0, sizeof(TxData));
        delay(100);
        // delay(500);
        digitalWrite(ledPin, 0);
      }
      else if (((mappedX <= 55 && mappedX >= 40) && (mappedY <= 100 && mappedY >= 90)))
      {
        Serial.println("Transmitting RIGHT signal...");
        digitalWrite(ledPin, HIGH);
        delay(10);
        TxData[0] = createDirectionData(0, 0, 1, 0, 0);
        // TxData[0] = 0x0;
        // TxData[1] = 0x0;
        // TxData[2] = 0x0;
        // TxData[3] = 0x1;
        TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
        Serial.println("Transmission is over");
        Serial.print("\r\n");
        digitalWrite(ledPin, LOW);
        memset(TxData, 0, sizeof(TxData));
        delay(100);
        // delay(500);
        digitalWrite(ledPin, 0);
      }
      
      // }
    }
    else{
          Serial.println("Transmitting STOP signal...");
          digitalWrite(ledPin, HIGH);
          delay(10);
          TxData[0] = createDirectionData(0, 0, 0, 0, 1);
          TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
          Serial.println("Transmission is over");
          Serial.print("\r\n");
          digitalWrite(ledPin, LOW);
          memset(TxData, 0, sizeof(TxData));
          // delay(500);
          digitalWrite(ledPin, 0);
        for (size_t i = 0; i < 10; i++)
        {
          delay(500);

          int x_Value = analogRead(X_PIN);
          int y_Value = analogRead(Y_PIN);
          if (x_Value || y_Value)
          {
            i = 9;
          }
          
          /* code */
        }
      }
    // else{
    //   delay(100);
    //   Serial.println("Transmitting STOP signal...");
    //     digitalWrite(ledPin, HIGH);
    //     delay(10);
    //     TxData[0] = createDirectionData(0, 0, 0, 0, 1);
    //     // TxData[0] = 0x0;
    //     // TxData[1] = 0x0;
    //     // TxData[2] = 0x0;
    //     // TxData[3] = 0x1;
    //     TxData_RF(No_of_Bytes); //  Transmit No_of_Bytes-1
    //     Serial.println("Transmission is over");
    //     Serial.print("\r\n");
    //     digitalWrite(ledPin, LOW);
    //     memset(TxData, 0, sizeof(TxData));
    //     // delay(500);
    //      digitalWrite(ledPin, 0);
    // }
  }

  /*
 while (buttonState)
   {
   // read the state of the pushbutton value:
   buttonState = digitalRead(buttonPin);
   Serial.println("PB = 1");
   }
   */
}

// Function to create 1-byte data based on direction
uint8_t createDirectionData(int top, int bottom, int right, int left, int stop) {
    uint8_t directionData = 0;

    // Set direction bits
    if (top) {
        directionData |= TOP_BIT;
    }

    if (bottom) {
        directionData |= BOTTOM_BIT;
    }

    if (right) {
        directionData |= RIGHT_BIT;
    }

    if (left) {
        directionData |= LEFT_BIT;
    }

    if (stop) {
        directionData |= STOP_BIT;
    }

    return directionData;
}


//  Send slide strobe
void TxData_RF(unsigned char length)
{
  // Make sure that the radio is in IDLE state before flushing the FIFO
  SendStrobe(CC2500_IDLE);
  // Flush TX FIFO
  SendStrobe(CC2500_FTX);

  // prepare Packet
  unsigned char packet[length];
  // First Byte = Length Of Packet
  packet[0] = length;
  // packet[1] = 0x4;
  // packet[2] = hexMappedX[1];
  // packet[3] = hexMappedY[0];
  // packet[4] = hexMappedY[1];
  packet[1] = TxData[0];
  // packet[2] = TxData[1];
  // packet[3] = TxData[2];
  // packet[4] = TxData[3];
  /*
  for(int i = 1; i < length; i++)
  {
      packet[i] = i;
  }

  */

  // SIDLE: exit RX/TX
  SendStrobe(CC2500_IDLE);

  for (int i = 0; i < length; i++)
  {
    Serial.println("Transmitting ");
    Serial.println(packet[i], HEX);
    WriteReg(CC2500_TXFIFO, packet[i]);
  }
  // STX: enable TX
  SendStrobe(CC2500_TX);

  // Wait for GDO0 to be set -> sync transmitted
  while (!GDO0_State)
  {
    // read the state of the GDO0_PIN value:
    GDO0_State = digitalRead(GDO0_PIN);
    // Serial.println("GDO0 = 0");
  }

  // Wait for GDO0 to be cleared -> end of packet
  while (GDO0_State)
  {
    // read the state of the GDO0_PIN value:
    GDO0_State = digitalRead(GDO0_PIN);
    // Serial.println("GDO0 = 1");
  }
} // Rf TX Packet

void WriteReg(char addr, char value)
{
  digitalWrite(SS, LOW);

  while (digitalRead(MISO) == HIGH)
  {
  };

  SPI.transfer(addr);
  delay(10);
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
}

char ReadReg(char addr)
{
  addr = addr + 0x80;
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH)
  {
  };
  char x = SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  return y;
}

char SendStrobe(char strobe)
{
  digitalWrite(SS, LOW);

  while (digitalRead(MISO) == HIGH)
  {
  };

  char result = SPI.transfer(strobe);
  digitalWrite(SS, HIGH);
  delay(10);
  return result;
}

void init_CC2500()
{
  WriteReg(REG_IOCFG2, VAL_IOCFG2);
  WriteReg(REG_IOCFG1, VAL_IOCFG1);
  WriteReg(REG_IOCFG0, VAL_IOCFG0);

  WriteReg(REG_FIFOTHR, VAL_FIFOTHR);
  WriteReg(REG_SYNC1, VAL_SYNC1);
  WriteReg(REG_SYNC0, VAL_SYNC0);
  WriteReg(REG_PKTLEN, VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1, VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0, VAL_PKTCTRL0);
  WriteReg(REG_ADDR, VAL_ADDR);
  WriteReg(REG_CHANNR, VAL_CHANNR);
  WriteReg(REG_FSCTRL1, VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0, VAL_FSCTRL0);
  WriteReg(REG_FREQ2, VAL_FREQ2);
  WriteReg(REG_FREQ1, VAL_FREQ1);
  WriteReg(REG_FREQ0, VAL_FREQ0);
  WriteReg(REG_MDMCFG4, VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3, VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2, VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1, VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0, VAL_MDMCFG0);
  WriteReg(REG_DEVIATN, VAL_DEVIATN);
  WriteReg(REG_MCSM2, VAL_MCSM2);
  WriteReg(REG_MCSM1, VAL_MCSM1);
  WriteReg(REG_MCSM0, VAL_MCSM0);
  WriteReg(REG_FOCCFG, VAL_FOCCFG);

  WriteReg(REG_BSCFG, VAL_BSCFG);
  WriteReg(REG_AGCCTRL2, VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1, VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0, VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1, VAL_WOREVT1);
  WriteReg(REG_WOREVT0, VAL_WOREVT0);
  WriteReg(REG_WORCTRL, VAL_WORCTRL);
  WriteReg(REG_FREND1, VAL_FREND1);
  WriteReg(REG_FREND0, VAL_FREND0);
  WriteReg(REG_FSCAL3, VAL_FSCAL3);
  WriteReg(REG_FSCAL2, VAL_FSCAL2);
  WriteReg(REG_FSCAL1, VAL_FSCAL1);
  WriteReg(REG_FSCAL0, VAL_FSCAL0);
  WriteReg(REG_RCCTRL1, VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0, VAL_RCCTRL0);
  WriteReg(REG_FSTEST, VAL_FSTEST);
  WriteReg(REG_PTEST, VAL_PTEST);
  WriteReg(REG_AGCTEST, VAL_AGCTEST);
  WriteReg(REG_TEST2, VAL_TEST2);
  WriteReg(REG_TEST1, VAL_TEST1);
  WriteReg(REG_TEST0, VAL_TEST0);
  WriteReg(REG_RSSI, VAL_RSSI);
  /*
    WriteReg(REG_PARTNUM,VAL_PARTNUM);
    WriteReg(REG_VERSION,VAL_VERSION);
    WriteReg(REG_FREQEST,VAL_FREQEST);
    WriteReg(REG_LQI,VAL_LQI);
    WriteReg(REG_RSSI,VAL_RSSI);
    WriteReg(REG_MARCSTATE,VAL_MARCSTATE);
    WriteReg(REG_WORTIME1,VAL_WORTIME1);
    WriteReg(REG_WORTIME0,VAL_WORTIME0);
    WriteReg(REG_PKTSTATUS,VAL_PKTSTATUS);
    WriteReg(REG_VCO_VC_DAC,VAL_VCO_VC_DAC);
    WriteReg(REG_TXBYTES,VAL_TXBYTES);
    WriteReg(REG_RXBYTES,VAL_RXBYTES);
    WriteReg(REG_RCCTRL1_STATUS,VAL_RCCTRL1_STATUS);
    WriteReg(REG_RCCTRL0_STATUS,VAL_RCCTRL0_STATUS);
    */
}

void Read_Config_Regs(void)
{
  Serial.println(ReadReg(REG_IOCFG2), HEX);
  delay(10);
  Serial.println(ReadReg(REG_IOCFG1), HEX);
  delay(10);
  Serial.println(ReadReg(REG_IOCFG0), HEX);
  delay(10);
  /* Serial.println(ReadReg(REG_FIFOTHR),HEX);
     delay(10);
    Serial.println(ReadReg(REG_SYNC1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_SYNC0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_PKTLEN),HEX);
     delay(10);
    Serial.println(ReadReg(REG_PKTCTRL1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_PKTCTRL0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_ADDR),HEX);
     delay(10);
    Serial.println(ReadReg(REG_CHANNR),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCTRL1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCTRL0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FREQ2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FREQ1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FREQ0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MDMCFG4),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MDMCFG3),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MDMCFG2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MDMCFG1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MDMCFG0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_DEVIATN),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MCSM2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MCSM1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_MCSM0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FOCCFG),HEX);
     delay(10);

    Serial.println(ReadReg(REG_BSCFG),HEX);
     delay(10);
    Serial.println(ReadReg(REG_AGCCTRL2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_AGCCTRL1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_AGCCTRL0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_WOREVT1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_WOREVT0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_WORCTRL),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FREND1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FREND0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCAL3),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCAL2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCAL1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSCAL0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_RCCTRL1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_RCCTRL0),HEX);
     delay(10);
    Serial.println(ReadReg(REG_FSTEST),HEX);
     delay(10);
    Serial.println(ReadReg(REG_PTEST),HEX);
     delay(10);
    Serial.println(ReadReg(REG_AGCTEST),HEX);
     delay(10);
    Serial.println(ReadReg(REG_TEST2),HEX);
     delay(10);
    Serial.println(ReadReg(REG_TEST1),HEX);
     delay(10);
    Serial.println(ReadReg(REG_TEST0),HEX);
     delay(10);
   /*
    Serial.println(ReadReg(REG_PARTNUM),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_VERSION),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_FREQEST),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_LQI),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_RSSI),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_MARCSTATE),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_WORTIME1),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_WORTIME0),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_PKTSTATUS),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_VCO_VC_DAC),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_TXBYTES),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_RXBYTES),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_RCCTRL1_STATUS),HEX);
     delay(1000);
    Serial.println(ReadReg(REG_RCCTRL0_STATUS),HEX);
     delay(1000);
  */
}

void intToHex(int num, char hexString[])
{
  int index = 0;

  if (num == 0)
  {
    hexString[index++] = '0';
  }
  else
  {
    while (num != 0)
    {
      int remainder = num % 16;

      if (remainder < 10)
      {
        hexString[index++] = remainder + '0';
      }
      else
      {
        hexString[index++] = remainder - 10 + 'A';
      }

      num = num / 16;
    }
  }

  // Null-terminate the string
  hexString[index] = '\0';

  // Reverse the string
  int start = 0;
  int end = index - 1;
  while (start < end)
  {
    char temp = hexString[start];
    hexString[start] = hexString[end];
    hexString[end] = temp;
    start++;
    end--;
  }
}
