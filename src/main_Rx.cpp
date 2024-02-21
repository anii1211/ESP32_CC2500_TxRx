#include <Arduino.h>
#include <cc2500_REG.h>
#include <cc2500_VAL.h>
#include <HardwareSerial.h>



#include <SPI.h>

// Define direction bits
#define TOP_BIT    (1 << 0)
#define BOTTOM_BIT (1 << 5)
#define RIGHT_BIT  (1 << 2)
#define LEFT_BIT   (1 << 3)
#define STOP_BIT   (1 << 4)

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F

#define No_of_Bytes    2

#define UART_TX_PIN 17
#define UART_RX_PIN 16

const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status

const int GDO0_PIN = 4;     // the number of the GDO0_PIN pin
int GDO0_State = 0;         // variable for reading the pushbutton status
int led = 2;
int lastMillis = 0;
int rxFlag = 0;

void RxData_RF(void);
void WriteReg(char addr, char value);
char SendStrobe(char strobe);
void init_CC2500();
void Read_Config_Regs(void);
char ReadReg(char addr);
void parseDirectionData(uint8_t receivedData);

void IRAM_ATTR function_ISR() {
//  if(millis() - lastMillis &gt; 10){ // Software debouncing buton
         rxFlag = 1;
         Serial.printf("ISR triggered\r\n");
         buttonState = !buttonState;
         digitalWrite(led,buttonState);
//  }
//  lastMillis = millis();
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pinMode(SS,OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  SPI.begin();
  digitalWrite(SS,HIGH);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);     
  pinMode(GDO0_PIN, INPUT);         

  Serial.println("Starting..");
  init_CC2500();
  Read_Config_Regs();
  attachInterrupt(GDO0_PIN, function_ISR, CHANGE);
}

void loop()
{
 
  // Read_Config_Regs();
  /*
//  To start transmission
    buttonState = digitalRead(buttonPin);
    Serial.println(buttonState);

    while (!buttonState)
    {
        // read the state of the pushbutton value:
        buttonState = digitalRead(buttonPin);
        Serial.println("PB = 0");
    }
    
    Serial.println("BP = 1");
    */
    RxData_RF();

  
  /*
    while (buttonState)
    {
        // read the state of the pushbutton value:
        buttonState = digitalRead(buttonPin);
        Serial.println("PB = 1");
    }
    */
}


void RxData_RF(void) 
{
  
  
    int PacketLength = 0;
   // RX: enable RX
    SendStrobe(CC2500_RX);

  //   GDO0_State = digitalRead(GDO0_PIN);
  //  Serial.println("GDO0");
  //  Serial.println(GDO0_State);
 
    // // Wait for GDO0 to be set -> sync received
    // while (!rxFlag)
    // {
    //     // read the state of the GDO0_PIN value:
    //     Serial.println("GD0 = 0");
    //     delay(100);
    //     rxFlag = 0;
    // }
    // // Wait for GDO0 to be cleared -> end of packet
    // while (rxFlag)
    // {
    //   // read the state of the GDO0_PIN value:
    //     Serial.println("GD0 = 1");
    //     delay(100);
    // }
    
    // /*
    if(rxFlag )
    {
    char rxbytes = ReadReg(0x3B);
    if (rxbytes == 0x2)
    {
      Serial.println("---------------------");
      Serial.println("RX Bytes: ");
      Serial.println(rxbytes, HEX);
      Serial.println("---------------------");
    }

    // */
    
    char data1, data2, data3, data4, data5;
    uint8_t rxData = 0;
  // Read length byte
        PacketLength = ReadReg(CC2500_RXFIFO);
        if(PacketLength == 0x2)
        {
        Serial.println("---------------------");
          Serial.println(PacketLength,HEX);
          Serial.println(" Packet Received ");
        } 
        if(No_of_Bytes == PacketLength)
        {
          
          
          // Read data from RX FIFO and store in rxBuffer
          // data1 = ReadReg(CC2500_RXFIFO);
          // Serial.println(data1, HEX);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*top(36,34,32,45)
x = 4095 /100
Y = 1906 /46

Bottom (30,0,32,45)
X = 0 
Y = 1968 /46

Left(33,30,30,0)
X = 2018 /49
Y = 0

Right (33,31,36,34)
X = 2015 /44
Y = 4095 /100

Center
X = 49
Y = 46*/
////////////////////////////////////////////////////////////////////////////////////////////////////////

          rxData = ReadReg(CC2500_RXFIFO);
          // Parse received data
          
          parseDirectionData(rxData);
          
          // data2 = ReadReg(CC2500_RXFIFO);
          // // Serial.println(data2, HEX);


          // data3 = ReadReg(CC2500_RXFIFO);
          // // Serial.println(data3, HEX);
         
          // data4 = ReadReg(CC2500_RXFIFO);
          // // Serial.println(data4, HEX);
        

          // data5 = ReadReg(CC2500_RXFIFO);
          // // Serial.println(data5, HEX);

        // if(data2 == 0x1)
        // {
        //   Serial.println("*****TOP DIRECTION*****");
        //   Serial2.write(0x1);
        // }
        // else if(data3 == 0x1)
        // {
        //   Serial.println("*****BOTTOM DIRECTION*****");
        //   Serial2.write(0x2);
        // }
        // else if(data4 == 0x1)
        // {
        //   Serial.println("*****LEFT DIRECTION*****");
        //   Serial2.write(0x3);
        // }
        // else if(data5 == 0x1)
        // {
        //   Serial.println("*****RIGHT DIRECTION*****");
        //   Serial2.write(0x4);
        // }

        // if(((data2 >= 0x33 && data2 <= 0x38) && (data3 >= 0x32 && data3 <= 0x36) && (data4 >= 0x30 && data4 <= 0x35) && (data5 >= 0x38 && data5 <= 0x47)))
        // {
        //   Serial.println("*****TOP DIRECTION*****");
        //   Serial.print("*****TOP DIRECTION*****");
        // }
        // else if(((data2 >= 0x28 && data2 <= 0x33) && (data3 >= 0x0 && data3 <= 0x10) && (data4 >= 0x30 && data4 <= 0x35) && (data5 >= 0x38 && data5 <= 0x47)))
        // {
        //   Serial.println("*****BOTTOM DIRECTION*****");
        // }
        // else if(((data2 >= 0x30 && data2 <= 0x36) && (data3 >= 0x28 && data3 <= 0x34) && (data4 >= 0x28 && data4 <= 0x34) && (data5 >= 0x0 && data5 <= 0x10)))
        // {
        //   Serial.println("*****LEFT DIRECTION*****");
        // }
        // else if(((data2 >= 30 && data2 <= 36) && (data3 >= 28 && data3 <= 34) && (data4 >= 33 && data4 <= 38) && (data5 >= 32 && data5 <= 36)))
        // {
        //   Serial.println("******RIGHT DIRECTION*****");
        // }
//  Serial.println("send data UART");
          // Serial2.write(data2);
          // Serial2.write(data3);
          // Serial2.write(data4);
          // Serial2.write(data5);

          // data5 = ReadReg(CC2500_RXFIFO);
          // Serial.println(data5, HEX);
          // for(int i = 1; i < PacketLength; i++)
          // {            
          //   data1 = ReadReg(CC2500_RXFIFO);
          //   Serial.println(data1,HEX);
          //   if(data1 = 0x09){
          //       data2 = ReadReg(CC2500_RXFIFO);
          //       Serial.println(data2,HEX);
          //       if(data2 == 0x01 ){
          //           digitalWrite(led, LOW);
          //       }
          //       else{
          //           digitalWrite(led, HIGH);
          //       }
          //   }
          //   // Serial.println(ReadReg(CC2500_RXFIFO), HEX);
          // }
          Serial.println("\r");
          Serial.println("----------***********-----------\r");
          // Serial.print("RSSI: \r");
          // rxData = ReadReg(REG_RSSI);
          // rxData = ReadReg(0X34);
          uint8_t rssi = 0;
          rssi = ReadReg(CC2500_RXFIFO);
          // Serial.printf("RSSI: %x\n\r", rssi);

          int dbm;
          if (rssi < 0x7F)
          {
            dbm = (rssi / 2) - 72;
          }
          else
          {
            dbm = abs(rssi - 256);
            dbm = (dbm / 2) - 72;
          }

          Serial.print("RSSI(Raw): 0x\r");
          Serial.print(rssi, HEX);
          Serial.print(" RSSI(dBm): \r");
          Serial.print(dbm);
          // Serial.println(ReadReg(CC2500_RXFIFO), HEX);
          Serial.println("----------***********-----------\r");
          Serial.println("\r");
        }
       
        // Make sure that the radio is in IDLE state before flushing the FIFO
        // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
        SendStrobe(CC2500_IDLE);
        // Flush RX FIFO
        SendStrobe(CC2500_FRX);

        rxFlag = 0;
    }
}// Rf RxPacket

// Function to parse received data and extract direction information
void parseDirectionData(uint8_t receivedData)
{
  // Check each bit to determine the direction
  Serial.printf("receive Data: %x\n\r", receivedData);
  if (receivedData & TOP_BIT)
  {
    rxFlag = 0;
    Serial2.write(0x1);
    Serial.print("Direction: Top\n\r");
  }
  else if (receivedData & BOTTOM_BIT)
  {
    rxFlag = 0;
    Serial2.write(0x2);
    Serial.print("Direction: Bottom\n\r");
  }
  else if (receivedData & RIGHT_BIT)
  {
    rxFlag = 0;
    Serial2.write(0x3);
    Serial.print("Direction: Right\n\r");
  }
  else if (receivedData & LEFT_BIT)
  {
    rxFlag = 0;
    Serial2.write(0x4);
    Serial.print("Direction: Left\n\r");
  }
  else if (receivedData & STOP_BIT)
  {
    rxFlag = 0;
    Serial2.write(0x5);
    Serial.print("Direction: Stop\n\r");
  }
  else
  {
    rxFlag = 0;
    Serial.print("Unknown direction\n");
  }

  receivedData = 0;
}

void WriteReg(char addr, char value)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  SPI.transfer(addr);
  delay(10);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr)
{
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}


void init_CC2500()
{
  WriteReg(REG_IOCFG2,VAL_IOCFG2);
  WriteReg(REG_IOCFG1,VAL_IOCFG1);
  WriteReg(REG_IOCFG0,VAL_IOCFG0);

  WriteReg(REG_FIFOTHR,VAL_FIFOTHR);
  WriteReg(REG_SYNC1,VAL_SYNC1);
  WriteReg(REG_SYNC0,VAL_SYNC0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);

  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_PTEST,VAL_PTEST);
  WriteReg(REG_AGCTEST,VAL_AGCTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
  WriteReg(REG_RSSI,VAL_RSSI);
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
  Serial.println("Configuration registers");
  Serial.println(ReadReg(REG_IOCFG2),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
   delay(1000);
   
/* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(1000);
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
//  /*
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
// */  
}

