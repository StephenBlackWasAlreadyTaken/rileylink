/* Minimed RF communications */

/* Note: must issue software reset (command = 4) before transferring data
 * to make sure spi is synced */

#include "minimed_rf.h"

#ifdef __GNUC__
#define XDATA(x)
#else
#define XDATA(x) __xdata __at x
#endif

unsigned char lastCmd = CMD_NOP;

// SPI Mode
#define SPI_MODE_CMD  0
#define SPI_MODE_ARG  1
#define SPI_MODE_READ 2
/*unsigned char spiMode = SPI_MODE_CMD;*/
unsigned char spiMode = SPI_MODE_READ;

// Errors
#define ERROR_DATA_BUFFER_OVERFLOW 0x50
#define ERROR_TOO_MANY_PACKETS 0x51
#define ERROR_RF_TX_OVERFLOW 0x52

// Resource usage defines
#define BUFFER_SIZE 1024
#define MAX_PACKETS 100
#define MAX_PACKET_SIZE 250

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define TRUE 1
#define FALSE 0

#define BIT0 0x1
#define BIT1 0x2
#define BIT2 0x4
#define BIT3 0x8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

// Radio mode
unsigned char radioMode = RADIO_MODE_RX;

// Data buffer
int bufferWritePos = 0;
int bufferReadPos = 0;
unsigned int dataBufferBytesUsed = 0;

unsigned char packetNumber = 0;

// Packet
typedef struct Packet {
  int dataStartIdx;
  unsigned char length;
  unsigned char rssi;
  unsigned char packetNumber;
} Packet;


int packetCount = 0;
int packetHeadIdx = 0;
int packetTailIdx = 0;

// Packet sending counters
int currentPacketByteIdx = 0;
int currentPacketBytesRemaining = 0;
int crcErrorCount = 0;

unsigned char lastError = 0;
unsigned char sendingPacket = FALSE;
unsigned char packetOverflowCount = 0;
unsigned char bufferOverflowCount = 0;

int symbolInputBitCount = 0;
int symbolOutputBuffer = 0;
int symbolOutputBitCount = 0;
int symbolErrorCount = 0;

// Packet transmitting
int radioOutputBufferWritePos = 0;
int radioOutputBufferReadPos = 0;
int radioOutputDataLength = 0;

// 1024 bytes (0xfb00 - 0xff00)
unsigned char XDATA(0xfb00) dataBuffer[BUFFER_SIZE]; // RF Input buffer

// 100 * 5 bytes = 500 bytes
Packet XDATA(0xf7a8) packets[MAX_PACKETS];

// 256 bytes
unsigned char XDATA(0xf6a8) crcTable[256];

// 256 bytes
unsigned char XDATA(0xf5a8) radioOutputBuffer[256];

// Symbol decoding - 53 bytes + 1 pad 
unsigned char XDATA(0xf572) symbolTable[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 11, 16, 13, 14, 16, 16, 16, 16, 16, 16, 0, 7, 16, 16, 9, 8, 16, 15, 16, 16, 16, 16, 16, 16, 3, 16, 5, 6, 16, 16, 16, 10, 16, 12, 16, 16, 16, 16, 1, 2, 16, 4};




#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long
#define uint8 unsigned char
#define uint8_t unsigned char
#define uint16 unsigned int
#define uint16_t unsigned int
#define uint32 unsigned long
#define uchar unsigned char
#define xdata __xdata
#define _PRAGMA(x) _Pragma(#x)
#define interrupt __interrupt

#define UART1_TX_ISR_VECTOR 0x73
#define UART1_RX_ISR_VECTOR 0x1B

#define SIZE_OF_UART_RX_BUFFER 256
#define SIZE_OF_UART_TX_BUFFER 256

#define UART_BAUD_M 131
#define UART_BAUD_E 8

/*typedef struct IncomingUartPayload {*/
  /*int dataStartIdx;*/
  /*unsigned char length;*/
  /*unsigned char packetNumber;*/
/*}*/
/*typedef struct IncomingUartMessage {*/
  /*int dataStartIdx;*/
  /*int id;*/
  /*unsigned char length;*/
  /*unsigned char packetNumber;*/
/*}*/
unsigned char __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
unsigned char __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
unsigned short __xdata uartRxIndex, uartTxIndex;
unsigned short __xdata uartRxInterruptIndex, uartTxInterruptIndex;
/*IncomingUartPayload __xdata incomingUartPayloads[MAX_PACKETS];*/
/*IncomingUartMessage __xdata incomingUartMessages[MAX_PACKETS];*/
unsigned char currentIncomingMessage;
unsigned char currentIncomingPayload;
unsigned char lastProcessedPayload;

void uartMapPort(unsigned char uartPortAlt) {
    if(uartPortAlt == 1) {
        PERCFG &= ~0x02; //rx = 5, tx = 4
        P0SEL |= 0x3C;
        P1SEL &= ~0xF0;
    } else {
        PERCFG |= 0x02; //rx = 7, tx = 6
        P1SEL |= 0xF0;
        P0SEL &= ~0x3C;
    }
}


void setBaudUartBaud() { // 9600, to adjust just change UART_BAUD_M and UART_BAUD_E
    /*CLKCON &= 0x80;*/
    /*while(CLKCON & 0x40);*/
    /*SLEEP |= 0x04;*/
    U0BAUD = UART_BAUD_M;
    U0GCR = (U0GCR&~0x1F) | UART_BAUD_E;
}

typedef struct {
    uint8 START : 1; // Start bit level (low/high)
    // Start bit level = low => Idle level = high (U0UCR.START = 0)
    // Start bit level = high => Idle level = low (U0UCR.START = 1)
    uint8 STOP : 1; // Stop bit level (low/high)
    // Stop bit level = high (U0UCR.STOP = 1)
    // Stop bit level = low (U0UCR.STOP = 0)
    uint8 SPB : 1; // Stop bits (0 => 1, 1 => 2)
    // Number of stop bits = 1 (U0UCR.SPB = 0)
    // Number of stop bits = 2 (U0UCR.SPB = 1)
    uint8 PARITY : 1; // Parity control (enable/disable)
    // Parity = disabled (U0UCR.PARITY = 0)
    // Parity = enabled (U0UCR.PARITY = 1)
    uint8 BIT9 : 1; // 9 bit enable (8bit / 9bit)
    // 9-bit data disable = 8 bits transfer (U0UCR.BIT9 = 0)
    // 9-bit data enable = 9 bits transfer (U0UCR.BIT9 = 1)
    uint8 D9 : 1; // 9th bit level or Parity type
    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
    uint8 FLOW : 1; // HW Flow Control (enable/disable)
    // Flow control = disabled (U0UCR.FLOW = 0)
    // Flow control = enabled (U0UCR.FLOW = 1)
    uint8 ORDER : 1; // Data bit order(LSB/MSB first)
    // Bit order = MSB first (U0GCR.ORDER = 1)
    // Bit order = LSB first (U0GCR.ORDER = 0) => For PC/Hyperterminal
} UART_PROT_CONFIG;
UART_PROT_CONFIG __xdata defaultUartProtConfig;

void uartSetStart(uint8 start) {
    defaultUartProtConfig.START = start;
}
void uartSetStop(uint8 stop) {
    defaultUartProtConfig.STOP = stop;
}
void uartSetSbp(uint8 sbp) {
    defaultUartProtConfig.SPB = sbp;
}
void uartSetParity(uint8 parity) {
    defaultUartProtConfig.PARITY = parity;
}
void uartSetBit9(uint8 bit9) {
    defaultUartProtConfig.BIT9 = bit9;
}
void uartSetD9(uint8 d9) {
    defaultUartProtConfig.D9 = d9;
}
void uartSetFlow(uint8 flow) {
    defaultUartProtConfig.FLOW = flow;
}
void uartSetOrder(uint8 order) {
    defaultUartProtConfig.ORDER = order;
}

void uartInitProtocol(UART_PROT_CONFIG* uartProtConfig) {
    U1CSR |= 0x80;
    U1UCR = (U1UCR&~0x01) | uartProtConfig->START;
    U1UCR = (U1UCR&~0x02) | (uartProtConfig->STOP << 1);
    U1UCR = (U1UCR&~0x04) | (uartProtConfig->SPB << 2);
    U1UCR = (U1UCR&~0x08) | (uartProtConfig->PARITY << 3);
    U1UCR = (U1UCR&~0x10) | (uartProtConfig->BIT9 << 4);
    U1UCR = (U1UCR&~0x20) | (uartProtConfig->D9 << 5);
    U1UCR = (U1UCR&~0x40) | (uartProtConfig->FLOW << 6);
    U1GCR = (U1GCR&~0x20) | (uartProtConfig->ORDER << 5);
}

void uartStartTxForIsr() {
    uartTxIndex = 1;
    UTX1IF = 0;
    IEN2 |= 0x08;
    U1DBUF = uartTxBuffer[0];
    IEN0 |= 0x80;
}

void UART1_TX_ISR( void ) __interrupt( UART1_TX_ISR_VECTOR ) {
    UTX1IF = 0;
    if (uartTxIndex > uartTxInterruptIndex) {
        uartTxIndex = 0;
        uartTxInterruptIndex = 0;
        IEN2 &= ~0x08;
        return;
    }
    U1DBUF = uartTxBuffer[uartTxIndex++];
}

void uartStartRxForIsr() {
    uartRxIndex = 0;
    uartRxInterruptIndex = 0;
    URX1IF = 0;
    U1CSR |= 0x40;
    IEN0 |= 0x08;
    IEN0 |= 0x80;
}


/*unsigned char __xdata messageBuffer[256];*/
/*unsigned char __xdata payloadBuffer[256];*/
/*unsigned char __xdata waitingForNewPacket = 1;*/
/*unsigned char __xdata currentPayloadIdentifier;*/

void UART1_RX_ISR( void ) __interrupt( UART1_RX_ISR_VECTOR ) {
    //Payload is comprised of multiple messages
    //Payloads are created once messages are completely received
    //Acks need to be sent before a next message can be received
    //a message is considered complete when 20 bytes or a full message queue is built 
    //    (when payload size == difference between message buffer index and current message buffer index)
    
    /*URX1IF = 0;*/
    /*if(waitingForNewPacket) {*/
        /*currentIncomingPayload = U1DBUF;*/
        /*Payloads[currentIncomingPayload].dataStartIdx = currentPayloadWriteIndex;*/
        /*Payloads[currentIncomingPayload].size = 0;*/
        /*Payloads[currentIncomingPayload].messagesSize = 0;*/
        /*waitingForNewPacket = 0;*/
    /*} else if(!waitingForNewPacket) {*/

    /*}*/

        /*IncomingUartPayload payload;*/
        /*payload.dataStartIdx = currentPayloadWriteIndex++;*/
        /*IncomingUartMessage message;*/
        /*message.dataStartIdx = currentMessageWriteIndex++;*/
        /*message.id = U1DBUF;*/
        
    /*}*/
    uartRxBuffer[uartRxIndex++] = U1DBUF;
    if (uartRxIndex >= SIZE_OF_UART_RX_BUFFER) {
        uartRxIndex = 0; IEN0 &= ~0x08;
    }
}

uint8 uartTxAvailable() {
    if(uartTxInterruptIndex) {
        return 1;
    } else {
        return 0;
    }
}

void uartTx(uint8 __xdata buffer[], uint8 size){
    uint8 position = 0;
    uartTxInterruptIndex = size - 1;
    while (position < size) {
        uartTxBuffer[position] = buffer[position];
        position++;
    }
    uartStartTxForIsr();
}

void uartTxSendByte(uint8 byte) {
    uartTxInterruptIndex = 0;
    uartTxBuffer[0] = byte;
    uartStartTxForIsr();
}

uint8 uartNRxReceiveByte() {
    uint8 byte = uartRxBuffer[uartRxIndex];
    uartRxIndex++;
    if(uartRxIndex > uartRxInterruptIndex){
        uartRxInterruptIndex = 0;
        uartRxIndex = 0;
    }
    return byte;
}

void uartInit() {
    setBaudUartBaud();
    uartMapPort(2); // specify alt pin location (1 or 2)
    uartInitProtocol(&defaultUartProtConfig);
    /*uartStartRxForIsr();*/
}

void initUart() {
    uartSetStart(0);
    uartSetStop(1);
    uartSetSbp(0);
    uartSetParity(0);
    uartSetBit9(0);
    uartSetD9(0);
    uartSetFlow(0);
    uartSetOrder(1);
    uartInit();
}


//RF RELATED
int symbolInputBuffer = 0;
void initMinimedRF() {

  // init crc table
  unsigned char polynomial = 0x9b;
  const unsigned char msbit = 0x80;
  int i, j;
  unsigned char t = 1;

  crcTable[0] = 0;

  // msb
  t = msbit;
  for (i = 1; i < 256; i *= 2) {
    t = (t << 1) ^ (t & msbit ? polynomial : 0);
    for (j = 0; j < i; j++) {
      //printf("i = %d, j = %d, t = %d\n", i, j, t);
      crcTable[i+j] = crcTable[j] ^ t;
    }
  }

  // Initialize first packet
  packets[0].dataStartIdx = 0;
  packets[0].length = 0;
  setChannel(2);
}

unsigned char cmdGetByte() {
  Packet *packet;
  unsigned char rval = 0;
  if (packetCount > 0)
  {
    packet = &packets[packetTailIdx];
    if (!sendingPacket) {
      bufferReadPos = packet->dataStartIdx;
      currentPacketBytesRemaining = packet->length;
      sendingPacket = TRUE;
    }
    if (currentPacketBytesRemaining > 0) {
      rval = dataBuffer[bufferReadPos++];
      currentPacketBytesRemaining--;
      if (currentPacketBytesRemaining == 0) {
        // Done sending packet
        sendingPacket = FALSE;
        packetCount--;
        packetTailIdx++;
        if (packetTailIdx == MAX_PACKETS) {
          packetTailIdx = 0;
        }
      }
      if (bufferReadPos == BUFFER_SIZE) {
        bufferReadPos = 0;
      }
      dataBufferBytesUsed--;
    } else {
      rval = 0x88;
    }
  } else {
    // Request to get packet data, when there are no packets available!
    rval = 0x99;
  }
  return rval;
}

void sendPacketUart() {
  Packet *packet;
  unsigned char rval = 0;
  if (packetCount > 0)
  {
    packet = &packets[packetTailIdx];
    if (!sendingPacket) {
      bufferReadPos = packet->dataStartIdx;
      currentPacketBytesRemaining = packet->length;
      sendingPacket = TRUE;
    }
    while(currentPacketBytesRemaining > 0) {
      uartTxSendByte(dataBuffer[bufferReadPos++]);
      currentPacketBytesRemaining--;
      if (currentPacketBytesRemaining == 0) {
        // Done sending packet
        sendingPacket = FALSE;
        packetCount--;
        packetTailIdx++;
        if (packetTailIdx == MAX_PACKETS) {
          packetTailIdx = 0;
        }
      }
      if (bufferReadPos == BUFFER_SIZE) {
        bufferReadPos = 0;
      }
      dataBufferBytesUsed--;
    }
  }
}

void doCommand(unsigned char cmd) {
  lastCmd = cmd;
  switch (cmd) {
  case CMD_GET_CHANNEL:
    U1DBUF = CHANNR;
    break;
  case CMD_SET_CHANNEL:
  case CMD_SEND_PACKET:
    spiMode = SPI_MODE_ARG;
    break;
  case CMD_GET_LENGTH:
    //P1_1 = packetHeadIdx == packetTailIdx ? 0 : 1;
    //P0_0 = packets[packetTailIdx].length > 0 ? 1 : 0; // low
    //P0_1 = packetCount > 0 ? 1 : 0;                   // high
    if (packetCount > 0) {
      U1DBUF = packets[packetTailIdx].length;
    } else {
      U1DBUF = 0;
    }
    break;
  case CMD_GET_RSSI:
    U1DBUF = packets[packetTailIdx].rssi;
    break;
  case CMD_GET_PACKET_NUMBER:
    U1DBUF = packets[packetTailIdx].packetNumber;
    break;
  case CMD_GET_BYTE:
    U1DBUF = cmdGetByte();
    break;
  case CMD_RESET:
    P1_1 = 1; // Red
    EA = 0;
    WDCTL = BIT3 | BIT0;
    break;
  case CMD_GET_ERROR:
    U1DBUF = lastError;
    lastError = 0;
    break;
  case CMD_GET_RADIO_MODE:
    U1DBUF = radioMode;
    break;
  case CMD_GET_PACKET_COUNT:
    U1DBUF = packetCount; 
    break;
  case CMD_GET_PACKET_HEAD_IDX:
    U1DBUF = packetHeadIdx;
    break;
  case CMD_GET_PACKET_TAIL_IDX:
    U1DBUF = packetTailIdx;
    break;
  case CMD_GET_PACKET_OVERFLOW_COUNT:
    U1DBUF = packetOverflowCount;
    break;
  case CMD_GET_BUFFER_OVERFLOW_COUNT:
    U1DBUF = bufferOverflowCount;
    break;

  default:
    U1DBUF = 0x22;
    break;
  }
}

void setChannel(unsigned char newChannel) {
  // Guard against using remote channel
  if (newChannel != 4) {
    CHANNR = newChannel;
    RFTXRXIE = 0;
  }
}


void handleRX1() {
  unsigned char value;
  if (spiMode == SPI_MODE_CMD) {
    doCommand(U1DBUF);
  } else if (spiMode == SPI_MODE_ARG) {
    value = U1DBUF;
    switch (lastCmd) {
    case CMD_SET_CHANNEL:
      setChannel(value);
      break;
    case CMD_SEND_PACKET:
      radioOutputDataLength = value;
      spiMode = SPI_MODE_READ;
      break;
    }
    spiMode = SPI_MODE_CMD;
  } else if (spiMode == SPI_MODE_READ) {
    radioOutputBuffer[radioOutputBufferWritePos++] = U1DBUF;
    if (radioOutputBufferWritePos == radioOutputDataLength) {
      radioOutputBufferReadPos = 0;
       /*Set radio mode to tx;*/
      /*radioMode = RADIO_MODE_TX;*/
      RFTXRXIE = 0;
    }
  }
}

void dropCurrentPacket() {
  bufferWritePos = packets[packetHeadIdx].dataStartIdx;
  dataBufferBytesUsed -= packets[packetHeadIdx].length;
  packets[packetHeadIdx].length = 0;
  // Disable RFTXRX interrupt, which signals main loop to restart radio.
  RFTXRXIE = 0;
}

void addDecodedByte(unsigned char value) {
  if (dataBufferBytesUsed < BUFFER_SIZE) {
    dataBuffer[bufferWritePos] = value;
    bufferWritePos++;
    dataBufferBytesUsed++;
    packets[packetHeadIdx].length++;
    if (bufferWritePos == BUFFER_SIZE) {
      bufferWritePos = 0;
    }
    if (packets[packetHeadIdx].length >= MAX_PACKET_SIZE) {
      dropCurrentPacket();
    }
  } else {
    bufferOverflowCount++;
    dropCurrentPacket();
  }
}

void finishIncomingPacket() {
  // Compute crc
  
  unsigned int packetCrc;
  unsigned char crc = 0;

  int crcReadIdx = packets[packetHeadIdx].dataStartIdx;
  int crcLen = packets[packetHeadIdx].length-1;

  /* Assign rssi */
  packets[packetHeadIdx].rssi = RSSI;

  /* Assign packet number */
  packets[packetHeadIdx].packetNumber = packetNumber;
  packetNumber++;

  /* loop over the buffer data */
  while (crcLen-- > 0) {
    crc = crcTable[(crc ^ dataBuffer[crcReadIdx]) & 0xff];
    crcReadIdx++;
    if (crcReadIdx >= BUFFER_SIZE) {
      crcReadIdx = 0;
    }
  }
  packetCrc = dataBuffer[crcReadIdx];

  if (packetCount+1 == MAX_PACKETS) {
    // Packet count overflow
    lastError = ERROR_TOO_MANY_PACKETS;
    // Reuse this packet's space.
    packetOverflowCount++;
    dropCurrentPacket();
//  } else if (packetCrc != crc) {
//    //printf("invalid crc\n");
//    crcErrorCount++;
//    // Drop this packet
//    dropCurrentPacket();
  } else {
    P0_1 = !P0_1;
    //printf("valid crc\n");
    if (packets[packetHeadIdx].length == 0) {
    }
    packetCount++;
    packetHeadIdx++;
    if (packetHeadIdx == MAX_PACKETS) {
      packetHeadIdx = 0;
    }
    packets[packetHeadIdx].dataStartIdx = bufferWritePos;
    packets[packetHeadIdx].length = 0;

//TODO: get rid of this, this is just a test!
    sendPacketUart();
  }

  // Reset symbol processing state
  symbolInputBuffer = 0;
  symbolInputBitCount = 0;
  symbolOutputBuffer = 0;
  symbolOutputBitCount = 0;
  symbolErrorCount = 0;

  // Disable RFTXRX interrupt, which signals main loop to restart radio.
  RFTXRXIE = 0;
}

void receiveRadioSymbol(unsigned char value) {

  unsigned char symbol;
  unsigned char outputSymbol;
  //printf("receiveRadioSymbol %d\n", value);
  uartTxSendByte(value);
  if (value == 0) {
    if (packets[packetHeadIdx].length > 0) {
      finishIncomingPacket();
    }
    return;
  }

  symbolInputBuffer = (symbolInputBuffer << 8) + value;
  symbolInputBitCount += 8;
  while (symbolInputBitCount >= 6) {
    symbol = (symbolInputBuffer >> (symbolInputBitCount-6)) & 0b111111;
    symbolInputBitCount -= 6;
    if (symbol == 0) {
      continue;
    }
    if (symbol >= sizeof(symbolTable) ||
        (symbol = symbolTable[symbol]) == 16) {
      symbolErrorCount++;
      break;
    } else {
      symbolOutputBuffer = (symbolOutputBuffer << 4) + symbol;
    }
    symbolOutputBitCount += 4;
  }
  while (symbolOutputBitCount >= 8) {
    outputSymbol = (symbolOutputBuffer >> (symbolOutputBitCount-8)) & 0b11111111;
    symbolOutputBitCount-=8;
    addDecodedByte(outputSymbol);
  }
  if (symbolErrorCount > 0 && packets[packetHeadIdx].length > 0) {
    finishIncomingPacket();
  }
}

void handleRFTXRX() {
  switch (MARCSTATE) {
  case MARC_STATE_RX:
    receiveRadioSymbol(RFD);
    break;
  case MARC_STATE_TX:
    RFD = radioOutputBuffer[radioOutputBufferReadPos++];
    if (radioOutputBufferReadPos == radioOutputDataLength) {
      radioOutputBufferWritePos = 0;
      radioOutputBufferReadPos = 0;
      radioOutputDataLength = 0;
      /*radioMode = RADIO_MODE_IDLE;*/
      RFTXRXIE = 0;
    }
    break;
  }
  uartTxSendByte(1);
}

void handleRF()
{
  S1CON &= ~0x03; // Clear CPU interrupt flag
  if(RFIF & 0x80) // TX underflow
  {
    //irq_txunf(); // Handle TX underflow
    RFIF &= ~0x80; // Clear module interrupt flag
  }
  else if(RFIF & 0x40) // RX overflow
  {
    //irq_rxovf(); // Handle RX overflow
    RFIF &= ~0x40; // Clear module interrupt flag
  }
  else if(RFIF & 0x20) // RX timeout
  {
    RFIF &= ~0x20; // Clear module interrupt flag
  }
  // Use ”else if” to check and handle other RFIF flags
}
