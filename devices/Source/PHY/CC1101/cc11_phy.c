/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// CC11xx RF interface

#include "../../config.h"

#ifdef CC11_PHY

//#define CC11_ANAREN

#include "cc11_reg.h"

#ifndef CC11_ANAREN     // Fosc = 26M

// 433 MHz
#if (RF_BASE_FREQ > 433050000UL) && (RF_BASE_FREQ < 434790000UL)
#define CC11_DEFVAL_FREQ2       0x10
#define CC11_DEFVAL_FREQ1       0xA7
#define CC11_DEFVAL_FREQ0       0x62
// 868 MHz
#elif (RF_BASE_FREQ > 868000000UL) && (RF_BASE_FREQ < 870000000UL)
#define CC11_DEFVAL_FREQ2       0x21
#define CC11_DEFVAL_FREQ1       0x62
#define CC11_DEFVAL_FREQ0       0x76
#endif  // RF_BASE_FREQ

#define CC11_MDMCFG3_VAL        0x83    // Data Rate = 38,383 kBaud
#define CC11_DEVIATN_VAL        0x33    // Deviation 17,46 kHz

#else   // Fosc = 27M

// 433 MHz
#if (RF_BASE_FREQ > 433050000UL) && (RF_BASE_FREQ < 434790000UL)
#define CC11_DEFVAL_FREQ2       0x10
#define CC11_DEFVAL_FREQ1       0x09
#define CC11_DEFVAL_FREQ0       0x7B
// 868 MHz
#elif (RF_BASE_FREQ > 868000000UL) && (RF_BASE_FREQ < 870000000UL)
#define CC11_DEFVAL_FREQ2       0x20
#define CC11_DEFVAL_FREQ1       0x25
#define CC11_DEFVAL_FREQ0       0xED
#endif  //  RF_BASE_FREQ

#define CC11_MDMCFG3_VAL        0x75    // Data Rate = 38,4178 kBaud
#define CC11_DEVIATN_VAL        0x32    // Deviation 16,5 kHz

#endif  //  CC11_ANAREN

#if (CC11_PHY == 1)
#define cc11_adr                phy1addr
#define CC11_RF_POWER           0x50
#elif (CC11_PHY == 2)
#define cc11_adr                phy2addr
#define CC11_RF_POWER           0xC0
#endif

#ifndef RF_WAIT_LOW_MISO
#define RF_WAIT_LOW_MISO()      while(RF_PIN & (1<<RF_PIN_MISO))
#endif  //  RF_WAIT_LOW_MISO

#ifndef RF_SELECT
#define RF_SELECT()             RF_PORT &= ~(1<<RF_PIN_SS)
#define RF_RELEASE()            RF_PORT |= (1<<RF_PIN_SS)
#endif  //  RF_SELECT()

#ifndef RxLEDon
#define RxLEDon()
#endif  //  RxLEDon

typedef enum
{
    CC11_STATE_IDLE = 0,

    CC11_STATE_RXIDLE,
    CC11_STATE_RXDATA,
  
    CC11_STATE_TXHDR,
    CC11_STATE_TXDATA
}CC11_STATE_e;

typedef struct
{
    uint8_t     Length;
    uint8_t     SrcAddr;
    uint8_t     RSSI;
    uint8_t     LQI;
    uint8_t     data[MQTTSN_MSG_SIZE];
}CC11_RX_DATA_t;

static const uint8_t cc11config[][2] =
{
#ifndef CC11_ANAREN
  {CC11_IOCFG2,   CC11_GDO_DISABLE},    // GDO2 - High impedance (3-State)
  {CC11_IOCFG0,   CC11_GDO_SYNC},       // GDO0 - Asserts when sync word has been sent/received, and de-asserts at the end of the packet.
#else
  {CC11_IOCFG2,   CC11_GDO_SYNC},       // GDO2 - High impedance (3-State)
  {CC11_IOCFG0,   CC11_GDO_DISABLE},    // GDO0 - Asserts when sync word has been sent/received, and de-asserts at the end of the packet.
#endif
  {CC11_FIFOTHR,  0x47},                // ADC_RETENTION, RX Attenuation: 0 dB, FIFO Threshold 33/32 bytes 
  {CC11_PKTLEN,   0x3D},                // default packet length 61 byte
  {CC11_PKTCTRL1, 0x06},                // Append Status, Check Address and Broadcast
  {CC11_PKTCTRL0, 0x05},                // CRC calculation: enable, variable packet length
  {CC11_FSCTRL1,  0x08},                // IF = 100 kHz
  {CC11_FREQ2,    CC11_DEFVAL_FREQ2},   // Set default carrier frequency
  {CC11_FREQ1,    CC11_DEFVAL_FREQ1},
  {CC11_FREQ0,    CC11_DEFVAL_FREQ0},
  {CC11_MDMCFG4,  0xCA},                // RX filter BW 101,6 kHz
  {CC11_MDMCFG3,  CC11_MDMCFG3_VAL},    // Data Rate
  {CC11_MDMCFG2,  0x93},                // Current optimized, GFSK, sync word 30/32 bit detected
  {CC11_MDMCFG1,  0x00},                // Channel spacing 25 kHz
  {CC11_MDMCFG0,  0x00},
  {CC11_DEVIATN,  CC11_DEVIATN_VAL},    // Deviation 20 kHz
  {CC11_MCSM0,    0x18},                // Automatically calibrate when going from IDLE to RX or TX,  PO_TIMEOUT: 150uS
  {CC11_FOCCFG,   0x16},                // Frequency offset compensation 67,5 kHz
  {CC11_AGCCTRL2, 0x43},                // The highest gain setting can not be used, Target amplitude from channel filter: 33 dB 
  {CC11_WORCTRL,  0xFB},
  {CC11_FSCAL3,   0xE9},
  {CC11_FSCAL2,   0x2A},
  {CC11_FSCAL1,   0x00},
  {CC11_FSCAL0,   0x1F},
  {CC11_TEST2,    0x81},
  {CC11_TEST1,    0x35},
  {CC11_TEST0,    0x09}
};

static uint8_t          cc11s_NodeID;
static CC11_RX_DATA_t   cc11_rx_data = {0, 0, 0, 0, {0,}};
static Queue_t          cc11_tx_queue = {NULL, NULL};

static volatile CC11_STATE_e cc11v_State = CC11_STATE_IDLE;

extern void     hal_cc11_init_hw(void);
extern uint8_t  hal_cc11_spiExch(uint8_t data);

// Send command strobe to the CC1101 IC via SPI
static void cc11_cmdStrobe(uint8_t cmd) 
{
    RF_SELECT();                        // Select CC1101
    RF_WAIT_LOW_MISO();                 // Wait until MISO goes low
    hal_cc11_spiExch(cmd);
    RF_RELEASE();                       // Release CC1101
}

// Write single register into the CC1101 IC via SPI
static void cc11_writeReg(uint8_t Addr, uint8_t value) 
{
    RF_SELECT();                        // Select CC1101
    RF_WAIT_LOW_MISO();                 // Wait until MISO goes low
    hal_cc11_spiExch(Addr);             // Send register address
    hal_cc11_spiExch(value);            // Send value
    RF_RELEASE();                       // Release CC1101
}

// Read single CC1101 register via SPI
static uint8_t cc11_readReg(uint8_t Addr)
{
    uint8_t retval;

    RF_SELECT();                        // Select CC1101
    RF_WAIT_LOW_MISO();                 // Wait until MISO goes low
    hal_cc11_spiExch(Addr);             // Send register address
    // Read result
    retval = hal_cc11_spiExch(0);
    RF_RELEASE();                       // Release CC1101
    return retval;
}

static void cc11_tx_task(void)
{
    if((cc11_tx_queue.pTail != NULL) && 
      ((cc11_readReg(CC11_PKTSTATUS | CC11_STATUS_REGISTER) & CC11_PKTSTATUS_CCA) != 0))
    {
        MQ_t * pTxBuf = mqDequeue(&cc11_tx_queue);

        // Fill Buffer
        uint8_t i, len;

        cc11_cmdStrobe(CC11_SIDLE);                 // Switch to Idle state
        cc11_cmdStrobe(CC11_SFTX);                  // Flush the TX FIFO buffer
        cc11v_State = CC11_STATE_TXHDR;
        TxLEDon();

        len = pTxBuf->Length;
        // Send burst
        RF_SELECT();                                // Select CC1101
        RF_WAIT_LOW_MISO();                         // Wait until MISO goes low
        hal_cc11_spiExch(CC11_BIT_BURST | CC11_TXFIFO);
        hal_cc11_spiExch(len + 2);                  // Set data length at the first position of the TX FIFO
        hal_cc11_spiExch(*pTxBuf->cc11_adr);        // Send destination address
        hal_cc11_spiExch(cc11s_NodeID);             // Send Source address
        for(i = 0; i < len; i++)                    // Send Payload
            hal_cc11_spiExch(pTxBuf->raw[i]);
        RF_RELEASE();                               // Release CC1101
        cc11_cmdStrobe(CC11_STX);                   // Switch to TX state
        mqFree(pTxBuf);
    }
}

void CC11_IRQ_Handler()
{
    uint8_t marcs = cc11_readReg(CC11_MARCSTATE | CC11_STATUS_REGISTER);
    
    switch(cc11v_State)
    {
        case CC11_STATE_RXIDLE:
            if(marcs != CC11_MARCSTATE_RX)          // Synchro received
                break;

            RxLEDon();
            cc11v_State = CC11_STATE_RXDATA;
            return;
        case CC11_STATE_RXDATA:                     // Data received
            if(marcs == CC11_MARCSTATE_IDLE)
            {
                // read number of bytes in receive FIFO
                // Due a chip bug, the RXBYTES register must read the same value twice in a row to guarantee an accurate value.
                uint8_t frameLen, tmp, i;
                frameLen = cc11_readReg(CC11_RXBYTES | CC11_STATUS_REGISTER);
                do
                {
                    tmp = frameLen;
                    frameLen = cc11_readReg(CC11_RXBYTES | CC11_STATUS_REGISTER);
                } while (tmp != frameLen);

                if(((tmp & 0x7F) < 7) ||            // Packet is too small
                    (tmp & 0x80) ||                 // or Overflow
                    (tmp > (MQTTSN_MSG_SIZE + 3)))  // or Packet is too Big
                    break;

                frameLen -= 5;
                cc11_rx_data.Length = frameLen;

                // Read Burst
                RF_SELECT();                                    // Select CC1101
                RF_WAIT_LOW_MISO();                             // Wait until MISO goes low
                hal_cc11_spiExch(CC11_BIT_READ | CC11_BIT_BURST | CC11_RXFIFO);
                hal_cc11_spiExch(0);                            // Read Length
                hal_cc11_spiExch(0);                            // Read Destination address
                cc11_rx_data.SrcAddr = hal_cc11_spiExch(0);     // Read Source address

                for(i = 0; i < frameLen; i++)                   // Read Payload
                    cc11_rx_data.data[i] = hal_cc11_spiExch(0);

                cc11_rx_data.RSSI = hal_cc11_spiExch(0);        // Read RSSI
                cc11_rx_data.LQI  = hal_cc11_spiExch(0);        // Read LQI 
                RF_RELEASE();                                   // Release CC1101

                //cc11_rx_data.Foffs = cc11_readReg(CC11_FREQEST | CC11_STATUS_REGISTER);    // int8_t frequency offset
            }
            break;
        case CC11_STATE_TXHDR:
            if(marcs != CC11_MARCSTATE_TX)
                break;
            cc11v_State = CC11_STATE_TXDATA;
            return;
        default:
            break;
    }

    LEDsOff();
    cc11_cmdStrobe(CC11_SIDLE);     // Enter to the IDLE state
    cc11_cmdStrobe(CC11_SFTX);
    cc11_cmdStrobe(CC11_SFRX);
    cc11_cmdStrobe(CC11_SRX);       // Enter to RX State
    cc11v_State = CC11_STATE_RXIDLE;
}

void CC11_Init(void)
{
    uint8_t     Channel;
    uint16_t    GroupID;
    
    MQ_t * pBuf;
    while((pBuf = mqDequeue(&cc11_tx_queue)) != NULL)
        mqFree(pBuf);    

    // Load Device ID
    uint8_t Len = sizeof(uint8_t);
    ReadOD(objRFNodeId, MQTTSN_FL_TOPICID_PREDEF,  &Len, &cc11s_NodeID);
    // Load Frequency channel
    ReadOD(objRFChannel, MQTTSN_FL_TOPICID_PREDEF, &Len, &Channel);
    // Load Group ID(Synchro)
    Len = sizeof(uint16_t);
    ReadOD(objRFGroup, MQTTSN_FL_TOPICID_PREDEF,  &Len, (uint8_t *)&GroupID);

    hal_cc11_init_hw();
    // Reset CC1101
    _delay_us(5);
    RF_SELECT();
    _delay_us(10);
    RF_RELEASE();
    _delay_us(40);
    RF_SELECT();
    RF_WAIT_LOW_MISO();                     // Wait until MISO goes low
    hal_cc11_spiExch(CC11_SRES);            // Reset CC1101 chip
    RF_WAIT_LOW_MISO();                     // Wait until MISO goes low
    RF_RELEASE();

    // Configure CC1101
    uint8_t i;
    for (i=0; i<(sizeof(cc11config)/sizeof(cc11config[0])); i++)
        cc11_writeReg(cc11config[i][0], cc11config[i][1]);
        
    // Load Device ID
    cc11_writeReg(CC11_ADDR, cc11s_NodeID);
    // Load Group ID(Synchro)
    cc11_writeReg(CC11_SYNC1, GroupID>>8);
    cc11_writeReg(CC11_SYNC0, GroupID & 0xFF);
    // Load Frequency channel
    cc11_writeReg(CC11_CHANNR, Channel);
    // Configure PATABLE, No Ramp
    cc11_writeReg(CC11_PATABLE, CC11_RF_POWER);
    // Init Internal variables
    cc11v_State = CC11_STATE_IDLE;
    
    RF_ENABLE_IRQ();                      // Enable IRQ
}

void CC11_Send(void *pBuf)
{
    if(!mqEnqueue(&cc11_tx_queue, pBuf))
        mqFree(pBuf);
    else if(cc11v_State == CC11_STATE_RXIDLE)
        cc11_tx_task();
}

void * CC11_Get(void)
{
    MQ_t * pMqBuf;

    if(cc11v_State == CC11_STATE_IDLE)
    {
        cc11_cmdStrobe(CC11_SIDLE);         // Enter to the IDLE state
        cc11_cmdStrobe(CC11_SFTX);
        cc11_cmdStrobe(CC11_SFRX);
        cc11_cmdStrobe(CC11_SRX);           // Enter to RX State
        cc11v_State = CC11_STATE_RXIDLE;
    }
    else if(cc11v_State == CC11_STATE_RXIDLE)
    {
        if(cc11_rx_data.LQI & CC11_LQI_CRC_OK)
        {
            cc11_rx_data.LQI = 0;
            pMqBuf = mqAlloc(sizeof(MQ_t));
            if(pMqBuf != NULL)
            {
                pMqBuf->Length = cc11_rx_data.Length;
                memcpy(pMqBuf->raw, cc11_rx_data.data, cc11_rx_data.Length);
                pMqBuf->cc11_adr[0] = cc11_rx_data.SrcAddr;
                return pMqBuf;
            }
        }
        
        cc11_tx_task();
    }

    return NULL;
}

uint8_t CC11_GetRSSI(void)
{
    uint8_t rssi = cc11_rx_data.RSSI;
    if(rssi & 0x80)
    {
        rssi = ~rssi;
        rssi++;
        rssi >>= 1;
        rssi += 74;
    }
    else
    {
        rssi >>= 1;
        rssi = 74 - rssi;
    }
    return rssi;
}

#endif  //  CC11_PHY
