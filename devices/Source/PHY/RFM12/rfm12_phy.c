/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// RFM12 RF interface

#include "../../config.h"

#ifdef RFM12_PHY

#include "rfm12_reg.h"

// 433 MHz
#if (RF_BASE_FREQ > 433050000UL) && (RF_BASE_FREQ < 434790000UL)
#define RFM12_BAND          RFM12_BAND_433
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 433000000UL)/25000)
// 868 MHz
#elif (RF_BASE_FREQ > 868000000UL) && (RF_BASE_FREQ < 870000000UL)
#define RFM12_BAND          RFM12_BAND_868
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 868000000UL)/25000)
// 915 MHz
#elif (RF_BASE_FREQ > 902000000UL) && (RF_BASE_FREQ < 928000000UL)
#define RFM12_BAND          RFM12_BAND_915
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 902000000UL)/25000)
#else
#error  RF_BASE_FREQ does not belond to ISM band
#endif  // RF_BASE_FREQ

// Mode definition
#define RFM12_SLEEP_MODE    (RFM12_CMD_PWRMGT | RFM12_PWRMGT_DC)
#define RFM12_IDLE_MODE     (RFM12_CMD_PWRMGT | \
                             RFM12_PWRMGT_ES | RFM12_PWRMGT_EX | RFM12_PWRMGT_DC)
#define RFM12_RECEIVE_MODE  (RFM12_CMD_PWRMGT | RFM12_PWRMGT_ER | RFM12_PWRMGT_EBB | \
                             RFM12_PWRMGT_ES | RFM12_PWRMGT_EX | RFM12_PWRMGT_DC)
#define RFM12_TRANSMIT_MODE (RFM12_CMD_PWRMGT | RFM12_PWRMGT_ET | \
                             RFM12_PWRMGT_ES | RFM12_PWRMGT_EX | RFM12_PWRMGT_DC)

#define RFM12_TXFIFO_DIS    (RFM12_CMD_CFG | RFM12_CFG_EF | RFM12_BAND | RFM12_XTAL_12PF)
#define RFM12_TXFIFO_ENA    (RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BAND | \
                             RFM12_XTAL_12PF)

#define RFM12_RXFIFO_DIS    (RFM12_CMD_FIFORESET | (0x08<<RFM12_FIFOITLVL_OFS) | RFM12_FIFORESET_DR)
#define RFM12_RXFIFO_ENA    (RFM12_CMD_FIFORESET | (0x08<<RFM12_FIFOITLVL_OFS) | \
                             RFM12_FIFORESET_FF  | RFM12_FIFORESET_DR)
// Chip configuration
#define RFM12_BAUD          RFM12_BAUD_38K4         // Follow FSK shift & bandwidth
// Quartz +-50 ppm
#define RFM12_BANDWIDTH     RFM12_RXCTRL_BW_270
#define RFM12_FSKWIDTH      RFM12_TXCONF_FS_135

//#define RFM12_BANDWIDTH     RFM12_RXCTRL_BW_134
//#define RFM12_FSKWIDTH      RFM12_TXCONF_FS_90

#define RFM12_GAIN          RFM12_RXCTRL_LNA_6
#define RFM12_DRSSI         RFM12_RXCTRL_RSSI_91
#define RFM12_POWER         RFM12_TXCONF_POWER_0

#define RFM12_TX_RETRYS		64

#if (RFM12_PHY == 1)
#define rfm12_adr               phy1addr

#ifdef LED1_On
void SetLED1mask(uint16_t mask);
#define rfm12_active()          SetLED1mask(1);
#else
#define rfm12_active()
#endif  //  LED1_On

#elif (RFM12_PHY == 2)
#define rfm12_adr               phy2addr

#ifdef LED2_On
void SetLED2mask(uint16_t mask);
#define rfm12_active()          SetLED2mask(1);
#else
#define rfm12_active()
#endif  //  LED2_On

#endif  // RFM12_PHY

// RF States
enum e_RF_TRVSTATE
{
    RF_TRVPOR = 0,
    RF_TRVSLEEP,
    RF_TRVIDLE,
    RF_TRVRXIDLE,
    RF_TRVRXHDR_DST,
    RF_TRVRXHDR_SRC,
    RF_TRVRXDATA,
    RF_TRVRXDONE,
    RF_TRVTXHDR,
    RF_TRVTXDATA,
    RF_TRVTXDONE,
    RF_TRVASLEEP,
    RF_TRVWKUP
};

static uint8_t          rfm12_state;
static uint8_t          rfm12_NodeID;
static uint16_t         rfm12_GroupID;
static Queue_t          rfm12_tx_queue = {NULL, NULL, 0, 0};
static MQ_t           * rfm12_pRxBuf = NULL;
static MQ_t           * rfm12_pTxBuf = NULL;

// HAL section
void        hal_rfm12_init_hw(void);
uint16_t    hal_rfm12_spiExch(uint16_t data);
void        hal_rfm12_spi_fast(void);
void        hal_rfm12_spi_slow(void);
bool        hal_rfm12_irq_stat(void);
void        hal_rfm12_enable_irq(void);


// Local subroutines
static void rfm12_CalcCRC(uint8_t data, uint16_t *pCRC)     // CRC Calculation compatible with cc1101
{
    uint8_t i;
    uint16_t crcReg = *pCRC;
    for (i = 0; i < 8; i++)
    {
        if(((crcReg & 0x8000) >> 8) ^ (data & 0x80))
            crcReg = (crcReg<<1) ^ 0x8005;
        else
            crcReg = (crcReg<<1);
        data <<= 1;
    }
    *pCRC = crcReg;
}

// Load byte from RX FIFO
static uint8_t rfm12_get_fifo(void)
{
    uint8_t data;
    hal_rfm12_spi_slow();
    data = hal_rfm12_spiExch(RFM12_CMD_READ) & 0xFF;
    hal_rfm12_spi_fast();
    return data;
}

static void rfm12_tx_task(void)
{
    static uint8_t rfm12_tx_delay = 0;
    static uint8_t rfm12_tx_retry = RFM12_TX_RETRYS;

    if(rfm12_tx_queue.Size == 0)
        return;

    // CDMA
    if(rfm12_tx_delay > 0)
    {
        rfm12_tx_delay--;
        return;
    }

    // Carrier ?
    if((hal_rfm12_spiExch(0) & RFM12_STATUS_RSSI) != 0)   // Channel Busy
    {
        if(rfm12_tx_retry > 0)
        {
            rfm12_tx_retry--;
            rfm12_tx_delay = (rfm12_NodeID>>1) + (halRNG() & 0x7F);
            return;
        }
    }
    // Send
    rfm12_pTxBuf = mqDequeue(&rfm12_tx_queue);
    if(rfm12_pTxBuf == NULL)                    // Queue Busy
        return;

    rfm12_tx_delay = 0;
    rfm12_tx_retry = RFM12_TX_RETRYS;
    rfm12_state = RF_TRVTXHDR;

    hal_rfm12_spiExch(RFM12_IDLE_MODE);         // Switch to Idle state
    hal_rfm12_spiExch(RFM12_TXFIFO_ENA);        // Enable TX FIFO

    hal_rfm12_spiExch(RFM12_TRANSMIT_MODE);
}

// IRQ subroutine
void rfm12_irq(void)
{
    static uint16_t     rfm12v_RfCRC;       // Actual CRC
    static uint8_t      rfm12v_RfLen;       // Packet Length
    static uint8_t      rfm12v_Pos;         // Position

    uint16_t intstat = hal_rfm12_spiExch(0);
    uint8_t ch;
    
    if(intstat & RFM12_STATUS_POR)          // Power-on reset
    {
        hal_rfm12_spiExch(RFM12_SLEEP_MODE);
        rfm12_state = RF_TRVPOR;
        return;
    }
    
    if(intstat & RFM12_STATUS_RGIT)         // Rx/Tx FIFO events
    {
        switch(rfm12_state)
        {
            // Start Rx Section
            case RF_TRVRXIDLE:              // Get Packet Length
                ch = rfm12_get_fifo();
                if((ch < 4) || (ch > (MQTTSN_MSG_SIZE + 3)))
                    break;
                rfm12v_RfCRC = 0xFFFF;
                rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                rfm12_state = RF_TRVRXHDR_DST;
                rfm12v_RfLen = ch - 2;      // Packet length(data), w/o CRC
                return;
            case RF_TRVRXHDR_DST:           // Destination Address
                ch = rfm12_get_fifo();
                if((ch == 0) || (ch == rfm12_NodeID))
                {
                    rfm12_active();

                    rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                    rfm12_state = RF_TRVRXHDR_SRC;
                    return;
                }
                break;
            case RF_TRVRXHDR_SRC:           // Source Address
                ch = rfm12_get_fifo();
                rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);

                rfm12_pRxBuf->phy1addr[0] = ch;
                rfm12v_Pos = 0;
                rfm12_state = RF_TRVRXDATA;
                return;
            case RF_TRVRXDATA:
                ch = rfm12_get_fifo();
                if(rfm12v_Pos < rfm12v_RfLen)
                {
                    rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                    rfm12_pRxBuf->raw[rfm12v_Pos++] = ch;
                    return;
                }
                else if(rfm12v_Pos == rfm12v_RfLen)     // 1st CRC byte;
                {
                    uint8_t bTmp = rfm12v_RfCRC>>8;
                    if(ch == bTmp)
                    {
                        rfm12v_Pos++;
                        return;
                    }
                }
                else                                    // 2nd CRC byte
                {
                    hal_rfm12_spiExch(RFM12_IDLE_MODE);
                    hal_rfm12_spiExch(RFM12_RXFIFO_DIS);
/*
                    uint8_t bTmp = rfm12v_RfCRC & 0xFF;

                    if(ch == bTmp)
                    {
//                        rfm12v_Foffs = (uint8_t)(intstat & 0x1F)<<3;	// int8_t frequency offset
                    }
                    return;
*/
                }
                break;
            case RF_TRVTXHDR:
                if(rfm12v_Pos == 0)             // Send preamble, preamble length 3 bytes
                {
                    ch = 0xAA;
                    rfm12_active();
                }
                else if(rfm12v_Pos == 1)        // Send Group ID, MSB
                    ch = rfm12_GroupID>>8;
                else if(rfm12v_Pos == 2)        // Send Group ID, LSB
                    ch = rfm12_GroupID & 0xFF;
/*
                else if(rfm12v_Pos == 3)        // Send packet length
                {
                    ch = rfm12v_RfLen;
                    rfm12v_RfCRC = 0xFFFF;
                    rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                    rfm12v_RfLen -= 1;
                }
                else if(rfm12v_Pos == 4)        // Send destination addr
                {
                    ch = rfm12v_pRfBuf[0];
                    rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                }
                else                            // Send Source addr;
                {
                    ch = rfm12s_NodeID;
                    rfm12_CalcCRC(ch, (uint16_t *)&rfm12v_RfCRC);
                    rfm12v_Pos = 0;
                    rfm12v_State = RF_TRVTXDATA;
                }
                rfm12v_Pos++;
                rfm12_control(RFM12_CMD_TX | ch);
                return;
                
                rfm12v_Pos = 0;
*/
                break;
                #warning rfm12_irq RF_TRVTXHDR
        }
    }

    // Switch to Receive mode
    hal_rfm12_spiExch(RFM12_IDLE_MODE);
    hal_rfm12_spiExch(RFM12_RXFIFO_DIS);
    hal_rfm12_spiExch(RFM12_TXFIFO_DIS);
    
    hal_rfm12_spiExch(RFM12_RECEIVE_MODE);
    hal_rfm12_spiExch(RFM12_RXFIFO_ENA);
    rfm12_state = RF_TRVRXIDLE;
}

// API Section
void RFM12_Init(void)
{
    uint8_t     Channel;

    MQ_t * pBuf;
    while((pBuf = mqDequeue(&rfm12_tx_queue)) != NULL)
        mqFree(pBuf);

    // Load Device ID
    uint8_t Len = sizeof(uint8_t);
    ReadOD(objRFNodeId, MQTTSN_FL_TOPICID_PREDEF,  &Len, &rfm12_NodeID);
    // Load Frequency channel
    ReadOD(objRFChannel, MQTTSN_FL_TOPICID_PREDEF, &Len, &Channel);
    // Load Group ID(Synchro)
    Len = sizeof(uint16_t);
    ReadOD(objRFGroup, MQTTSN_FL_TOPICID_PREDEF,  &Len, (uint8_t *)&rfm12_GroupID);
    
    hal_rfm12_init_hw();
    
    hal_rfm12_spiExch(0);           // initial SPI transfer added to avoid power-up problem
    hal_rfm12_spiExch(RFM12_SLEEP_MODE);
    
    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    hal_rfm12_spiExch(RFM12_CMD_TX);
    while(hal_rfm12_irq_stat())
        hal_rfm12_spiExch(0);

    hal_rfm12_spiExch(RFM12_CMD_CFG |           // Configuration Setting
                      RFM12_CFG_EL |            // Enable TX FIFO
                      RFM12_CFG_EF |            // Enable RX FIFO
                      RFM12_BAND |              // Select Band
                      RFM12_XTAL_12PF);         // Set XTAL capacitor
    hal_rfm12_spiExch(RFM12_SLEEP_MODE);
    hal_rfm12_spiExch(RFM12_CMD_FREQUENCY |     // Frequency Setting Command
                      Channel);
    hal_rfm12_spiExch(RFM12_CMD_DATARATE |      // Data Rate Command
                      RFM12_BAUD);
    hal_rfm12_spiExch(RFM12_CMD_RXCTRL |        // Receiver Control Command
                      RFM12_RXCTRL_P16_VDI |    // Pin16 - VDI output
                      RFM12_RXCTRL_VDI_MEDIUM | // VDI response - Medium
                      RFM12_GAIN |              // gain select
                      RFM12_BANDWIDTH |         // Receiver baseband bandwidth
                      RFM12_DRSSI);             // RSSI detector threshold
    hal_rfm12_spiExch(RFM12_CMD_DATAFILTER |    // Data Filter Command
                      RFM12_DATAFILTER_AL |     // Clock recovery (CR) auto lock control, Slow mode,
                                                //  Digital Filter
                      RFM12_DQD_THRESH_4);      // DQD threshold = 4( good)
    hal_rfm12_spiExch(RFM12_RXFIFO_DIS);        // FIFO and Reset Mode Command
    
    
    hal_rfm12_spiExch(RFM12_CMD_SYNCPATTERN |   // Synchro Pattern = 0x2D[grp]
                     (rfm12_GroupID & 0xFF));
    hal_rfm12_spiExch(RFM12_CMD_AFC |           // AFC Command
#if (RFM12_PHY != 1)    // Gateway
                      RFM12_AFC_AUTO_VDI |      // Keep the foffset only during receiving (VDI=high)
#else   // Node
                      RFM12_AFC_AUTO_KEEP |     // Keep the foffset value independently from the state of the VDI signal
#endif  // (RFM12_PHY == 2)
                      RFM12_AFC_LIMIT_16 |      // Limit  +75 -80 kHz
                      RFM12_AFC_FI |            // accuracy (fine) mode
                      RFM12_AFC_OE |            // Enables the frequency offset register
                      RFM12_AFC_EN);            // Enables the calculation of the offset frequency 
                                                //  by the AFC circuit.
    hal_rfm12_spiExch(RFM12_CMD_TXCONF |        // TX Configuration Control Command
                      RFM12_FSKWIDTH |          // FSK =  fo + offset
                      RFM12_POWER);             // Relative Output Power
    hal_rfm12_spiExch(RFM12_CMD_PLL |           // PLL Setting Command
                      RFM12_CLK_FRQ_HIGH |      // uC CLK Freq >= 5MHz
                      RFM12_PLL_DDIT);          // disable the dithering in the PLL loop.
    hal_rfm12_spiExch(RFM12_CMD_WAKEUP);        // Wake-Up Timer Command, not used
    hal_rfm12_spiExch(RFM12_CMD_DUTYCYCLE);     // Low Duty-Cycle Command, not used
    hal_rfm12_spiExch(RFM12_CMD_LBDMCD |        // Low Battery Detector and 
                                                //  Microcontroller Clock Divider Command
                      RFM12_MCD_DIV5);          // CLK Out = 2 MHz

    rfm12_state = RF_TRVIDLE;
    hal_rfm12_spiExch(RFM12_IDLE_MODE);

    hal_rfm12_spi_fast();
    hal_rfm12_enable_irq();
}

void RFM12_Send(void *pBuf)
{
    if(!mqEnqueue(&rfm12_tx_queue, pBuf))
        mqFree(pBuf);
}

void * RFM12_Get(void)
{
    if(rfm12_state == RF_TRVPOR)    // Power On Reset
    {
        RFM12_Init();
    }
    else if(rfm12_state == RF_TRVIDLE)
    {
        if(rfm12_pRxBuf == NULL)
            rfm12_pRxBuf = mqAlloc(sizeof(MQ_t));

        if(rfm12_pRxBuf != NULL)
        {
            rfm12_state = RF_TRVRXIDLE;
            hal_rfm12_spiExch(RFM12_RECEIVE_MODE);
            hal_rfm12_spiExch(RFM12_RXFIFO_ENA);
        }
    }
    else if(rfm12_state == RF_TRVRXDONE)
    {
        MQ_t * pRetVal = rfm12_pRxBuf;
        rfm12_pRxBuf = NULL;
        rfm12_state = RF_TRVIDLE;
        return pRetVal;
    }
    else if(rfm12_state == RF_TRVRXIDLE)
    {
        rfm12_tx_task();
    }

    return NULL;
}

void * RFM12_GetAddr(void)
{
    return &rfm12_NodeID;
}

#endif  //  RFM12_PHY