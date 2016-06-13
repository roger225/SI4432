/**
  SI4432 Driver File

  @Company
    roger225 Inc.

  @File Name
    si4432.c

  @Summary
    This is the generated driver implementation file for the MSSP driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for MSSP.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16LF1937
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
 */
#include "mcc_generated_files/mcc.h"
#include <xc.h>
#include "si4432.h"
#include "door.h"

type_si4432_status SI4432_Status;
uint8_t SI4432_Int_Status_1;
uint8_t SI4432_Int_Status_2;

/**
  Section: Macro Declarations
 */

//#define SPI_RX_IN_PROGRESS 0x0

/**
  Section: Module APIs
 */

uint8_t SI4432_B1_Read(uint8_t addr)
{
    uint8_t     myWriteBuffer[2];
    uint8_t     myReadBuffer[2];
    uint8_t     total;
    
    addr &= 0b01111111; //read must 0 @ bit 7
    myWriteBuffer[0] = addr;
    myWriteBuffer[1] = 0x00;
    myReadBuffer[0] = 0x00;
    myReadBuffer[1] = 0x00;
    
    SI4432_SPI_SS_SetLow();
    //NOP();
    total = 0;
    do
    {
        total = SPI_Exchange8bitBuffer(&myWriteBuffer[total], 2 - total, &myReadBuffer[total]);

        // Do something else...

    } while(total < 2);
    //NOP();
    SI4432_SPI_SS_SetHigh();

    return  myReadBuffer[1];
}

void SI4432_B1_Write(uint8_t addr, uint8_t data)
{
    uint8_t     myWriteBuffer[2];
    uint8_t     total;
    
    addr |= 0b10000000; //write must 1 @ bit 7
    myWriteBuffer[0] = addr;
    myWriteBuffer[1] = data;
    
    SI4432_SPI_SS_SetLow();
    //NOP();
    
    total = 0;
    do
    {
        total = SPI_Exchange8bitBuffer(&myWriteBuffer[total], 2 - total, NULL);

        // Do something else...

    } while(total < 2);

    //NOP();
    SI4432_SPI_SS_SetHigh();

    //return  myReadBuffer[1];
}

void SI4432_B1_Send(uint8_t data01,uint8_t data02)
{
    SI4432_B1_Write( 0x7F, data01 );
    SI4432_B1_Write( 0x7F, data02 );
    //go to tx mode
    SI4432_Status = SI4432_TX_WAIT;
    SI4432_B1_TX();
    while(SI4432_Status != SI4432_TX_FINISHED);
    SI4432_B1_FIFO_Reset();
    SI4432_B1_Sleep();
}

void SI4432_B1_Rceive(uint8_t *data01,uint8_t *data02)
{
    *data01 = SI4432_B1_Read( 0x7F );
    *data02 = SI4432_B1_Read( 0x7F );
    SI4432_B1_FIFO_Reset();
    SI4432_B1_Sleep();
}

/*  
    When si4432 nIRQ fall,
    Retrieve SI4432_Int register to determin what happened
*/
void SI4432_Int_Read(void)
{
    uint8_t Switch_Condition;
   
    SI4432_Int_Status_1 = SI4432_B1_Read(0x03); //ifferr itxffafull itxffaem irxffafull iext ipksent ipkvalid icrcerror 
    SI4432_Int_Status_2 = SI4432_B1_Read(0x04);

    Switch_Condition = SI4432_Int_Status_1 & 0b00000110;//because for only use ipksent,ipkvalid
    
    switch( Switch_Condition )
    {
        case 0b00000010:
            SI4432_Status = SI4432_RX_Packet_Valid;
            break;
        case 0b00000100:
            SI4432_Status = SI4432_TX_FINISHED;
            break;
        default:
            break;                
/*for complex condition rx tx at same tiom
        case 0b00000110:
       
            break;*/
    }

}


void SI4432_B1_FIFO_Reset(void)
{
    SI4432_B1_Write( 0x08, 0b00000011 );
    SI4432_B1_Write( 0x08, 0b00000100 );
        //Operating Mode and Function Control 2:

            //000   Enable Antenna Diversity.
            //0     Automatic Transmission.
                    //When autotx = 1 the transceiver will enter automatically TX State when the FIFO is
                    //almost full. When the FIFO is empty it will automatically return to the Idle State.
            //1     RX Multi Packet.  ""重要"""
                    /*
                    When the chip is selected to use FIFO Mode (dtmod[1:0]) and RX Packet Handling (enpacrx)
                    then it will fill up the FIFO with multiple valid packets if this bit is set, otherwise
                    the transceiver will automatically leave the RX State after the first valid packet has been
                    received.
                    */
            //1     Enable Low Duty Cycle Mode.
                    /*
                    If this bit is set to 1 then the chip turns on the RX regularly. The frequency should
                    be set in the Wake-Up Timer Period register, while the minimum ON time should be set
                    in the Low-Duty Cycle Mode Duration register. The FIFO mode should be enabled also.
                    */
            //0     RX FIFO Reset/Clear.
                    //This has to be a two writes operation: Setting ffclrrx =1 followed by ffclrrx = 0
                    //will clear the contents of the RX FIFO.
            //0     TX FIFO Reset/Clear.
                    //This has to be a two writes operation: Setting ffclrtx =1 followed by ffclrtx = 0
                    //will clear the contents of the TX FIFO.
}

void SI4432_B1_WakeUp_Timer_Period_Change_ms(uint16_t Timer_Period)
{
    uint8_t Timer_Period_H,Timer_Period_L;
    //SI4432_B1_Write( 0x08, 0b00000000 );
    if( (Timer_Period >= 102) && (Timer_Period <= 2048) )
    {

    }
    else if( Timer_Period < 102)
    {
        Timer_Period = 102;
    }
    else
    {
        Timer_Period = 512;
    }

    Timer_Period = Timer_Period << 3; // 
    /*
    100 ms = 0x64

    */ 
    Timer_Period_H = Timer_Period >> 8;
    Timer_Period_L = Timer_Period;
    SI4432_B1_Write( 0x15, Timer_Period_H );//SI4432_B1_Write( 0x15, 0x00 );
    SI4432_B1_Write( 0x16, Timer_Period_L );//SI4432_B1_Write( 0x16, 0x01 );
        //Wake-Up Timer Period 2+1 wtm[15:8] + wtm[7:0]     0x2000 1s   0x1666 700ms 0x1000 500ms
                                                        //  0x17ae 0.74s    0x0333 100ms
            //XXXXXXXX
            //XXXXXXX1  Wake Up Timer Mantissa (M) Value*.
                        /*
                        The period of the wake-up timer can be calculated as 
                        TWUT = (4 x M x 2^R)/32.768 ms.
                        */
    //SI4432_B1_Write( 0x08, 0b00000100 );
    //SI4432_Int_Read();          
    SI4432_B1_FIFO_Reset();
    SI4432_B1_Sleep();
}




//CCP4CON = CCP4CON | 0b00001100; //  利用與 1 的 | 運算可以將某位元設成1
//CCP4CON = CCP4CON & 0b11110011; //  利用與 0 的 & 運算可以將某位元設成0


void SI4432_B1_Initialize(void)
{
    //  SI4432_B1_Read(uint8_t addr)
    //  SI4432_B1_Write(uint8_t addr, uint8_t data)
    SDN_SetLow();   //start si4432
    SI4432_Status    = SI4432_IDLE;
    SI4432_Int_Status_1 = 0x00;
    SI4432_Int_Status_2 = 0x00;
    __delay_ms(30); //si4432 poweron_time use boost should more than 30ms

//*****************************************************************************
//*
//* Interrupts Registers 
//* 
//* from 0x03 ~ 0x04 Interrupt Status
//* from 0x05 ~ 0x06 Interrupt Enable
//*****************************************************************************

    SI4432_B1_Write( 0x05, 0b00000110 );
        //Interrupt Enable 1

            //0     Enable FIFO Underflow/Overflow.
            //0     Enable TX FIFO Almost Full.
            //0     Enable TX FIFO Almost Empty.
            //0     Enable RX FIFO Almost Full.
            //0     Enable External Interrupt.
            //1     Enable Packet Sent.     1
            //1     Enable Valid Packet Received.       1
            //0     Enable CRC Error.

    SI4432_B1_Write( 0x06, 0x00 );
        //Interrupt Enable 2

            //0     Enable Sync Word Detected.
            //0     Enable Valid Preamble Detected.
            //0     Enable Invalid Preamble Detected.
            //0     Enable RSSI.
            //0     Enable Wake-Up Timer.
            //0     Enable Low Battery Detect.
            //0     Enable Chip Ready (XTAL).
            //0     Enable POR.




//*****************************************************************************







    SI4432_B1_Write( 0x07, 0x01 );
        //Operating Mode and Function Control 1:

            //x     Software Register Reset Bit.
            //x     Enable Low Battery Detect.
            //x     Enable Wake-Up-Timer.
                    //Enabled when enwt = 1. If the Wake-up-Timer function is enabled it will operate in
                    //any mode and notify the microcontroller through the GPIO interrupt when the timer expires.
            //x     32,768 kHz Crystal Oscillator Select.
            //x     TX on in Manual Transmit Mode.
                    //Automatically cleared in FIFO mode once the packet is sent.
            //x     RX on in Manual Receiver Mode.
                    //Automatically cleared if Multiple Packets config. is disabled and a valid packet received.
            //x     TUNE Mode (PLL is ON).
            //x     READY Mode (Xtal is ON).
            /*
            WDS-Ready   = 0x01  00000001
            WDS-Sleep   = 0x20  00010000    SI4432_B1_Write( 0x07, WDS-Sleep );
            WDS-RX      = 0x04  00000100
            WDS-TX      = 0x08  00001000
            */
    SI4432_B1_Write( 0x08, 0b00000100 );
        //Operating Mode and Function Control 2:

            //000   Enable Antenna Diversity.
            //0     Automatic Transmission.
                    //When autotx = 1 the transceiver will enter automatically TX State when the FIFO is
                    //almost full. When the FIFO is empty it will automatically return to the Idle State.
            //1     RX Multi Packet.  ""重要"""
                    /*
                    When the chip is selected to use FIFO Mode (dtmod[1:0]) and RX Packet Handling (enpacrx)
                    then it will fill up the FIFO with multiple valid packets if this bit is set, otherwise
                    the transceiver will automatically leave the RX State after the first valid packet has been
                    received.
                    */
            //1     Enable Low Duty Cycle Mode.
                    /*
                    If this bit is set to 1 then the chip turns on the RX regularly. The frequency should
                    be set in the Wake-Up Timer Period register, while the minimum ON time should be set
                    in the Low-Duty Cycle Mode Duration register. The FIFO mode should be enabled also.
                    */
            //0     RX FIFO Reset/Clear.
                    //This has to be a two writes operation: Setting ffclrrx =1 followed by ffclrrx = 0
                    //will clear the contents of the RX FIFO.
            //0     TX FIFO Reset/Clear.
                    //This has to be a two writes operation: Setting ffclrtx =1 followed by ffclrtx = 0
                    //will clear the contents of the TX FIFO.



        
    SI4432_B1_Write( 0x0B, 0b00011111 );
        //GPIO Configuration 0
    SI4432_B1_Write( 0x0C, 0b00011111 );
        //GPIO Configuration 1
    SI4432_B1_Write( 0x0D, 0b00011111 );
        //GPIO Configuration 2
    SI4432_B1_Write( 0x0E, 0x00 );
        //I/O Port Configuration


    SI4432_B1_Write( 0x14, 0x00 );
    //SI4432_B1_Write( 0x14, 0x03 );
        //Wake-Up Timer Period 1

            //000
            //00011     Wake Up Timer Exponent (R) Value*.
                        /*
                        Maximum value for R is decimal 20. A value greater than 20 will yield a result
                        as if 20 were written. R Value = 0 can be written here.
                        */

    SI4432_B1_Write( 0x15, 0x10 );//SI4432_B1_Write( 0x15, 0x00 );
    SI4432_B1_Write( 0x16, 0x00 );//SI4432_B1_Write( 0x16, 0x01 );
        //Wake-Up Timer Period 2+1 wtm[15:8] + wtm[7:0]     0x2000 1s   0x1666 700ms 0x1000 500ms
                                                        //  0x17ae 0.74s    0x0333 100ms
            //XXXXXXXX
            //XXXXXXX1  Wake Up Timer Mantissa (M) Value*.
                        /*
                        The period of the wake-up timer can be calculated as 
                        TWUT = (4 x M x 2^R)/32.768 ms.
                        */  

    SI4432_B1_Write( 0x19, 0x82 );//SI4432_B1_Write( 0x19, 0x52 ); a4
        //Low-Duty Cycle Mode Duration (LDC)*.
            /*
            20160509 for 2 bytes data recive should 0x82(15.8ms)
            if want get improve might use

             "Enable Valid Preamble Detected" @ si4432 0x06 Interrupt Enable 2
             
            to detect Valid Preamble than make si4432 turn off LDC and into rx_mode 


            */

            /*
            If enabled, the LDC will start together when the WUT is supposed to start, and the duration
            of the LDC is specified by the address 19h and the equation that goes with it. In order
            for the LDC to work, the LDC value has to be smaller than the M value specified in registers
            15h and 16h.
            LDC = 0 is not allowed here. Write at least decimal 1.
            */
            /*
            *Note: The period of the low-duty cycle ON time can be calculated as 
            TLDC_ON = (4 x LDC x 2R)/32.768 ms. 
            R is the same as in the wake-up timer setting in "Register 14h. Wake-Up Timer Period 1". 
            The LDC works in conjunction with the WUT. The LDC period must be specified to be smaller
            than the WUT period. (i.e., the LDC register must be smaller than the M register).
            The LDC may not be programmed to 0.
            */







//*****************************************************************************
//*
//* Packet Handler Registers 
//* 
//* from 0x30 ~ 0x4B
//* FIFO auto Packet Handler, let chip deal the communication,crc ...etc
//*****************************************************************************
    SI4432_B1_Write( 0x30, 0xAD );  
        //DatKa Access Control: 0xAD = 10101101

            //Enable Packet RX Handling = 1
            //LSB First Enable = 0
            //CRC Data Only Enable = 1
            //Skip 2nd Phase of Preamble Detection = 0
            //Enable Packet TX Handling = 1
            //CRC Enable = 1
            //CRC Polynomial Selection = 01 CRC-16 (IBM)

    SI4432_B1_Write( 0x32, 0xCC );
        //Header Control 1: 0xCC= 11001100

            //Broadcast Address (FFh) Check Enable = 1100
            //Received Header Bytes to be Checked Against the Check Header Bytes = 1100

    SI4432_B1_Write( 0x33, 0x22 );
        //Header Control 2 = 0x22= 00100010

            //Skipsyn = 0
            //Header Length = 010 Header 3 and 2
            //Fix Transmit/Receive Packet Length = 0
                /*
                When fixpklen = 0 the packet length is included in the transmit header.
                In receive mode, if this bit is set the packet length is obtained from
                the pklen[7:0] field in Reg 3Eh;
                otherwise the packet length is obtained from the received header packet length byte.
                */
            //Synchronization Word Length = 01 Synchronization Word 3 and 2
            //MSB of Preamble Length= 0

    SI4432_B1_Write( 0x34, 0x08 );
        //Preamble Length: 0x08 = 00001000 ""32bits Preamble""   20160424 40bits 0x0A
                                            //longer then Preamble Detection Control
                                            //if signal not very well maybe increase this value to 
                                            //increas the opportunity to wakeup reciver
            /*
            The value in the prealen[8:0] register corresponds to the number of nibbles (4 bits)
            in the packet. For example prealen[8:0] = ‘000001000’ corresponds to
            a preamble length of 32 bits (8 x 4bits) or 4 bytes.
            The maximum preamble length is prealen[8:0] = 111111111 which corresponds to a 255
            bytes Preamble.
            Writing 0 will have the same result as if writing 1, which corresponds to
            one single nibble of preamble.
            */

    SI4432_B1_Write( 0x35, 0x2A );
        //Preamble Detection Control 1: 0x2A = 00101010  0x20 00011010  
            //Preamble Detection Threshold = 00101 ""20bits Preamble""
                /*
                The value in the preath[4:0] register corresponds to the number of nibbles (4 bits)
                of preamble pattern (i.e., 01010...) that must be received correctly, before a
                PREAMBLE_VALID signal is issued.
                This threshold helps guard against false preamble detection upon noise.
                */
            //rssi_offset[2:0] = 010

    SI4432_B1_Write( 0x36, 0x2D );
        //Synchronization Word 3 (4th byte of the synchronization word.)
    SI4432_B1_Write( 0x37, 0xD4 );
        //Synchronization Word 2 (3th byte of the synchronization word.)
    SI4432_B1_Write( 0x38, 0x00 );
        //Synchronization Word 1 (2th byte of the synchronization word.)
    SI4432_B1_Write( 0x39, 0x00 );
        //Synchronization Word 0 (1th byte of the synchronization word.)


    SI4432_B1_Write( 0x3A, 0x19 );
        //Transmit Header 3 (4th byte of the header to be transmitted.)
    SI4432_B1_Write( 0x3B, 0x88 );
        //Transmit Header 2 (3th byte of the header to be transmitted.)
    SI4432_B1_Write( 0x3C, 0x02 );
        //Transmit Header 1 (2th byte of the header to be transmitted.)
    SI4432_B1_Write( 0x3D, 0x25 );
        //Transmit Header 0 (1th byte of the header to be transmitted.)

    SI4432_B1_Write( 0x3E, 0x02 );
        //Packet Length First_byte=>Instruction, Second_byte=>Data
            /*
            The value in the pklen[7:0] register corresponds directly to the number of bytes in the
            Packet. For example pklen[7:0] = ‘00001000’ corresponds to a packet length of 8 bytes.
            The maximum packet length is pklen[7:0] = ‘11111111’, a 255 byte packet. Writing 0 is
            possible, in this case we do not send any data in the packet. During RX, if fixpklen = 1,
            this will specify also the Packet Length for RX mode
            */

    SI4432_B1_Write( 0x3F, 0x19 );
        //Check Header 3 (4th byte of the check header.)   
    SI4432_B1_Write( 0x40, 0x88 );
        //Check Header 2 (3th byte of the check header.) 
    SI4432_B1_Write( 0x41, 0x02 );
        //Check Header 1 (2th byte of the check header.) 
    SI4432_B1_Write( 0x42, 0x25 );
        //Check Header 0 (1th byte of the check header.)


    SI4432_B1_Write( 0x43, 0xFF );
        //Header Enable 3 (4th byte of the check header.)
    SI4432_B1_Write( 0x44, 0xFF );
        //Header Enable 2 (3th byte of the check header.)
    SI4432_B1_Write( 0x45, 0xFF );
        //Header Enable 1 (2th byte of the check header.)
    SI4432_B1_Write( 0x46, 0xFF );
        //Header Enable 0 (1th byte of the check header.)

//*******************************************************************






//*******************************************************************
//*
//* Wireless Control Registers 
//* 
//* from 0x09 ~ 2E, 0x7C ~ 0x7F
//*
//****************************Do not Touch***************************

    SI4432_B1_Write( 0x09, 0x79 );
        //30 MHz Crystal Oscillator Load Capacitance
    SI4432_B1_Write( 0x1C, 0x1C );
        //IF Filter Bandwidth 
    SI4432_B1_Write( 0x1D, 0x44 );
        //AFC Loop Gearshift Override
    SI4432_B1_Write( 0x1E, 0x0A );
        //AFC Timing Control
    SI4432_B1_Write( 0x1F, 0x03 );
        //Clock Recovery Gearshift Override
    SI4432_B1_Write( 0x20, 0x64 );
        //Clock Recovery Oversampling Rate
    SI4432_B1_Write( 0x21, 0x00 );
        //Clock Recovery Offset 2
    SI4432_B1_Write( 0x22, 0xA3 );
        //Clock Recovery Offset 1
    SI4432_B1_Write( 0x23, 0xD7 );
        //Clock Recovery Offset 0
    SI4432_B1_Write( 0x24, 0x01 );
        //Clock Recovery Timing Loop Gain 1
    SI4432_B1_Write( 0x25, 0xB7 );
        //Clock Recovery Timing Loop Gain 0

    SI4432_B1_Write( 0x2A, 0x1E );
        //AFC Limiter
    SI4432_B1_Write( 0x2C, 0x18 );
        //OOK Counter Value 1
    SI4432_B1_Write( 0x2D, 0x3E );
        //OOK Counter Value 2
    SI4432_B1_Write( 0x2E, 0x29 );
        //Slicer Peak Holder

//26h. Received Signal Strength Indicator

    SI4432_B1_Write( 0x69, 0x60 );
        //AGC Override 1: 0x60 = 01100000

            //0
            //1     sgin
                /*
                AGC stop increasing gain override bit (active low). When ‘0’ (default), AGC gain
                increases during signal reductions are prevented. When ‘1’, AGC gain increases during
                signal reductions are allowed. Only effective during Preamble, prior to detection of
                PREAMBLE_VALID signal.
                */
            //1     Automatic Gain Control Enable  
            //0     LNA Gain Select   0 – min. gain = 5   dB
            //                     1 – max. gain = 255 dB
            //0000  PGA Gain Override Value  0000: 0 dB

    SI4432_B1_Write( 0x6D, 0x1C );
        //TX Power: 0x19 = 00011100 +11 dBm 

    SI4432_B1_Write( 0x6E, 0x51 );  //TX Data Rate Upper Byte
    SI4432_B1_Write( 0x6F, 0xEC );  //TX Data Rate Lower Byte
        //TX Data Rate 10k

    SI4432_B1_Write( 0x70, 0x2F );
        //Modulation Mode Control 1: 0x2F = 00101111
            
            //00
            //1     This bit should be set for Data Rates below 30 kbps.
            //0     If set, the Packet Handler will be powered down when chip is in low power mode
            //1     Manchester Preamble Polarity (will transmit a series of 1 if set, or series of 0 if reset).
            //1     Manchester Data Inversion is Enabled if this bit is set.
            //1     Manchester Coding is Enabled if this bit is set.
            //1     Data Whitening is Enabled if this bit is set.

    SI4432_B1_Write( 0x71, 0x23 );
        //Modulation Mode Control 2: 0x23 = 00100011
            
            //00    TX Data Clock Configuration.
                /*
                00: No TX Data CLK is available (asynchronous mode – Can only work with modulations FSK or OOK).
                */
            //10    Modulation Source.  10: FIFO Mode
            //0     Invert TX and RX Data.
            //0     MSB of Frequency Deviation Setting, see "Register 72h. Frequency Deviation".
            //11    Modulation Type.    11: GFSK (enable TX Data CLK (trclk[1:0]) when direct mode is used)

    SI4432_B1_Write( 0x72, 0x30 );
        //Frequency Deviation   頻偏為 30KHz

    SI4432_B1_Write( 0x73, 0x00 );
        //Frequency Offset 1    沒有頻率偏差
    SI4432_B1_Write( 0x74, 0x00 );
        //Frequency Offset 2    沒有頻率偏差

    SI4432_B1_Write( 0x75, 0x53 );
        //Frequency Band Select: 0x53 = 01010011

            //0
            //1     Side Band Select.
                /*
                Setting sbsel = 1 (recommended setting) will result in tuning the RX LO below the desired
                channel frequency in RX mode (low-side injection) such that the high-side sideband is
                selected. Note that setting sbsel = 0 will result in positioning the RX LO above the desired
                tuned frequency (high-side injection), but will NOT additionally flip the processing of the
                complex (I + jQ) signals in the IF chain necessary to select the lower sideband as the
                desired signal.
                */
            //0     High Band Select. 0 = choose the frequency range from 240–479.9 MHz (low bands).
            //10011 Frequency Band Select.

    SI4432_B1_Write( 0x76, 0x57 );
    SI4432_B1_Write( 0x77, 0x80 );
        //Nominal Carrier Frequency Setting.

    SI4432_B1_Write( 0x79, 0x00 );
        //Frequency Hopping Channel Select

    SI4432_B1_Write( 0x7A, 0x00 );
        //Frequency Hopping Step Size

//*****************************************************************************





//*****************************************************************************
//*
//* FIFO Control Registers 
//* 
//* from 0x7C ~ 0x7F
//*
//*****************************************************************************

    SI4432_B1_Write( 0x7C, 0x37 );
        //TX FIFO Control 1
        //00
        //110111(reset value)    TX FIFO Almost Full Threshold.

            /*
            This register specifies the threshold value at which the TXFFAFULL status bit/interrupt
            will be generated, as data bytes are stored into the TX FIFO for later transmission. This
            value should be programmed to 1 byte less than the desired threshold value. Example: A
            value of 0x3C=60d will not generate an interrupt if 60 bytes (or less) are written to the TX
            FIFO, but will generate an interrupt when 61 bytes (or more) are written to the TX FIFO.
            */

    SI4432_B1_Write( 0x7D, 0x04 );
        //TX FIFO Control 2
        //00
        //000100(reset value)    TX FIFO Almost Empty Threshold.

            /*
            This register specifies the threshold value at which the TXFFAEM status bit/interrupt will
            be generated, as data bytes are pulled from the TX FIFO and transmitted. This value
            should be programmed to 1 byte less than the desired threshold value. Example: A value
            of 0x05 will generate an interrupt when 6 bytes remain in the TX FIFO.
            */

    SI4432_B1_Write( 0x7E, 0x02 );
        //RX FIFO Control
        //00
        //000010    RX FIFO Almost Full Threshold.  ""2 Bytes""
                                                    //Packet Length First_byte=>Instruction, Second_byte=>Data
            /*
            This register specifies the threshold value at which the RXFFAFULL status bit/interrupt
            will be generated, as data bytes are received and stored into the RX FIFO for later
            retrieval. This value should be programmed to 1 byte less than the desired threshold
            value. Example: A value of 0x3C=60d will not generate an interrupt if 60 bytes (or less)
            are received and stored to the RX FIFO, but will generate an interrupt when 61 bytes (or
            more) are received and stored to the RX FIFO.
            */

    //SI4432_B1_Write( 0x7F, 0x00 );
        //FIFO Data.
            /*
            A Write (R/W = 1) to this Address will begin a Burst Write to the TX FIFO. The FIFO will
            be loaded in the same manner as a Burst SPI Write but the SPI address will not be incremented.
            To conclude the TX FIFO Write the SEL pin should be brought HIGH. A Read
            (R/W = 0) to this address will begin a burst read of the RX FIFO, in the same manner.   
            */
//**************************************************************************************************
    //SI4432_B1_FIFO_Reset();
    //SI4432_Int_Read();
    SI4432_B1_Sleep();


}









/**
 End of File
 */





