/**
  si4432 Driver API Header File

  @Company
    roger225 Inc.

  @File Name
    si4432.h

  @Summary
    

  @Description
    This header file provides APIs for SI4432.
    Generation Information :
        Product Revision  :  roger225
        Device            :  SI4432
        Driver Version    :  1.00
 *******************************************************************************/



#ifndef _SI4432_H
#define _SI4432_H

#define WDS_Ready 0x01 //00000001
#define WDS_Sleep 0b00000000 //00010000
#define WDS_RX    0x04 //00000100
#define WDS_TX    0x08 //00001000
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
            WDS-Sleep   = 0x20  00010000
            WDS-RX      = 0x04  00000100
            WDS-TX      = 0x08  00001000
            */

#define SI4432_B1_Ready()   SI4432_B1_Write( 0x07, WDS_Ready )
#define SI4432_B1_Sleep()   SI4432_B1_Write( 0x07, WDS_Sleep )
#define SI4432_B1_RX()      SI4432_B1_Write( 0x07, WDS_RX )
#define SI4432_B1_TX()      SI4432_B1_Write( 0x07, WDS_TX )

#define SI4432_B1_WakeUp_Time_at_Sleep 1177
#define SI4432_B1_WakeUp_Time_at_DOOR_Working 102  //TOO SHORT MIGHT CAUSE PROBLEM
                                                  
/*TOO SHORT

case DOOR_STOP:
                DOOR_STOP_f();
                Motor_Control(Motor_STOP);
                __delay_ms(100);   
                Motor_Control(Motor_OFF);
                Door_Status = DOOR_IDLE;
                LED_STOP_SetLow();
                SI4432_B1_WakeUp_Timer_Period_Change_ms( SI4432_B1_WakeUp_Time_at_Sleep );
                break;  
these code can be pause by int request and 
interrupt too many times
might cause some problem

__delay_ms(xxx); change xxx short than "SI4432_B1_WakeUp_Time_at_DOOR_Working time" may help


*/



/**
  Section: Included Files
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
  
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
//Golbal Var
typedef enum{
  SI4432_IDLE,
  SI4432_TX_WAIT,
  SI4432_TX_FINISHED,
  SI4432_RX_Packet_Valid
}type_si4432_status;

extern type_si4432_status SI4432_Status;
extern uint8_t SI4432_Int_Status_1;
extern uint8_t SI4432_Int_Status_2;







    /**
      Section: SI4432 Module APIs
     */



    /**
      @Summary
        Read a data byte from SI4432 via SPI

      @Description
        This routine exchanges a data byte over SPI bus.
        This is a blocking routine.

      @Preconditions
        The SI4432_B1_Initialize() routine should be called
        prior to use this routine.

      @Param
        data - data byte to be transmitted over SPI bus

      @Returns
        The received byte over SPI bus

      @Example
        <code>
        uint8_t     writeData;
        uint8_t     readData;
        uint8_t     readDummy;

        SPI_Initialize();

        // for transmission over SPI bus
        readDummy = SPI_Exchange8bit(writeData);

        // for reception over SPI bus
        readData = SPI_Exchange8bit(DUMMY_DATA);
        </code>
     */
    uint8_t SI4432_B1_Read(uint8_t addr);

    /**
     @Summary
       Exchanges buffer of data over SPI

     @Description
       This routine exchanges buffer of data (of size one byte) over SPI bus.
       This is a blocking routine.

     @Preconditions
       The SPI_Initialize() routine should be called
       prior to use this routine.

     @Param
       dataIn  - Buffer of data to be transmitted over SPI.
       bufLen  - Number of bytes to be exchanged.
       dataOut - Buffer of data to be received over SPI.

     @Returns
       Number of bytes exchanged over SPI.

     @Example
       <code>
       uint8_t     myWriteBuffer[MY_BUFFER_SIZE];
       uint8_t     myReadBuffer[MY_BUFFER_SIZE];
       uint8_t     total;

       SPI_Initialize();

       total = 0;
       do
       {
           total = SPI_Exchange8bitBuffer(&myWriteBuffer[total], MY_BUFFER_SIZE - total, &myWriteBuffer[total]);

           // Do something else...

       } while(total < MY_BUFFER_SIZE);
       </code>
     */
    void SI4432_B1_Write(uint8_t addr, uint8_t data);

    void SI4432_B1_Send(uint8_t data01,uint8_t data02);
    void SI4432_B1_Rceive(uint8_t *data01,uint8_t *data02);
    void SI4432_Int_Read(void);
    void SI4432_B1_FIFO_Reset(void);
    void SI4432_B1_WakeUp_Timer_Period_Change_ms(uint16_t Timer_Period);  

    /**
      @Summary
        Initializes the SI4432

      @Description
        This routine initializes the SI4432.
        This routine must be called before any other SI4432 routine is called.
        This routine should only be called once during system initialization.

      @Preconditions
        None

      @Param
        None

      @Returns
        None

      @Comment
    

      @Example
        <code>
        uint8_t     myWriteBuffer[MY_BUFFER_SIZE];
        uint8_t     myReadBuffer[MY_BUFFER_SIZE];
        uint8_t     writeData;
        uint8_t     readData;
        uint8_t     total;

        SPI_Initialize();

        total = 0;
        do
        {
            total = SPI_Exchange8bitBuffer(&myWriteBuffer[total], MY_BUFFER_SIZE - total, &myWriteBuffer[total]);

            // Do something else...

        } while(total < MY_BUFFER_SIZE);

        readData = SPI_Exchange8bit(writeData);
        </code>
     */
    void SI4432_B1_Initialize(void);


#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif // _SI4432_H
/**
 End of File
 */
