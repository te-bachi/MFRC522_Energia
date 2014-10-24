/*
* Copyright (c) 2012, Mauro Scomparin
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Mauro Scomparin nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY Mauro Scomparin ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL Mauro Scomparin BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* File:			main.c.
* Author:		Mauro Scomparin <http://scompoprojects.worpress.com>.
* Version:		1.0.0.
* Description:	Main sample file.
*/

#include <stdbool.h>
#include <string.h>

#include "mfrc522.h"

/* UART */
#include "uartstdio.h"

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

int selectCard(int dumpInfo);
void dumpHex(char* buffer, int len);

//4 bytes Serial number of card, the 5th byte is crc
uint8_t serNum[5];
//7 bytes Serial number of card, the 8th byte is crc
uint8_t serNum7[8];
//buffer
//uchar str[MAX_LEN];

uint8_t defaultKeyA[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t madKeyA[16] =     { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5 };
uint8_t NDEFKeyA[16] =    { 0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7 };

// main function.
int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlDelay(5000);


    MFRC522_Init();
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
//    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
//    TimerControlStall(TIMER1_BASE, TIMER_A, true);
//    TimerLoadSet(TIMER1_BASE, TIMER_A, 2111);
//    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1A_ISR);
//    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
//    count=3;
//    TimerEnable(TIMER1_BASE, TIMER_A);


    uint8_t version = Read_MFRC522(VersionReg);
    UARTprintf("MFRC522 Version: 0x%02x\n", version);

    if (0) {
        // Enable PORT F GPIO
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

        // set LED pins as outputs
        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);

        // loop forever
        int k;
        for (int i = 0; true; i++) {
            k = i % 2;
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, (k << 5));
            SysCtlDelay(5000);
        }

    }

    while(1) {
        uint8_t status;
        uint8_t buffer[MAX_LEN];
        if (selectCard(true))
        {
             for(int block=0; block < 64; block++)
             {
                  status = MFRC522_Auth(PICC_AUTHENT1A, block, defaultKeyA, serNum); //auth with default key
                  if (status != MI_OK)
                  {
                       selectCard(false);
                       status = MFRC522_Auth(PICC_AUTHENT1A, block, madKeyA, serNum); //auth with MAD key
                  }
                  if (status != MI_OK)
                  {
                       selectCard(false);
                       status = MFRC522_Auth(PICC_AUTHENT1A, block, NDEFKeyA, serNum); //auth NDEF data key
                  }
                  if (status == MI_OK)
                  {
                       status = MFRC522_Read(block, buffer);
                       if (status == MI_OK)
                       {
                            if (block % 4 == 0)
                            {
                                UARTprintf("Sector ");
                                UARTprintf("%d",  block / 4);
                                UARTprintf(":\n");

                            }
                            dumpHex((char*)buffer, MAX_LEN);
                        }
                        else
                        {
                            UARTprintf("Read failed\n");
                            break;
                        }
                  }
                  else
                  {
                      UARTprintf("Auth failed\n");
                      //TODO Mifare Ultra-Light
                      //MIFARE Ultralight C http://www.nxp.com/documents/short_data_sheet/MF0ICU2_SDS.pdf
                      break;
                  }
             }//for
             SysCtlDelay(26666666);
        }
        else
        {
            UARTprintf("no card select\n");
        }
        //reset/init for next loop
        MFRC522_Init();
        SysCtlDelay(26666666);
    }
}

int selectCard(int dumpInfo)
{
  uint8_t status;
  uint8_t buffer[MAX_LEN];
  //Search card, return card types
  status = MFRC522_Request(PICC_REQIDL, buffer);//ShortFrame: 0x26 REQA (Request Type A)
  //status = MFRC522_Request(PICC_REQALL, buffer);//0x52 WUPA (Wake-Up)
  if (status == MI_OK)
  {
     if (dumpInfo)
     {
         UARTprintf("Card detected.\n ATQA:");
         dumpHex((char*)buffer, 2);
         UARTprintf("\n");
     }
     //Prevent conflict, return the 4 bytes Serial number of the card
     status = MFRC522_Anticoll(buffer);
     if (status == MI_OK)
     {
         memcpy(serNum, buffer, 5);
         uint8_t sak = 0;
         status = MFRC522_SelectTag(serNum, &sak);
         if (status == MI_OK && ((sak & 0x04) == 0x00))
         {
             if (dumpInfo)
             {
                 UARTprintf(" UID: ");
                 dumpHex((char*)serNum, 4);
                 UARTprintf("\n");
             }
             if ((sak & 0x20) == 0x20)
             {
                 //ISO/IEC FCD 14443-3: Table 9 — Coding of SAK
                 //if (dumpInfo)
                 //    Serial.println(" UID complete, PICC compliant with ISO/IEC 14443-4");
                 //send RATS (Request for Answer To Select)
                 uint8_t ats[MAX_LEN];
                 uint32_t unLen = 0;
                 status = MFRC522_RATS(ats, &unLen);
                 if (status == MI_OK && dumpInfo)
                 {
                     UARTprintf(" ATS:\n");
                      dumpHex((char*)ats, ats[0]);
                      UARTprintf("\n");
                 }
             }
             if (dumpInfo)
             {
                 UARTprintf(" SAK: 0x%02x\n", sak);
             }
             return true;
         }
         else
         {
             //cascading level 2
             memcpy(serNum7, &serNum[1], 3);//cascading L1
             status = MFRC522_Anticoll2(buffer);
             if (status == MI_OK)
             {
                 memcpy(&serNum7[3], buffer, 4);
                 status = MFRC522_SelectTag2(&serNum7[3], &sak);
                 if (dumpInfo)
                 {
                    UARTprintf(" UID: ");
                    dumpHex((char*)serNum7, 7);
                    UARTprintf("\n SAK: 0x%02x\n", sak);
                 }
                 return true;
             }
             else
             {
                 UARTprintf("ANTICOLL error: cascading level 2\n");
             }
         }
   }//Anticoll
   else
   {
       UARTprintf("ANTICOLL failed\n");
   }
 }
 else
 {
     //Serial.print("-");
 }
 return false;
}//selectCard

void dumpHex(char* buffer, int len)
{
  for(uint8_t i=0; i < len; i++) {
     if (i % 16 == 0) {
         UARTprintf(" ");
     }
     UARTprintf("%02x \x00", (uint8_t)(*(buffer + i)));

     if (i % 16 == 15) {
         UARTprintf("\n");
     }
  }
  //Serial.println(" ");
}
