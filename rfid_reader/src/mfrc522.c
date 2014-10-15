
#include "mfrc522.h"

// #include "inc/hw_ssi.h"
//#include "inc/hw_memmap.h"

/* UART */
#include "uartstdio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#define NUM_SSI_DATA 3


static void InitConsole(void);

unsigned long ulDataTx[NUM_SSI_DATA];
unsigned long ulDataRx[NUM_SSI_DATA];

//4 bytes Serial number of card, the 5th byte is crc
//uint8_t serNum[5];
//7 bytes Serial number of card, the 8th byte is crc
//uint8_t serNum7[8];
//buffer
//uint8_t str[MAX_LEN];

//uint8_t defaultKeyA[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//uint8_t madKeyA[16] =     { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5 };
//uint8_t NDEFKeyA[16] =    { 0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7 };

static void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioInit(0);
}

/*
 * Function：Write_MFRC5200
 * Description：write a byte data into one register of MR RC522
 * Input parameter：addr--register address；val--the value that need to write in
 * Return：Null
 */
void Write_MFRC522(uint8_t addr, uint8_t val)
{
    unsigned long a;
    unsigned long tmp;
    unsigned long w_value;
    unsigned long r_value;

    //UARTprintf("=== Write_MFRC522 ===\n");

    a = ((addr<<1)&0x7E);
    //UARTprintf("SSI Put: addr = 0x%02x\n", tmp);
    SSIDataPut(SSI1_BASE, a);
    SSIDataGet(SSI1_BASE, &tmp);
    //UARTprintf("SSI Get: tmp = 0x%02x\n", tmp);

    w_value = val;
    //UARTprintf("SSI Put: val = 0x%02x\n", val);
    SSIDataPut(SSI1_BASE, w_value);
    SSIDataGet(SSI1_BASE, &r_value);
    //UARTprintf("SSI Get: tmp = 0x%02x\n", tmp);
    //UARTprintf("\n");

    UARTprintf("Write_MFRC522: address = 0x%02x / 0x%02x, w-value = 0x%02x, r-value = 0x%02x\n", addr, a, w_value, r_value);
}


/*
 * Function：Read_MFRC522
 * Description：read a byte data into one register of MR RC522
 * Input parameter：addr--register address
 * Return：return the read value
 */
uint8_t Read_MFRC522(uint8_t addr)
{
    unsigned long a;
    unsigned long tmp;
    unsigned long w_value;
    unsigned long r_value;

    //UARTprintf("=== Read_MFRC522 ===\n");

    a = (((addr<<1)&0x7E) | 0x80);
    //UARTprintf("SSI Put: addr = 0x%02x\n", tmp);
    SSIDataPut(SSI1_BASE, a);
    SSIDataGet(SSI1_BASE, &tmp);
    //UARTprintf("SSI Get: tmp = 0x%02x\n", tmp);

    w_value = 0x00;
    //UARTprintf("SSI Put: val = 0x%02x\n", tmp);
    SSIDataPut(SSI1_BASE, w_value);
    SSIDataGet(SSI1_BASE, &r_value);
    //UARTprintf("SSI Get: tmp = 0x%02x\n", tmp);
    //UARTprintf("\n");

    UARTprintf("Read_MFRC522:  address = 0x%02x / 0x%02x, w-value = 0x%02x, r-value = 0x%02x\n", addr, a, w_value, r_value);

    return r_value;
}

/*
 * Function：SetBitMask
 * Description：set RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void SetBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}


/*
 * Function：ClearBitMask
 * Description：clear RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void ClearBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Function：AntennaOn
 * Description：Turn on antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOn(void)
{
    uint8_t temp;

    temp = Read_MFRC522(TxControlReg);
    if (!(temp & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/*
 * Function：AntennaOff
 * Description：Turn off antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function：ResetMFRC522
 * Description： reset RC522
 * Input parameter：null
 * Return：null
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}


/*
 * Function：InitMFRC522
 * Description：initilize RC522
 * Input parameter：null
 * Return：null
 */
void MFRC522_Init(void)
{

    InitConsole();

    UARTprintf("=== MFRC522_Init BEGIN ===\n");

    //UARTprintf("SSI ->\n");
    //UARTprintf("  Mode: SPI\n");
    //UARTprintf("  Data: 8-bit\n\n");

    // The SSI0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                   GPIO_PIN_3);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI1_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(SSI1_BASE, &ulDataRx[0]))
    {
    }

    MFRC522_Reset();

    //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);      //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);
    Write_MFRC522(TReloadRegH, 0);

    Write_MFRC522(TxAutoReg, 0x40);     //100%ASK
    Write_MFRC522(ModeReg, 0x3D);       //CRC initilizate value 0x6363  ???

    //ClearBitMask(Status2Reg, 0x08);       //MFCrypto1On=0
    //Write_MFRC522(RxSelReg, 0x86);        //RxWait = RxSelReg[5..0]
    //Write_MFRC522(RFCfgReg, 0x7F);        //RxGain = 48dB

    AntennaOn();        //turn on antenna

    UARTprintf("=== MFRC522_Init END ===\n");
}


/*
 * Function：MFRC522_Request
 * Description：Searching card, read card type
 * Input parameter：reqMode--search methods，
 *           TagType--return card types
 *              0x4400 = Mifare_UltraLight
 *              0x0400 = Mifare_One(S50)
 *              0x0200 = Mifare_One(S70)
 *              0x0800 = Mifare_Pro(X)
 *              0x4403 = Mifare_DESFire
 * return：return MI_OK if successed
 */
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType)
{
    uint8_t status;
    uint32_t backBits;          //the data bits that received

    Write_MFRC522(BitFramingReg, 0x07);     //TxLastBists = BitFramingReg[2..0] ???

    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10))
    {
         status = MI_ERR;
/*
             Serial.print("status: ");
             Serial.print(status, HEX);
             Serial.print(" backBits: ");
             Serial.print(backBits, HEX);
             Serial.println("");
*/
    }
    return status;
}


/*
 * Function：MFRC522_ToCard
 * Description：communicate between RC522 and ISO14443
 * Input parameter：command--MF522 command bits
 *           sendData--send data to card via rc522
 *           sendLen--send data length
 *           backData--the return data from card
 *           backLen--the length of return data
 * return：return MI_OK if successed
 */
uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;

    switch (command)
    {
        case PCD_AUTHENT:       //verify card password
        {
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        }
    case PCD_TRANSCEIVE:    //send data in the FIFO
        {
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        }
    default:
            break;
    }

    Write_MFRC522(CommIEnReg, irqEn|0x80);  //Allow interruption
    ClearBitMask(CommIrqReg, 0x80);         //Clear all the interrupt bits
    SetBitMask(FIFOLevelReg, 0x80);         //FlushBuffer=1, FIFO initilizate

    Write_MFRC522(CommandReg, PCD_IDLE);    //NO action;cancel current command  ???

    //write data into FIFO
    for (i=0; i<sendLen; i++)
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    //procceed it
    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80);        //StartSend=1,transmission of data starts
    }

    //waite receive data is finished
    i = 2000;   //i should adjust according the clock, the maxium the waiting time should be 25 ms???
    do
    {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);          //StartSend=0

    if (i != 0)
    {
        if(!(Read_MFRC522(ErrorReg) & 0x1B))    //BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
               status = MI_NOTAGERR;            //??
            }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
           *backLen = (n-1)*8 + lastBits;
        }
                else
                {
           *backLen = n*8;
        }

                if (n == 0)
                {
            n = 1;
        }
                if (n > MAX_LEN)
                {
            n = MAX_LEN;
        }

        //read the data from FIFO
                for (i=0; i<n; i++)
                {
            backData[i] = Read_MFRC522(FIFODataReg);
        }
            }
        }
        else
        {
        status = MI_ERR;
    }

    }
    else
    {
         //Serial.print("i=0");
    }

    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}


/*
 * Function：MFRC522_Anticoll
 * Description：Prevent conflict, read the card serial number
 * Input parameter：serNum--return the 4 bytes card serial number, the 5th byte is recheck byte
 * return：return MI_OK if successed
 */
uint8_t MFRC522_Anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck=0;
    uint32_t unLen;


    //ClearBitMask(Status2Reg, 0x08);       //TempSensclear
    //ClearBitMask(CollReg,0x80);       //ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00);     //TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
     //Verify card serial number
         for (i=0; i<4; i++)
     {
         serNumCheck ^= serNum[i];
     }
     if (serNumCheck != serNum[i])
     {
         status = MI_ERR;
     }
    }
    //SetBitMask(CollReg, 0x80);        //ValuesAfterColl=1
    return status;
}

//ANTICOLL cascading level 2
uint8_t MFRC522_Anticoll2(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck=0;
    uint32_t unLen;


    //ClearBitMask(Status2Reg, 0x08);       //TempSensclear
    //ClearBitMask(CollReg,0x80);       //ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00);     //TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL2;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
     //Verify card serial number
         for (i=0; i<4; i++)
     {
         serNumCheck ^= serNum[i];
     }
     if (serNumCheck != serNum[i])
     {
         status = MI_ERR;
     }
    }
    //SetBitMask(CollReg, 0x80);        //ValuesAfterColl=1
    return status;
}

//send RATS and returns ATS
uint8_t MFRC522_RATS(uint8_t *recvData, uint32_t *pLen)
{
    uint8_t status;
    uint32_t unLen = 0;

    recvData[0] = 0xE0; // RATS
    recvData[1] = 0x50; // FSD=128, CID=0
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
    /*
    Serial.print("unLen: ");
    Serial.print(unLen, HEX);
    Serial.print(" status: ");
    Serial.print(status, HEX);
    Serial.println("");
    */
    //TODO
    //if ((status != MI_OK) || (unLen != 0x90))
    //{
    //    status = MI_ERR;
    //}
    return status;
}


/*
 * Function：CalulateCRC
 * Description：Use MF522 to caculate CRC
 * Input parameter：pIndata--the CRC data need to be read，len--data length，pOutData-- the caculated result of CRC
 * return：Null
 */
void CalulateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    uint8_t i, n;

    ClearBitMask(DivIrqReg, 0x04);          //CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);         //Clear FIFO pointer
    //Write_MFRC522(CommandReg, PCD_IDLE);

    //Write data into FIFO
    for (i=0; i<len; i++)
    {
        Write_MFRC522(FIFODataReg, *(pIndata+i));
    }
    Write_MFRC522(CommandReg, PCD_CALCCRC);

    //waite CRC caculation to finish
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));            //CRCIrq = 1

    //read CRC caculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}


/*
 * Function：MFRC522_SelectTag
 * Description：Select card, read card storage volume
 * Input parameter：serNum--Send card serial number
 * sak see ISO14443-3 Table 9 — Coding of SAK
 * return return MI_OK if successed
 */
uint8_t MFRC522_SelectTag(uint8_t *serNum, uint8_t *sak)
{
    uint8_t i;
    uint8_t status;
    //uint8_t size;
    uint32_t recvBits;
    uint8_t buffer[9];
    //uint8_t buffCheck=0;

    //ClearBitMask(Status2Reg, 0x08);           //MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
        buffer[i+2] = *(serNum+i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);     //??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    //TODO: the above call returns 2 instead of MI_OK -> why?
    status = MI_OK;
    //Serial.print("recvBits: ");
    //Serial.print(recvBits, DEC);
    /*
    for (i=0; i<recBits / 8; i++)
    {
        buff[i] = *(buffer+i);
    }*/
    //dumpHex((char*)buffer, recvBits / 8);
    *sak = buffer[0];
    //Verify received buffer
    /* TODO
    for (i=0; i< recvBits/8; i++)
    {
       buffCheck ^= buffer[i];
    }
    if (buffCheck != buffer[i])
    {
       status = MI_ERR;
    }*/
    return status;
}

uint8_t MFRC522_SelectTag2(uint8_t *serNum, uint8_t *sak)
{
    uint8_t i;
    uint8_t status;
    //uint8_t size;
    uint32_t recvBits;
    uint8_t buffer[9];
    //uint8_t buffCheck=0;

    //ClearBitMask(Status2Reg, 0x08);           //MFCrypto1On=0

    buffer[0] = PICC_ANTICOLL2;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
        buffer[i+2] = *(serNum+i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    //TODO: the above call returns 2 instead of MI_OK -> why?
    status = MI_OK;
    //Serial.print("recvBits: ");
    //Serial.print(recvBits, DEC);
    /*
    for (i=0; i<recBits / 8; i++)
    {
        buff[i] = *(buffer+i);
    }*/
    //dumpHex((char*)buffer, recvBits / 8);
    *sak = buffer[0];
    //Verify received buffer
    /* TODO
    for (i=0; i< recvBits/8; i++)
    {
       buffCheck ^= buffer[i];
    }
    if (buffCheck != buffer[i])
    {
       status = MI_ERR;
    }*/
    return status;
}


/*
 * Function：MFRC522_Auth
 * Description：verify card password
 * Input parameters：authMode--password verify mode
                 0x60 = verify A passowrd key
                 0x61 = verify B passowrd key
             BlockAddr--Block address
             Sectorkey--Block password
             serNum--Card serial number ，4 bytes
 * return：return MI_OK if successed
 */
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum)
{
    uint8_t status;
    uint32_t recvBits;
    uint8_t i;
    uint8_t buff[12];

    //Verify command + block address + buffer password + card SN
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
        buff[i+2] = *(Sectorkey+i);
    }
    for (i=0; i<4; i++)
    {
        buff[i+8] = *(serNum+i);
    }
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Function：MFRC522_Read
 * Description：Read data
 * Input parameters：blockAddr--block address;recvData--the block data which are read
 * return：return MI_OK if successed
 */
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t *recvData)
{
    uint8_t status;
    uint32_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}


/*
 * Function：MFRC522_Write
 * Description：write block data
 * Input parameters：blockAddr--block address;writeData--Write 16 bytes data into block
 * return：return MI_OK if successed
 */
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t *writeData)
{
    uint8_t status;
    uint32_t recvBits;
    uint8_t i;
    uint8_t buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)        //Write 16 bytes data into FIFO
        {
            buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

/*
 * Function：MFRC522_Halt
 * Description：Command the cards into sleep mode
 * Input parameters：null
 * return：null
 */
void MFRC522_Halt(void)
{
    uint8_t status;
    uint32_t unLen;
    uint8_t buff[4];

    //ISO14443-3: 6.4.3 HLTA command
    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);

    if (status != MI_OK) {
        //
    }
}
