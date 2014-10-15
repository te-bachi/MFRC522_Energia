


void setup() {
  Serial.begin(9600);                       // RFID reader SOUT pin connected to Serial RX pin at 2400bps
  // start the SPI library:
  SPI.begin();

  pinMode(chipSelectPin,OUTPUT);         // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin
  digitalWrite(chipSelectPin, LOW);      // Activate the RFID reader
  pinMode(NRSTPD,OUTPUT);                // Set digital pin 10 , Not Reset and Power-down
  digitalWrite(NRSTPD, HIGH);

  MFRC522_Init();

  //display version info
  //9.3.4.8 VersionReg register : 0x91 / 0x92
  uint8_t version = Read_MFRC522(VersionReg);
  Serial.print("MFRC522 Version: 0x");
  Serial.println(version, HEX);
}

void loop()
{
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
                            Serial.print("Sector ");
                            Serial.print(block / 4, DEC);
                            Serial.println(": ");

                        }
                        dumpHex((char*)buffer, MAX_LEN);
                    }
                    else
                    {
                        Serial.println("Read failed");
                        break;
                    }
              }
              else
              {
                  Serial.println("Auth failed");
                  //TODO Mifare Ultra-Light
                  //MIFARE Ultralight C http://www.nxp.com/documents/short_data_sheet/MF0ICU2_SDS.pdf
                  break;
              }
         }//for
         delay(1000);
    }
    else
    {
        Serial.println("no card select");
    }
    //reset/init for next loop
    MFRC522_Init();
    delay(500);
}


