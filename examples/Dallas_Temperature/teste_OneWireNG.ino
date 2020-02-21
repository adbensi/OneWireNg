#ifndef ARDUINO_ARCH_ESP8266
#error Oops!  Make sure you have 'ESP8266' compatible board selected from the 'Tools -> Boards' menu.
#endif

extern "C" {
#include  "user_interface.h"
}

#include  "OneWireNg_CurrentPlatform.h"

#define OW_PIN            13
#define PW_PIN            16

//*********************************************
// #define PARASITE_POWER
#ifdef PARASITE_POWER
// # define PWR_CTRL_PIN   9
#endif

/* DS therms commands */
#define CONVERT_T           0x44
#define COPY_SCRATCHPAD     0x48
#define WRITE_SCRATCHPAD    0x4e
#define RECALL_EEPROM       0xb8
#define READ_POW_SUPPLY     0xb4
#define READ_SCRATCHPAD     0xbe

/* supported DS therms families */
#define DS18S20             0x10
#define DS1822              0x22
#define DS18B20             0x28
#define DS1825              0x3b
#define DS28EA00            0x42

//*********************************************

static  OneWireNg *ow         = new OneWireNg_ArduinoESP8266(OW_PIN, false);
OneWireNg::ErrorCode          ec;
OneWireNg::Id                 id[5];

//*********************************************
uint8_t                       S = 0, I2CSensor1ON = 0, I2CSensor2ON = 0, I2CSensor3ON = 0, I2CSensor4ON = 0, I2CSensor5ON = 0;
//*********************************************
int64_t                       BASE;
int32_t                       low32 = 0, high32 = 0;
float                         S1, S2, S3, S4, S5;
char                          n = 0;
//*********************************************

//***********************************************
void setup()
//***********************************************
{
  // put your setup code here, to run once:
  digitalWrite(OW_PIN, HIGH); // 1 Wire Data
  pinMode(OW_PIN, OUTPUT);

  digitalWrite(PW_PIN, LOW);  // 3.3V Source Power
  pinMode(PW_PIN, OUTPUT);

  //*********************************************

  Serial.begin(115200);
  delay(100);

  //*********************************************
  // Fix sensor poor
  digitalWrite(OW_PIN, LOW);
  digitalWrite(PW_PIN, HIGH);
  delay(70);

  digitalWrite(PW_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(OW_PIN, HIGH);
  delay(30);

  //*********************************************
  // REGISTER
  char n = 0;
  ow->searchReset();
  do
  {
    ec = ow->search(id[n]);
    if (!(ec == OneWireNg::EC_MORE || ec == OneWireNg::EC_DONE)) break;

    Serial.print("\nT" + String(n, DEC) + ": ");
    Serial.print(id[n][0], HEX);

    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
      Serial.print(':');
      Serial.print(id[n][i], HEX);
    }
    Serial.print("\n");
    if (n < 4) n++; else break;
  } while (ec == OneWireNg::EC_MORE);
  //*********************************************

  //*********************************************
  // DETECT
  n = 0;
  ow->searchReset();
  OneWireNg::Id idn;
  do
  {
    ec = ow->search(idn);
    if (!(ec == OneWireNg::EC_MORE || ec == OneWireNg::EC_DONE)) break;

    char match = 1;

    for (size_t i = 0; i < sizeof(OneWireNg::Id); i++) {
      if (id[n][i] != idn[i]) match = 0;
    }

    if (match)
    {
      Serial.print("\nT" + String(n, DEC) + " detected");
      switch (n)
      {
        case 0: I2CSensor1ON = 1; break;
        case 1: I2CSensor2ON = 1; break;
        case 2: I2CSensor3ON = 1; break;
        case 3: I2CSensor4ON = 1; break;
        case 4: I2CSensor5ON = 1; break;
      }
    }

    if (n < 4) n++; else break;
  } while (ec == OneWireNg::EC_MORE);
  //*********************************************
  BASE = millis64();
}


//***********************************************
int64_t millis64()
//***********************************************
{
  int32_t new_low32 = millis();
  if (new_low32 < low32) high32++;
  low32 = new_low32;
  return (int64_t) high32 << 32 | low32;
}

//***********************************************
void loop()
//***********************************************
{
    //***********************************************
    // control with processing if it's time
    //***********************************************
    int64_t now = millis64();
    if (((int64_t)now - BASE) > 500)
    {
      if (((n == 0) && (I2CSensor1ON)) || ((n == 1) && (I2CSensor2ON)) || ((n == 2) && (I2CSensor3ON)) || ((n == 3) && (I2CSensor4ON)) || ((n == 4) && (I2CSensor5ON)))
      {
        if (S)
        {
          uint8_t touchScrpd1[] = {
            READ_SCRATCHPAD,
            /* the read scratchpad will be placed here (9 bytes) */
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
          };
    
          ow->addressSingle(id[n]);
          ow->touchBytes(touchScrpd1, sizeof(touchScrpd1));
          uint8_t *scrpd1 = &touchScrpd1[1];  /* scratchpad data */
    
          if (OneWireNg::crc8(scrpd1, 8) != scrpd1[8]) {
            Serial.println("  Invalid CRC 1");
          }
          else
          {
            long temp = ((long)(int8_t)scrpd1[1] << 8) | scrpd1[0];
    
            if (id[n][0] != DS18S20) {
              unsigned res = (scrpd1[4] >> 5) & 3;
              temp = (temp >> (3 - res)) << (3 - res); /* zeroed undefined bits */
              temp = (temp * 1000) / 16;
            } else if (scrpd1[7]) {
              temp = 1000 * (temp >> 1) - 250;
              temp += 1000 * (scrpd1[7] - scrpd1[6]) / scrpd1[7];
            } else {
              /* shall never happen */
              temp = (temp * 1000) / 2;
              Serial.println("  Zeroed COUNT_PER_C detected!");
            }
    
            switch (n)
            {
              case 0: S1 = temp / 1000.0; break;
              case 1: S2 = temp / 1000.0; break;
              case 2: S3 = temp / 1000.0; break;
              case 3: S4 = temp / 1000.0; break;
              case 4: S5 = temp / 1000.0; break;
            }
          }
    
          // I ready this sensor every second, because it control the process.. but, where is it? Not here, this is a file test..
          if ((n != 0)&&(I2CSensor1ON))
          {
            uint8_t touchScrpd2[] = {
              READ_SCRATCHPAD,
              /* the read scratchpad will be placed here (9 bytes) */
              0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
            };
          
            ow->addressSingle(id[0]);
            ow->touchBytes(touchScrpd2, sizeof(touchScrpd2));
            uint8_t *scrpd2 = &touchScrpd2[1];  /* scratchpad data */
      
            if (OneWireNg::crc8(scrpd2, 8) != scrpd2[8]) {
              Serial.println("  Invalid CRC 2");
            }
            else
            {
              long temp = ((long)(int8_t)scrpd2[1] << 8) | scrpd2[0];
      
              if (id[0][0] != DS18S20) {
                unsigned res = (scrpd2[4] >> 5) & 3;
                temp = (temp >> (3 - res)) << (3 - res); /* zeroed undefined bits */
                temp = (temp * 1000) / 16;
              } else if (scrpd2[7]) {
                temp = 1000 * (temp >> 1) - 250;
                temp += 1000 * (scrpd2[7] - scrpd2[6]) / scrpd2[7];
              } else {
                /* shall never happen */
                temp = (temp * 1000) / 2;
                Serial.println("  Zeroed COUNT_PER_C detected!");
              }
    
              S1 = temp / 1000.0;
            }
          }
    
          Serial.print("\n");
          if (I2CSensor1ON) Serial.print(" S1:" + String(S1, 2));
          if (I2CSensor2ON) Serial.print(" S2:" + String(S2, 2));
          if (I2CSensor3ON) Serial.print(" S3:" + String(S3, 2));
          if (I2CSensor4ON) Serial.print(" S4:" + String(S4, 2));
          if (I2CSensor5ON) Serial.print(" S5:" + String(S5, 2));
          
        }
    
        /* start next temperature conversion */
        do
        {
          n++;
          if (n > 4) n = 0;
          if (((n == 0) && (I2CSensor1ON)) || ((n == 1) && (I2CSensor2ON)) || ((n == 2) && (I2CSensor3ON)) || ((n == 3) && (I2CSensor4ON)) || ((n == 4) && (I2CSensor5ON))) break;
        } while (1);
    
        if (!S) S = 1;
        //ow->addressSingle(id[n]);
        ow->addressAll();
        ow->writeByte(CONVERT_T);
    
    #ifdef PARASITE_POWER
        /* power the bus until the next activity on it */
        ow->powerBus(true);
    #endif
    
      } else if (n < 4) n++; else n = 0;

      //***********************************************
      BASE = now;
      //***********************************************
    }
    yield();    ESP.wdtFeed();    delay(5);
}
