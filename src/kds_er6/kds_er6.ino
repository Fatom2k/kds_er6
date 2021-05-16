/**
ER6 ECU reader
Just for tests
SoftwareSerial > pins 2/3 for GPS
SoftwareSerial > pins 11/10 for bt or other
Serial is used for L9637D @ 10400bads 8N1

**/

#include <SoftwareSerial.h>

//-----------------------------//
// ##    Bluetooth HC-06    ## //
//Serial com. on Digital Pin 10 Rx & 11 Tx
SoftwareSerial BT(11, 10);
uint32_t lastBTrequest;
bool bluetoothConnected = false;
//-----------------------------//

SoftwareSerial gps(2,3);

//-----------------------------//
// ##       OBD K-Line      ## //
#define K_OUT 1 // K Output Line - TX (1) on Arduino
#define K_IN 0  // K Input  Line - RX (0) on Arduino

// Timings
#define MAXSENDTIME 2000 // 2 second timeout on KDS comms.
const uint8_t ISORequestByteDelay = 10;
const uint8_t ISORequestDelay = 40; // Time between requests.
// Source and destination adresses, ECU (0x11) & Arduino (0xF1)
const uint8_t ECUaddr = 0x11;
const uint8_t MyAddr = 0xF1;
bool ECUconnected = false;
//ToDo: Save only important parts to reduce size!!!
uint8_t ecuResponse[12];
uint32_t lastKresponse;
uint8_t ThrottlePosMax = 405;
uint8_t SubThrottleMax = 189;
//-----------------------------//

int rpm = 0;
int vitesse = 0;
int gear = 0;
int waterTemp = 0;
int intakeTemp = 0;
int throttle=0;

//test racedac format
char datatocompute[64];
char finaldata[64];
// compteur d echantillons
int countech = 0;
//compteur affichage
int countdat = 0;
int countdat1 = 90;

//Status LED (OnBoard)
#define BOARD_LED 13

//set 10hz
byte gps10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};

void setup() {
  //Other serial init------------
  
  gps.begin(9600);
  delay(50);
  gps.write(gps10hz, sizeof(gps10hz));
 delay(200);
 gps.end();
 
  BT.begin(9600);
  BT.println("Hello bt");
  // KDS Init--------------------
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
  // define board status led
  pinMode(BOARD_LED, OUTPUT);
  // status led aus
  digitalWrite(BOARD_LED, LOW);
  
  lastKresponse = 0;
  

  //questions for other things----

}

void loop() {

  rpm = readRpm();
  throttle = readThrottle();
  //vitesse = readSpeed();

  countdat++;
  if (countdat > 5){
    gear = readGear();
    countdat = 0;
  }

  countdat1++;
  if (countdat1 > 50) {
    waterTemp = readWaterTemp();
    intakeTemp = readIntakeTemp();
    countdat1 = 0;
  }


  //compteur des donnees acquises - necessaire pour reception des donnees par appli
  countech++;
  if (countech > 65535) {
    countech = 0;
  }
  //$RC1,<time>,<count>,<xacc>,<yacc>,<zacc>,<xgyro>,<ygyro>,<zgyro>,<rpm>,<a1>,<a2>,<a3>,<a4>,<a5>*checksum
  //$RC2,<time>,<count>,<xacc>,<yacc>,<zacc>,<d1>,<d2>,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>,<a7>,<a8>*checksum
  sprintf(datatocompute, "RC2,,%i,,,,%i,%i,%i,%i,%i,,,,,", countech, rpm, throttle, gear, waterTemp, intakeTemp );
  sprintf(finaldata, "$%s*%02X", datatocompute, chk(datatocompute));
  BT.println(finaldata);


}


//--- functions to read kds values -----------------------------------------------------
int readRpm()
{
  int value = 0;

  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request rpm pid ... 0x09
    // response is 2 bytes
    if (processRequest(0x09))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }

      // perhaps no error so compute the good value
      value = ecuResponse[2] * 100;
      value += ecuResponse[3];
      value *= 4;
      ecuResponse[3] = value % 100;
      value = (value - ecuResponse[3]);
      if (value >= 256)
        ecuResponse[2] = value / 256;
      else
        ecuResponse[2] = 0x00;

      value = value / 4;
      return value;

    }
  }
}

int readSpeed()
{
  int value = 0;

  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request speed pid 0x0C
    // response is 2 bytes
    if (processRequest(0x0C))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }
      //(A*100+B) / 2
      value = ecuResponse[2] * 100;
      value += ecuResponse[3];
      if (value >= 2)
        value /= 2;
      ecuResponse[2] = value;
      ecuResponse[3] = 0x00;
      return value;
    }
  }
}

int readGear()
{
  int value = 0;
  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request gear pid 0x0B
    // response is 2 bytes
    if (processRequest(0x0B))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }

      if (ecuResponse[2] == 0x00)
        value = 0;
      else
        value = ecuResponse[2];

      return value;

    }
  }
}

int readWaterTemp() {
  int value = 0;
  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request watertemp pid 0x06
    // response is 2 bytes
    if (processRequest(0x06))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }

      value = ecuResponse[2] - 48;
      value /= 1.6;
      //value += 40;

      return value;
    }
  }
}

int readIntakeTemp() {
  int value = 0;
  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request watertemp pid 0x07
    // response is 1 byte
    if (processRequest(0x07))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }

      value = ecuResponse[2] - 48;
      value /= 1.6;
      //value += 40;

      return value;
    }
  }
}

int readThrottle() {
  int value = 0;
  if (!ECUconnected)
    ECUconnected = fastInit();

  if (ECUconnected)
  {
    //request throttle pid 0x04
    // response is 2 bytes
    if (processRequest(0x04))
    {
      //Error Respondend from ECU! Re-Init after 2 Seconds
      if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
      {
        //Error responded
        return -1;
      }

      value = ecuResponse[2] * 100;
      value += ecuResponse[3];

      return value;
    }
  }
}

// ---------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------


// $RC1 or $RC2 checksum compute
int chk(String aa) {
  int checksum = 0;
  for (int i = 0; i < aa.length(); i++) {
    checksum = checksum ^ aa[i];
  }
  return checksum;
}


