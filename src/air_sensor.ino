// This #include statement was automatically added by the Particle IDE.
#include <SHT1x.h>

// This #include statement was automatically added by the Particle IDE.
#include <SdFat.h>

String boxLabel = "Sensors Box 2";
String calibrationTag = "Box01";
int sampleSize;
int redPin = D0;
unsigned long timeElapsed;
unsigned long firstTime;
unsigned long logTime;
unsigned long secTime = 0 ;
unsigned long prevTimeElapsed = 0;

//SD Card Initialization:
SdFat SD;
File fh2;
const uint8_t chipSelect = SS;
char buffer[13] = "photon00.txt";
String dataString = "";
String firstDataString = "";
int SDmarker;

//FTP:
#define FTPWRITE
IPAddress server( 152,3,52,104);
TCPClient client;
TCPClient dclient;
char outBuf[128];
char outCount;

//COZIRf: CO2 Sensors
double val;
double CO2conc = 0.0;
double FS = 2000;
double vsupply = 1023;
double CO2concSum;
double CO2concAvg;

//PM Sensors:
uint16_t pm10 = 0;
uint16_t pm25 = 0;
uint16_t pm100 = 0;
uint16_t tpm10 = 0;
uint16_t tpm25 = 0;
uint16_t tpm100 = 0;

uint16_t tpm10Sum;
uint16_t tpm25Sum;
uint16_t tpm100Sum;
uint16_t pm10Sum;
uint16_t pm25Sum;
uint16_t pm100Sum;
int PMerrors = 0;
uint8_t buf[48];
char* charbuf;

uint16_t TPM01ValueAvg;
uint16_t TPM2_5ValueAvg;
uint16_t TPM10ValueAvg;
uint16_t PM01ValueAvg;
uint16_t PM2_5ValueAvg;
uint16_t PM10ValueAvg;

//Alphasensor:
float workNO2, worksumNO2, auxNO2, auxsumNO2, workO3, worksumO3, auxO3, auxsumO3, NO2, O3, avgNO2, avgO3, sumNO2, sumO3 = 0;
float NO2sum, O3sum, NO2avg, O3avg;
int alphaPin=A1;
int alphaCount=0;

const int S0 = D2;
const int S1 = D3;

//Temp & Hum Sensor:
int tempCerrors = 0;
int humErrors = 0;
float tempC = 0;
float tempCsum;
float tempCavg;

float humidity = 0;
float humiditySum;
float humidityAvg;

const int dataPin = D5;
const int clkPin = D6;

//Calibration:
float zeroNO, aeNO, sensitivityNO, zeroCO, aeCO, sensitivityCO, zeroNO2, aeNO2, sensitivityNO2, zeroO3, aeO3, sensO3, auxNO2O3;

SYSTEM_THREAD(ENABLED); // before the setup() method

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  //Setup SD Card:
  Serial.print("Initializing SD card...");
  while(!SD.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("SD init fail");
    digitalWrite(redPin, HIGH);
    delay(300);
    digitalWrite(redPin, LOW);
    delay(300);
  }
  Serial.println("done.");

  firstDataString = Time.timeStr();
  String firstStr = boxLabel + "...New Logging Session..." + firstDataString;
  sdLog(buffer, firstStr);
  String infoStr = "yyyy/mm/dd, hh:mm:ss, TPM1, TPM2.5, TPM10, PM1, PM2.5, PM10, CO2, TempC, Humidity, NO, CO, NO2, O3";
  // String infoStr = "yyyy/mm/dd, hh:mm:ss, TPM1, TPM2.5, TPM10, PM1, PM2.5, PM10";
  sdLog(buffer, infoStr);

  //Setup Alphasensor:
  /* pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT); */

  //Initialize COZIR
  /* Serial.print("Initializing COZIR...");
  pinMode(A0, INPUT);
  Serial.println("done"); */

  //Timing
  firstTime = millis();
  secTime = millis();
  sampleSize = 0;
  int SDmarker=1;
}

bool done=true;
int FTPcount=0;
int fileNum=0;

void loop()
{
  if(millis()-secTime>1000)
  {
    sampleSize++;
    Serial.println(Time.timeStr());
    getPMValues();
    PMPrint();
    TimeElapseCalculation();
    Serial.print("\n");

    if (timeElapsed > 60000) {       //Take Avg Every Minute
      AverageCalculation();
      dataString = Time.timeStr();
      dataString += " "+String(TPM01ValueAvg) + ", " + String(TPM2_5ValueAvg) + ", " + String(TPM10ValueAvg);
      dataString += ", " + String(PM01ValueAvg) + ", " + String(PM2_5ValueAvg) + ", " + String(PM10ValueAvg);
      // dataString += ", " + String(CO2concAvg, 2) + ", " + String(tempCavg, 2) + ", " + String(humidityAvg, 2);
      // dataString += ", " + String(NO2avg) + ", " + String(O3avg);

      Serial.println("--------MINUTE AVERAGE below------------");
      Serial.println(dataString);
      Serial.println("----------------------------------------");
      PublishPM();
      //check if log is successful:
      SDmarker = sdLog(buffer, dataString);
      while (!SDmarker) {
        Serial.println("Card failed, or not present");  // keep flashing LED
        digitalWrite(redPin, HIGH);
        delay(300);
        digitalWrite(redPin, LOW);
        delay(300);
      }
      //Reset firstTime to the last logTime value
      firstTime = logTime;
      ReSet(true);
      done=false;
    }
    secTime = millis();
  }
}

void readSD()
{
  File fh = SD.open(buffer,FILE_READ);

  if(!fh)
  {
    Serial.println("Read SD fails...");
    return;
  }

  while(fh.available())
  {
    Serial.write(fh.read());
  }

  fh.close();
}

// ***Log values into SD Card ***//
int sdLog(const char* fileName, String stringToWrite) {
  int marker;
  Serial.print("Filename is :");
  Serial.print(fileName);
  Serial.println("Writing string: " + stringToWrite);
  File myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print(" to ");
    Serial.print(fileName);
    Serial.print("...");
    myFile.println(stringToWrite);
    myFile.close();
    Serial.println("done");
    Serial.print("\n");
    marker = 1;
  } else {

    Serial.print("error opening ");
    Serial.println(fileName);
    marker = 0;
  }
  myFile.close();
  return marker;
}

//***To Get Value from PM Sensor***//
bool getPMValues() {
  int idx = 0;
  bool hasPm25Value = false;
  int timeout = 200;
  String printout = "";
  bool flag = false;
  while (!hasPm25Value) {
    memset(buf, 0, sizeof(uint8_t)*48);
    while (Serial1.available()) { /* This gives us problems, why? */
        buf[idx++] = Serial1.read();
    }
    // Serial1.readBytes(charbuf, 24);
    // for(int i=0;i<24;i++)
    // {
    //   buf[i]=(int)charbuf[i]-48;
    // }

    if (buf[0] == 0x42 && buf[1] == 0x4d) {
      pm25 = ( buf[12] << 8 ) | buf[13];
      pm10 = ( buf[10] << 8 ) | buf[11];
      pm100 = ( buf[14] << 8 ) | buf[15];
      tpm10 = ( buf[4] << 8 ) | buf[5];
      tpm25 = ( buf[6] << 8 ) | buf[7];
      tpm100 = ( buf[8] << 8 ) | buf[9];

      if (checkValue(buf, 48)) {
        flag = true;
        // tpm10Sum += tpm10;  tpm25Sum += tpm25;  tpm100Sum += tpm100;
        // pm10Sum += pm10; pm10Sum += pm25; pm100Sum += pm100;
        //Serial.println(PMerrors);
      } else {
        PMerrors++;
      }
      hasPm25Value = true;

      //Comment out once PM checksum is fixed
      tpm10Sum += tpm10;  tpm25Sum += tpm25;  tpm100Sum += tpm100;
      pm10Sum += pm10; pm25Sum += pm25; pm100Sum += pm100;

      // Debbugging Tool: Print out 24 bytes PM data (See communication protocol for details)
      // for (int j = 0; j < 24; j++) {
      //   printout = printout + " " + buf[j];
      // }
      // Serial.print(printout);
      // Serial.print("\n");
      // Serial.print("\n");
    }
    timeout--;
    if (timeout < 0) {
      break;
    }
  }
  return flag;
}

// ***Checksum for PM values ***//
int checkValue(uint8_t thebuf[48], int leng)
{
  char receiveflag = 0;
  int receiveSum = 0;
  int i = 0;

  for (i = 0; i < leng; i++) {
    receiveSum = receiveSum + thebuf[i];
  }

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1] + thebuf[leng - 2] + thebuf[leng - 1])) //checksum the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

void PMPrint() {
  Serial.print("pm1.0= ");
  Serial.print(pm10);
  Serial.print(" ug/m3 ");
  Serial.print("pm2.5= ");
  Serial.print(pm25);
  Serial.print(" ug/m3 ");
  Serial.print("pm100= ");
  Serial.print(pm100);
  Serial.print(" ug/m3 ");
  Serial.print("tpm1.0= ");
  Serial.print(tpm10);
  Serial.print(" ug/m3 ");
  Serial.print("tpm2.5= ");
  Serial.print(tpm25);
  Serial.print(" ug/m3 ");
  Serial.print("tpm100= ");
  Serial.print(tpm100);
  Serial.println(" ug/m3");
}

void getAlpha() {
  if (alphaCount==0) {
    digitalWrite(S0,LOW);
    digitalWrite(S1,LOW);
    workNO2 = analogRead(alphaPin)* 4.9;
    alphaCount++;

  } else if (alphaCount==1) {
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
    auxNO2 = analogRead(alphaPin)* 4.9;
    alphaCount++;
  } else if (alphaCount==2) {
    digitalWrite(S0,LOW);
    digitalWrite(S1,HIGH);
    workO3 = analogRead(alphaPin)* 4.9;
    alphaCount++;
  } else {
    digitalWrite(S0,HIGH);
    digitalWrite(S1,HIGH);
    auxO3 = analogRead(alphaPin)* 4.9;

    // //Uncomment to debug alphasensor values:
    // Serial.print("workNO2 = ");
    // Serial.println(workNO2);
    // Serial.print("auxNO2 = ");
    // Serial.println(auxNO2);
    // Serial.print("workO3 = ");
    // Serial.println(workO3);
    // Serial.print("auxO3 = ");
    // Serial.println(auxO3);

    NO2 = ((workNO2 - 302) - (auxNO2 - 295)) / 0.264;
    O3 = (((workO3 - 400) - (auxO3 - 406)) - NO2 * 0.261) / 0.349;
    Serial.print("NO2 = ");
    Serial.print(NO2);
    Serial.print(" O3 = ");
    Serial.print(O3);

    NO2sum += NO2;
    O3sum += O3;

    alphaCount=0;
  }
}

// void getSHT1X() {
//   SHT1x sht1x(dataPin, clkPin);
//   tempC = sht1x.readTemperatureC();
//   if (tempC < 0 || tempC > 50) {
//     tempCerrors++;
//   }
//   else {
//     tempCsum += tempC;
//   }
//   humidity = sht1x.readHumidity();
//   if (humidity < 0 || humidity > 100) {
//     humErrors++;
//   }
//   else {
//     humiditySum += humidity;
//   }
//   Serial.print(" Temp = ");
//   Serial.print(tempC);
//   Serial.print(" ");
//   Serial.print("C");
//   Serial.print(" Humidity = ");
//   Serial.print(humidity);
//   Serial.println("%");
// }

void getCO2Values() {
  val = analogRead(A0);
  CO2conc = FS * (val / vsupply);
  CO2concSum += CO2conc;
  Serial.print("CO2=");
  Serial.println(CO2conc);
}

//***To Calculate Time Elapsed***//

void TimeElapseCalculation() {
  logTime = millis();
  timeElapsed = logTime - firstTime;
  Serial.print("Time elapsed: ");
  Serial.println(timeElapsed);

  long intervalTime = timeElapsed - prevTimeElapsed;
  Serial.print("Time from last measurement: ");
  Serial.println(intervalTime);
  prevTimeElapsed = timeElapsed;
  // prevTimeElapsed = millis();
}

void AverageCalculation() {
  Serial.print("A minute has elapsed...");

  TPM01ValueAvg = tpm10Sum / sampleSize;
  TPM2_5ValueAvg = tpm25Sum / sampleSize;
  TPM10ValueAvg = tpm100Sum / sampleSize;
  PM01ValueAvg = pm10Sum / sampleSize;
  PM2_5ValueAvg = pm25Sum / sampleSize;
  PM10ValueAvg = pm100Sum / sampleSize;

  CO2concAvg = CO2concSum / sampleSize;
  tempCavg = tempCsum / sampleSize;
  humidityAvg = humiditySum / sampleSize;

  NO2avg = NO2sum / sampleSize;
  O3avg = O3sum / sampleSize;

  Serial.println("Calculated avgs...");
}

void ReSet(boolean PMReSet) {
  CO2concSum = 0;
  tempCsum = 0;
  humiditySum = 0;
  NO2sum = 0;
  O3sum = 0;

  CO2concAvg = 0;
  tempCavg = 0;
  humidityAvg = 0;

  sampleSize = 0;
  humErrors = 0;
  tempCerrors = 0;

  if (PMReSet) {
    tpm10Sum = 0;
    tpm25Sum = 0;
    tpm100Sum = 0;
    pm10Sum = 0;
    pm25Sum = 0;
    pm100Sum = 0;

    TPM01ValueAvg = 0;
    TPM2_5ValueAvg = 0;
    TPM10ValueAvg = 0;
    PM01ValueAvg = 0;
    PM2_5ValueAvg = 0;
    PM10ValueAvg = 0;

    PMerrors = 0;
  }
}

byte myFTP () {
  #ifdef FTPWRITE
  fh2 = SD.open(buffer,FILE_READ);
  #else
  SD.remove(buffer);
  fh2 = SD.open(buffer,FILE_WRITE);
  #endif

  if(!fh2)
  {
    Serial.println("SD open fail");
    return 0;
  }

  #ifndef FTPWRITE
  if(!fh2.seek(0))
  {
    Serial.println("Rewind fail");
    fh2.close();
    return 0;
  }
  #endif

  Serial.println("SD opened");

  if (client.connect(server,21)) {
    Serial.println("Command connected");
  }
  else {
    fh2.close();
    Serial.println("Command connection failed");
    return 0;
  }

  //if(!eRcv()) return 0;
  client.println("USER particle");
  //if(!eRcv()) return 0;
  client.println("PASS photon");
  if(!eRcv()) return 0;
  client.println("SYST");
  if(!eRcv()) return 0;
  client.println("Type I");
  if(!eRcv()) return 0;
  client.println("PASV");
  if(!eRcv()) return 0;

  char *tStr = strtok(outBuf,"(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      Serial.println("Bad PASV Answer");
    }
  }

  unsigned int hiPort,loPort;

  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  Serial.print("Data port: ");
  hiPort = hiPort | loPort;
  Serial.println(hiPort);

  if (dclient.connect(server,hiPort)) {
    Serial.println("Data connected");
  }
  else {
    Serial.println("Data connection failed");
    client.stop();
    fh2.close();
    return 0;
  }

  #ifdef FTPWRITE
  client.print("STOR ");
  client.println(buffer);
  #else
  client.print("RETR ");
  client.println(buffer);
  #endif

  if(!eRcv())
  {
    dclient.stop();
    return 0;
  }

  #ifdef FTPWRITE
  Serial.println("Writing");

  byte clientBuf[64];
  int clientCount = 0;

  while(fh2.available())
  {
    clientBuf[clientCount] = fh2.read();
    clientCount++;

    if(clientCount > 63)
    {
      dclient.write(clientBuf,64);
      clientCount = 0;
    }
  }

  if(clientCount > 0) dclient.write(clientBuf,clientCount);

  #else
  while(dclient.connected())
  {
    while(dclient.available())
    {
      char c = dclient.read();
      fh2.write(c);
      Serial.write(c);
    }
    Serial.println("looping");
  }
  #endif

  dclient.stop();
  Serial.println("Data disconnected");

  if(!eRcv()) return 0;
  client.println("QUIT");
  if(!eRcv()) return 0;
  client.stop();
  Serial.println("Command disconnected");

  fh2.close();
  Serial.println("SD closed");
  return 1;
}

byte eRcv()
{
  byte respCode;
  byte thisByte;

  while(!client.available()) Spark.process();
  respCode = client.peek();
  outCount = 0;

  while(client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);

    if(outCount < 127)
    {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }

  if(respCode >= '4')
  {
    efail();
    return 0;
  }

  return 1;
}

void efail()
{
  byte thisByte = 0;

  client.println("QUIT");

  while(!client.available()) Spark.process();
  while(client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);
  }

  client.stop();
  Serial.println("Command disconnected");
  fh2.close();
  Serial.println("SD closed");
}

void fileChange(int Num) {
  int tens=floor(Num/10);
  int digit=Num%10;
  buffer[6]=tens+48;
  buffer[7]=digit+48;
}

void TCPrintPM(){
  Serial.println("Printing Data Over TCP");
  char data[256];
  snprintf(data, sizeof(data), "{\"Device\":\"%s\",\"PM10\":\"%s\", \"PM25\":\"%s\", \"PM100\":\"%s\", \"TPM10\":\"%s\",\"TPM25\":\"%s\",\"TPM100\":\"%s\"}", String(System.deviceID()).c_str(),String(PM01ValueAvg).c_str(), String(PM2_5ValueAvg).c_str(), String(PM10ValueAvg).c_str(), String(TPM01ValueAvg).c_str(), String(TPM2_5ValueAvg).c_str(), String(TPM10ValueAvg).c_str());

}

void PublishPM(){
  // dataString += " "+String(TPM01ValueAvg) + ", " + String(TPM2_5ValueAvg) + ", " + String(TPM10ValueAvg);
  // dataString += ", " + String(PM01ValueAvg) + ", " + String(PM2_5ValueAvg) + ", " + String(PM10ValueAvg);
  Serial.println("Publishing Data");
  char data[256];
  snprintf(data, sizeof(data), "{\"Device\":\"%s\",\"PM10\":\"%s\", \"PM25\":\"%s\", \"PM100\":\"%s\", \"TPM10\":\"%s\",\"TPM25\":\"%s\",\"TPM100\":\"%s\"}", String(System.deviceID()).c_str(),String(PM01ValueAvg).c_str(), String(PM2_5ValueAvg).c_str(), String(PM10ValueAvg).c_str(), String(TPM01ValueAvg).c_str(), String(TPM2_5ValueAvg).c_str(), String(TPM10ValueAvg).c_str());
  Particle.publish("air",data,60,PRIVATE);
}
