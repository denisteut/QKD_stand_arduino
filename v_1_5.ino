/*
  v 1.5
*/

#include <Adafruit_ADS1X15.h>
#include <Wire.h> 
#include <GyverStepper.h> 
#include <EEPROM.h>

// Базовые определения и настройки стенда
  int analogPin1 = A0;
  int analogPin2 = A1;
  int analogPin3 = A6;
  int analogPin4 = A7;
  GStepper<STEPPER2WIRE> stepper1(200, 5, 6); // 11, 12
  GStepper<STEPPER2WIRE> stepper2(200, 7, 8); // 9, 10
  GStepper<STEPPER2WIRE> stepper3(200, 9, 10); // 7 8
  GStepper<STEPPER2WIRE> stepper4(200, 11, 12); //5 6

  //Лазер
  #define UD  4  
  #define INC 3
  #define CS  2
  #define LaserPin 13
  Adafruit_ADS1115 ads1;
  Adafruit_ADS1115 ads2;  
  // CRC table
  const byte Crc8Table[256] = {
      0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
      0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
      0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
      0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
      0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
      0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
      0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
      0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
      0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
      0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
      0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
      0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
      0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
      0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
      0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
      0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
      0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
      0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
      0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
      0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
      0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
      0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
      0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
      0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
      0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
      0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
      0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
      0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
      0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
      0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
      0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
      0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
  };
//Структуры
  // Структура сообщения 
    struct Message {
      unsigned int lenght = 0;         //длинна пакета
      char condbyte = 0;      // код ошибки
      char key;              // ID команды
      unsigned int value[40];    // Параметры команды
      uint8_t checksum;// Контрольная сумма
      unsigned int params; //число паарметров 
      
      }; 
  //Структура, содержащая некоторые текущие значения показателей стенда
    struct{
      unsigned int MaxLaserPower = 100;
      int LaserPower = 0; // текущее значение мощности лазера
      int LaserState = 0; 
      unsigned int NoiseLevel1 = 0;
      unsigned int NoiseLevel2 = 0;
      unsigned int MaxSignalLevel1 = 0;
      unsigned int MaxSignalLevel2 = 0;
      double SensPD1 = 1;
      double SensPD2 = 1;
      unsigned int RotateStep = 1200;
      unsigned int StartNoiseLevel1 = 0;
      unsigned int StartNoiseLevel2 = 0;
      //Текущие углы поворота пластин
      unsigned int Angle1 = 0;
      unsigned int Angle2 = 0;
      unsigned int Angle3 = 0;
      unsigned int Angle4 = 0;
      unsigned int BaseAngle1 = 0;
      unsigned int BaseAngle2 = 0;
      unsigned int BaseAngle3 = 0;
      unsigned int BaseAngle4 = 0;
      // Характеристики стенда
      unsigned int HardwareState = 0;
      unsigned int Firmware[3] = {0, 0, 0};
      unsigned int MaxPayload = 0;
      unsigned int Secret[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      } Stand;     
    struct{
      unsigned int Angle1 = 0;
      unsigned int Angle2 = 0;
      unsigned int Angle3 = 0;
      unsigned int Angle4 = 0;
    } BaseAngles;

  // Структура, хранящая в себе размеры буферов
    struct{
      unsigned int buff;
      unsigned int outbuff;
      unsigned int printbuff;
    } Sizes;

// ------------------
// Глобальные переменные 
  int delayMK = 50;
  const int buffsize = 160;
  byte buff[buffsize];          //Буфер приема в buffsize байта
  unsigned int outbuff[buffsize/2];
  byte printbuff[buffsize];   // буфер для отправки на пк
  Message current;        //Экземпляр структуры для записи пакета
// ------------------


void setup() {
  Serial.begin(115200);
  pinMode(LaserPin, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  
  pinMode(INC, OUTPUT);
  pinMode(UD, OUTPUT);
  pinMode(CS, OUTPUT);

  digitalWrite(INC, LOW); 
  digitalWrite(CS, HIGH); 
  
  digitalWrite(INC, HIGH);
  
  ads1.setGain(GAIN_FOUR); // | 4х | +/-2.048V | 1bit = 0.0625mV
  ads2.setGain(GAIN_FOUR); // | 4х | +/-2.048V | 1bit = 0.0625mV   
  ads1.begin(0x48);
  ads2.begin(0x49);// Подключаем АЦП

  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setMaxSpeed(2400);
  stepper1.setAcceleration(0); 
  stepper2.setRunMode(FOLLOW_POS);
  stepper2.setMaxSpeed(2400);
  stepper2.setAcceleration(0); 
  stepper3.setRunMode(FOLLOW_POS);
  stepper3.setMaxSpeed(2400);
  stepper3.setAcceleration(0); 
  stepper4.setRunMode(FOLLOW_POS);
  stepper4.setMaxSpeed(2400);
  stepper4.setAcceleration(0); 


  ReWrite();

}

void loop() {
  if (Listening()){
    
    Send(); //Действия, отправка
    current.condbyte = 0; // костыль, вечен
  }
  memset(printbuff, NULL, Sizes.printbuff);
  //ReWrite(); //очистка, подготовка к новому циклу
  
}
// Подсчет контрольной суммы на входе массив и его длинна в байтах
byte GetCRC(byte *pcBlock, byte len){
    byte crc = 0xFF;
    while (len--)
        crc = Crc8Table[crc ^ *pcBlock++];
    return crc;
}

// метод "прослушивает" канал передачи, и в сучае, 
//если пришел полноценный пакет записывает его в глобальную стуктуру и возвращает "true"

bool Listening(){
  bool WrongCRC_Flag = false;
  int res;
  buff[0] = Serial.read();
  //поиск начала пакета
  while(true){ 
    buff[1] = Serial.read();
    if(buff[0] == 255 && buff[1] == 254){
    
      break;
    }
    buff[0] = buff[1];
  }
  delay(10); 
  //чтение байтов пришедших после символов начала пакета
  for(int i = 2; i < buffsize; i++){ 
    buff[i] = Serial.read();
    
    if(buff[i] == 255 && CheckCRC(i-1) != 0){
      i++;
      buff[i] = Serial.read();
      if(buff[i] == 255 ){
        Sizes.buff = i;
        current.params = (i - 8)/2; // 2 + 2 + 1 + <params> + 1 + 1 + 2 - 1 
        if(CheckCRC(i-3) == 0){
          break;
        }
        else{
          current.condbyte += 16; //x^4 -- Несоответствие контрольной суммы
          WrongCRC_Flag = true;
          break;
        }
      }
    }
  }

  if(Sizes.buff >= buffsize){
    //WrongCRC_Flag = true;
    current.condbyte += 4;//Нет метки конца пакета
    current.condbyte += 2;//большое кол-во параметров
  }
  // buff[] записан, парсинг
  current.key = buff[4];
  current.lenght = buff[2]<<8 | buff[3];
  for (int i = 0; i < current.params*2; i=i+2)
  {
    current.value [i/2]= (buff[i+5]<<8)| buff[i+6];
  }
  /*if(WrongCRC_Flag){
    ReturnPackage(buff); //отправка пакета назад
    return false;//несоответстие контрольной суммы 
  }*/
  return true;
}

//метод проверяет контрольную сумму для пришедшего buff[]
//n - количесво байт для подсчета (за вычетом заголовка)
void ReturnPackage (byte value[]){
  outbuff[0] = current.params;
  for (int i=0; i < current.params * 2; i=i+2){
    
    outbuff[i/2+1]= (value[i+3]<<8)| value[i+4];
  }
  SendUART();
}

void SendUART(){
    BuildMessage(outbuff);
    Serial.write(printbuff, Sizes.printbuff);
}
// строит вывод в массив printbuff
// первый байт в value отвечает за число переменных
void BuildMessage (unsigned int value[]){
  current.lenght = value[0]*2 + 3; // params + id + condbyte + salt
  memset(printbuff, 0, sizeof(printbuff));
  int pointer = 6;
  byte data[buffsize];
  printbuff[0] = B11111111; //255
  printbuff[1] = B11111110; //254
  printbuff[2] = current.lenght / 256;
  printbuff[3] = current.lenght % 256;

  printbuff[4] = (byte)current.key;
  printbuff[5] = (byte)current.condbyte;

  for(unsigned int i = 1; i < value[0]+1; i++){    
    printbuff[pointer] = (byte)(value[i] / 256);
    printbuff[pointer + 1] = (byte)(value[i] % 256);
    pointer = pointer + 2;
  }
  printbuff[pointer] = B00000000;

  for(int i = 0; i < pointer-1; i++){
    data[i] = printbuff[i + 2];  
  }  
  printbuff[pointer+1] = GetCRC((byte*)&data, pointer-1);
  printbuff[pointer + 2] = B11111111;
  printbuff[pointer + 3] = B11111111;
  Sizes.printbuff = pointer + 4;
}

void ReWrite(){
  current.condbyte = NULL;
  current.key = NULL;
  current.checksum = NULL;
  Sizes.outbuff = outbuff[0] + 1;
  memset(current.value, 0, sizeof(current.value));
  memset(buff, NULL, Sizes.buff);
  memset(outbuff, NULL, Sizes.outbuff);
  memset(printbuff, NULL, Sizes.printbuff);  
}

uint8_t CheckCRC(int n){
  byte curr[buffsize];
  for(int i = 0; i < n; i++){
    curr[i] = buff[i+2];
  }
  uint8_t CurrCRC = GetCRC((byte*)&curr, n);
  return CurrCRC;
}

void Send(){
    bool status = true;
    switch(current.key){

        case 17:                        //GetCurrentFirmwareVersion
          outbuff[0] = 3;
          outbuff[1] = Stand.Firmware[0];
          outbuff[2] = Stand.Firmware[1];
          outbuff[3] = Stand.Firmware[2];
          break;
        case 19:                        //GetMaxPayloadSize
          outbuff[0] = 1;
          outbuff[1] = Stand.MaxPayload;
          break;
        case 48:                        //CreateConfigSecret

          outbuff[0] = 1;
          outbuff[1] = CreateConfigSecret();
          break;
        case 49:                        //OpenConfigMode
          
          for(int i = 0; i < 10; i++){
            if (Stand.Secret[i] != current.value[i]){
              status = false;
            }
          }

          outbuff[0] = 1;
          if (status){
            outbuff[1] = 1;
          } else{
            outbuff[1] = 0;
          }
          
          break;
        case 65:                       //Init
          Init();
          outbuff[0] = 9;
          outbuff[1] = Stand.BaseAngle1;
          outbuff[2] = Stand.BaseAngle2;
          outbuff[3] = Stand.BaseAngle3;
          outbuff[4] = Stand.BaseAngle4;
          outbuff[5] = Stand.StartNoiseLevel1;
          outbuff[6] = Stand.StartNoiseLevel2;
          outbuff[7] = Stand.MaxSignalLevel1;
          outbuff[8] = Stand.MaxSignalLevel2;
          outbuff[9] = Stand.MaxLaserPower; 
          break;
        case 66:                       //SendMessage
          SendMessage(current.value[0], current.value[1], current.value[2], current.value[3], current.value[4]);
          outbuff[0] = 8;
          outbuff[1] = Stand.Angle1;
          outbuff[2] = Stand.Angle2;
          outbuff[3] = Stand.Angle3;
          outbuff[4] = Stand.Angle4;
          outbuff[5] = Stand.StartNoiseLevel1;
          outbuff[6] = Stand.StartNoiseLevel2;
          outbuff[7] = Stand.NoiseLevel1;
          outbuff[8] = Stand.NoiseLevel2;
          break;
        case 67:                       //SetLaserState
          SetLaserState(current.value[0]);
          delayMicroseconds(100);
          outbuff[0] = 1;
          outbuff[1] = current.value[0];
          //current.condbyte = current.value[0];
          break;
        case 68:                       //SetLaserPower
          SetLaserPower(current.value[0]);
          delayMicroseconds(100);
          Stand.LaserPower = current.value[0];
          outbuff[0] = 1;
          outbuff[1] = Stand.LaserPower;
          break;
        case 69:                       //SetPlateAngle
              
              SetPlateAngle(current.value[0], current.value[1]);
              delayMicroseconds(100);
              switch(current.value[1]){
                case 1:
                  Stand.Angle1 = current.value[0];
                  
                  outbuff[0] = 1;
                  outbuff[1] = Stand.Angle1;
                  
                break;
                case 2:
                  Stand.Angle2 = current.value[0];
                  outbuff[0] = 1;
                  outbuff[1] = Stand.Angle2;
                  
                break;
                case 3:
                  Stand.Angle3 = current.value[0];
                  outbuff[0] = 1;
                  outbuff[1] = Stand.Angle3;
                  
                break;
                case 4:
                  Stand.Angle4 = current.value[0];
                  outbuff[0] = 1;
                  outbuff[1] = Stand.Angle4;
                  
                break;
            default:
              break;
          }
          break;
        case 70:                       //SetTimeout - реализация на уровне API
          SetTimeout(current.value[0]);
          outbuff[0] = 1;
          EEPROM.get(0, outbuff[1]);
          break;
        case 71:                       //GetErrorCode 


          outbuff[0] = 1;
          outbuff[1] = Stand.HardwareState;
          break;
        case 72:                       //GetLaserState
          outbuff[0] = 1;
          outbuff[1] = Stand.LaserState;
          break;
        case 73:                       //GetLaserPower
          outbuff[0] = 1;
          outbuff[1] = Stand.LaserPower;
          break;
        case 74:                       //GetTimeout
          outbuff[0] = 1;
          EEPROM.get(0, outbuff[1]);
          break;
        case 75:                       //GetInitParams (prev: GetStartPlatesAngles)
          outbuff[0] = 9;
          outbuff[1] = Stand.BaseAngle1;
          outbuff[2] = Stand.BaseAngle2;
          outbuff[3] = Stand.BaseAngle3;
          outbuff[4] = Stand.BaseAngle4;
          outbuff[5] = Stand.StartNoiseLevel1;
          outbuff[6] = Stand.StartNoiseLevel2;
          outbuff[7] = Stand.MaxSignalLevel1;
          outbuff[8] = Stand.MaxSignalLevel2;
          outbuff[9] = Stand.MaxLaserPower; 
          break;
        case 76:                       //GetCurPlatesAngles
          outbuff[0] = 4;
          outbuff[1] = Stand.Angle1;
          outbuff[2] = Stand.Angle2;
          outbuff[3] = Stand.Angle3;
          outbuff[4] = Stand.Angle4;
          break;
        case 77:                       //GetSignalLevel
          GetSignalsLevels();
          outbuff[0] = 2;
          outbuff[1] = Stand.NoiseLevel1;
          outbuff[2] = Stand.NoiseLevel2;
          break;
        case 78:                       //GetRotateStep
          outbuff[0] = 1;
          outbuff[1] = Stand.RotateStep;
          break;
        case 79:                       //GetMaxLaserPower
          outbuff[0] = 1;
          outbuff[1] = Stand.MaxLaserPower;
          break;
        case 80:                       //GetLightNoises
          GetLightNoises(); 
          outbuff[0] = 2;
          outbuff[1] = Stand.NoiseLevel1;
          outbuff[2] = Stand.NoiseLevel2;
          break;
        case 81:                       //GetStartLightNoises
          outbuff[0] = 2;
          outbuff[1] = Stand.StartNoiseLevel1;
          outbuff[2] = Stand.StartNoiseLevel2;
          break;
        case 82:                       //GetMaxSignalLevel
          outbuff[0] = 2;
          outbuff[1] = Stand.MaxSignalLevel1;
          outbuff[2] = Stand.MaxSignalLevel2;        
          break;
        case 83:                       //RunSelfTest
          break;
        case 84:                       //InitByButtons 2
          Init2 (current.value[0], current.value[1], current.value[2], current.value[3]);
          outbuff[0] = 9;
          outbuff[1] = Stand.BaseAngle1;
          outbuff[2] = Stand.BaseAngle2;
          outbuff[3] = Stand.BaseAngle3;
          outbuff[4] = Stand.BaseAngle4;
          outbuff[5] = Stand.StartNoiseLevel1;
          outbuff[6] = Stand.StartNoiseLevel2;
          outbuff[7] = Stand.NoiseLevel1;
          outbuff[8] = Stand.NoiseLevel2;
          outbuff[9] = Stand.MaxLaserPower;  
          break; 
        case 85:                       //SetPlatesAngles 
          SetPlatesAngles(current.value[0], current.value[1], current.value[2], current.value[3]);
          outbuff[0] = 4;
          outbuff[1] = Stand.Angle1;
          outbuff[2] = Stand.Angle2;
          outbuff[3] = Stand.Angle3;
          outbuff[4] = Stand.Angle4;
          break;
        case 86:                       // получение данных из ячейки ReadEEPROM (byte)
          outbuff[0] = 1;
          outbuff[1] = EEPROM.read(current.value[0]);
          break;
        case 87:                       // запись данных в ячейку, WriteEEPROM
          EEPROM.put(current.value[0], current.value[1]);
          outbuff[0] = 1;
          outbuff[1] = EEPROM.read(current.value[0]);  
          

          break;
        case 88:                       // Запись в EEPROM базовых углов UpdateBaseAngles
          UpdateBaseAngles(current.value[0], current.value[1], current.value[2], current.value[3]);
          EEPROM.get(2, BaseAngles);
          outbuff[0] = 4;
          outbuff[1] = BaseAngles.Angle1;
          outbuff[2] = BaseAngles.Angle2;
          outbuff[3] = BaseAngles.Angle3;
          outbuff[4] = BaseAngles.Angle4;
          break;
        case 89:                       // Возврат из EEPROM базовых углов ReadBaseAngles
          EEPROM.get(2, BaseAngles);
          outbuff[0] = 4;
          outbuff[1] = BaseAngles.Angle1;
          outbuff[2] = BaseAngles.Angle2;
          outbuff[3] = BaseAngles.Angle3;
          outbuff[4] = BaseAngles.Angle4;
          break;
        default:
          current.condbyte +=1; //Неизвестный ID 8
          ReturnPackage(buff);
          break;
    }
    if(current.condbyte == 0)
    {
      current.condbyte +=1; // Успешное выполнение запроса (отсутствие ошибок)
    }
    
    SendUART();
}

int CreateConfigSecret(){
    for(int i = 0; i < 10; i++){
      Stand.Secret[i] = current.value[i];
    }
    EEPROM.put(20, Stand.Secret);
return 1;
}
void GetConfigSecret(){
    EEPROM.get(20, Stand.Secret);
}

void UpdateBaseAngles(unsigned int angle1, unsigned int angle2, unsigned int angle3, unsigned int angle4){
  BaseAngles.Angle1 = angle1;
  BaseAngles.Angle2 = angle2;
  BaseAngles.Angle3 = angle3;
  BaseAngles.Angle4 = angle4;
  EEPROM.put(2, BaseAngles);
}
// Считывает в буфер пакеты до конца буффера, либо до символа конца пакета

//строит выходной пакет по входным данным

void SetTimeout(int value){
  EEPROM.put(0, value);
}
//Инт в байт, записывает в буфер c numofbyte

void SetPlateAngle(int Steps, unsigned int PlateID){
  switch(PlateID){
    case 1:
      
      do{
        stepper1.setTarget(Steps);
        delay(1);
      }
      while(stepper1.tick());
      Stand.Angle1 = Steps;
      break;
    case 2:
     
      do{
        stepper2.setTarget(Steps);
        delay(1);
      }while(stepper2.tick());
      Stand.Angle2 = Steps;
      break;
    case 3:
      do{
        stepper3.setTarget(Steps);
        delay(1);
      }while(stepper3.tick());
      Stand.Angle3 = Steps;
      break;
    case 4:
      do{
        stepper4.setTarget(Steps);
        delay(1);
      }while(stepper4.tick());
      Stand.Angle4 = Steps;
      break;
    default:
      break;
  }
}
void SetPlatesAngles(int Steps1, int Steps2,  int Steps3,  int Steps4){

  int d1 = Steps1 - Stand.Angle1;
  int d2 = Steps2 - Stand.Angle2;
  int d3 = Steps3 - Stand.Angle3;
  int d4 = Steps4 - Stand.Angle4;

  int a1 = Stand.Angle1;
  int a2 = Stand.Angle2;
  int a3 = Stand.Angle3;
  int a4 = Stand.Angle4;


  if (abs(d1)>600){
    if (d1 > 0){
      stepper1.setCurrent(a1 + 1200);
    }
    else{
      stepper1.setCurrent(a1 - 1200);
    }
  }
  if (abs(d2)>600){
    if (d2 > 0){
      stepper2.setCurrent(a2 + 1200);
    }
    else{
      stepper2.setCurrent(a2 - 1200);
    }
  }
  if (abs(d3)>600){
    if (d3 > 0){
      stepper3.setCurrent(a3 + 1200);
    }
    else{
      stepper3.setCurrent(a3 - 1200);
    }
  }
  if (abs(d4)>600){
    if (d4 > 0){
      stepper4.setCurrent(a4 + 1200);
    }
    else{
      stepper4.setCurrent(a4 - 1200);
    }
  }

  Stand.Angle1 = Steps1;
  Stand.Angle2 = Steps2;
  Stand.Angle3 = Steps3;
  Stand.Angle4 = Steps4;
  
  do{
    delay(1);
    if (!stepper1.tick()){
      stepper1.setTarget(Steps1);
      delayMicroseconds(delayMK);
    }
    if (!stepper2.tick()){
      stepper2.setTarget(Steps2);
      delayMicroseconds(delayMK);
    }
    if (!stepper3.tick()){
      stepper3.setTarget(Steps3);
      delayMicroseconds(delayMK);
    }
    if (!stepper4.tick()){
      stepper4.setTarget(Steps4);
      delayMicroseconds(delayMK);
    }
    }while(stepper1.tick() or stepper2.tick() or stepper3.tick() or stepper4.tick());
}
// увеличивает мощность лазера на 1 пункт
void up(){                            
  digitalWrite(UD, HIGH);             // на U/D подаем единицу
  digitalWrite(INC, HIGH);            // и на INC тоже
  digitalWrite(CS, LOW);              // включаем микросхему
  delayMicroseconds(1);               // ждем
  digitalWrite(INC, LOW);             // дергаем вход INC
  delayMicroseconds(1);
  digitalWrite(INC, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS, HIGH);             // выключаем микросхему и записываем положение в EEPROM 
}
// уменьшает мощность лазера на 1 пункт
void down(){
  digitalWrite(UD, LOW);
  digitalWrite(INC, HIGH);
  digitalWrite(CS, LOW);
  delayMicroseconds(1);
  digitalWrite(INC, LOW);
  delayMicroseconds(1);
  digitalWrite(INC, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS, HIGH);
}
// устанавливает мощность лазера на value при помощи функций up() и down()
void SetLaserPower(unsigned int value){ 
  while(Stand.LaserPower < value){
    up();
    delay(10);
    Stand.LaserPower++;
  } 
  while(Stand.LaserPower > value){
    down();
    delay(10);
    Stand.LaserPower--;
  } 
  return;
}
//Установка значения мощности лазера в 0
void SetLaserPowerDown(){
  SetLaserState(1);
  for(int y=0; y < 100; y++){
    down();
  }
  Stand.LaserPower = 0;
}
// получает теккущие уровни засветки 
void GetSignalsLevels (){
  int16_t adc1;
  int16_t adc2;
  adc1 = (int)ads1.readADC_Differential_0_1();
  Stand.NoiseLevel1 = int(adc1);           // Считываем полученные значения с АЦП с нулевого канала и записываем в переменную.
  delayMicroseconds(2); 
  adc2 = ads2.readADC_Differential_0_1();
  Stand.NoiseLevel2 = int(adc2); 
  
}
//устанавливает состояние лазера 1 - on 0 - off
void SetLaserState(int value){
  if (value != 0){
    digitalWrite(LaserPin, HIGH);
    Stand.LaserState = 1;
  }
  else{
    digitalWrite(LaserPin, LOW);
    Stand.LaserState = 0;
  }
}
// n - количество байт, по которым нужно проверить контрольную сумму

void SendMessage(unsigned int Steps1, unsigned int Steps2, unsigned int Steps3, unsigned int Steps4, unsigned int lpower){
  SetLaserState(1);
  SetLaserPower(lpower); 
  SetPlatesAngles(Steps1, Steps2,  Steps3,  Steps4);
  GetLightNoises();
  GetSignalsLevels();
}
//установка пластин в начальное положение по концевикам
void SetBasePlatesAngles(){
  bool  flag1 = true,      flag2 = true,
        flag3 = true,      flag4 = true;
  int   t1 = Stand.Angle1, t2 = Stand.Angle2,
        t3 = Stand.Angle3, t4 = Stand.Angle3;  
  while(flag1 or flag2 or flag3 or flag4){
    if(analogRead(analogPin1) != 0 && flag1){                     // Шаг двигателя 1
      t1++;
      delayMicroseconds(delayMK);
      if(!stepper1.tick()){
        stepper1.setTarget(t2);
      }
    }
    else{
        stepper1.reset();
        flag1=false;
      }
    if(analogRead(analogPin2) != 0 && flag2){
      t2++;
      delayMicroseconds(100);
      if(!stepper2.tick()){
        stepper2.setTarget(t2);
      }
    }
    else{
        stepper2.reset();
        flag2=false;
      }
    if(analogRead(analogPin3) != 0 && flag3){
        t3++;
        delayMicroseconds(delayMK);
      if(!stepper3.tick()){
        stepper3.setTarget(t3);
      }
    }
    else{
        stepper3.reset();
        flag3=false;
      }
    if(analogRead(analogPin4) != 0 && flag4){
        t4++;
        delayMicroseconds(100);
      if(!stepper4.tick()){
        stepper4.setTarget(t4);
      }
    }
    else{
        stepper4.reset();
        flag4=false;
      }
    delay(1);
    Stand.Angle1 = 0;
    Stand.Angle2 = 0;
    Stand.Angle3 = 0;
    Stand.Angle4 = 0;
  }
}
//установка пластин по алгоритму
void SetBaseAngle(){  
  GetSignalsLevels();
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;
  int MaxSDelta = 0, Delta1, Delta2, SDelta,
      DirPlate2 = 1, RotStep2 = 150,   
      MaxSLevel1 = 0, MaxSLevel2 = Stand.NoiseLevel1, MinSLevel1 = Stand.NoiseLevel1,
      Step4_1 = 0,    Step4_2 = 0, flag = 0, SLevel1, SLevel2;
  SetLaserState(1);
  do{
    do{      
      GetSignalsLevels();
      if(Stand.NoiseLevel1 > MaxSLevel1){
        MaxSLevel1 = Stand.NoiseLevel1;
        Step4_1 = Stand.Angle4; //max
      }
      if(Stand.NoiseLevel1 < MinSLevel1){
        MinSLevel1 = Stand.NoiseLevel1;
        Step4_2 = Stand.Angle4; //min
      }
      SetPlateAngle(Stand.Angle4 + 2, 4);    //Поворот 4 пластины на 1 шаг
    } while (Stand.Angle4 < 600);                //Пока пластина 4 не пройдет 180 градусов (600 шагов)
    
    SDelta = MaxSLevel1 - MinSLevel1;
    SetPlateAngle(0, 4);
    
    if ((MaxSLevel1 - MinSLevel1) > MaxSDelta){ 
      SetPlateAngle(Stand.Angle2 + DirPlate2 * RotStep2, 2);
    }
    else {
      if(RotStep2 > 7){
        RotStep2 = RotStep2 / 2;
         
        DirPlate2 = -DirPlate2;
        SetPlateAngle(Stand.Angle2 + DirPlate2 * RotStep2, 2);
      } 
      else{
        
        flag = 1;
      }
      
    }
    MaxSDelta = MaxSLevel1 - MinSLevel1;
    MaxSLevel1 = MaxSDelta/2;
    MinSLevel1 = MaxSLevel1;
    
  } while(flag != 1);
  SetPlateAngle(Step4_1, 4);
  

  //SetPlateAngle(Step4_1, 4);
  SetPlatesAngles(Stand.Angle1, Stand.Angle2 + 150, Stand.Angle3, Stand.Angle4 + 150); // Поворот пластин 2 и 4 на 150 шагов
  MaxSLevel1 = 0;
  int Step3 = 0;
  do{
  
    GetSignalsLevels();
    SetPlateAngle(Stand.Angle3 + 5, 3);
    if(Stand.NoiseLevel1 > MaxSLevel1){
      MaxSLevel1 = Stand.NoiseLevel1;
      Step3 = Stand.Angle3;
    }    
  } while(Stand.Angle3 < 300); // Пока палстина 3 не повернулась на 90 градусов (не прошла 300 шагов)
  SetPlateAngle(Step3, 3);
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;
}

void SetBaseAngle_v2(){
  int points[3] = {0, 0, 0}; //измеренные уровни сигналов в процессе выполнения процедуры
  int Angles[4] = {0, 0, 0, 0}; //искомые углы волновых пластин
  GetPoints(points, Angles, 0, 0, 1, 1);



  int dAngle; // углы сдвига фазы
  
  if (Get_max_points(points) - Get_min_points(points) < 0.25 * (Get_max_points(points) + Get_min_points(points))){
    Angles[1] += 150; // +45
  }
  dAngle = GetDAngle(Angles, 0, 0, 1, 1);
  Angles[1] += 300; // 90
  Angles[2] += dAngle * 0.5;
  Angles[3] += dAngle; 
  dAngle = GetDAngle(Angles, 0, 0, 1, 1);
  Angles[2] = int(Angles[2] + dAngle * 0.25 - 75); //22.5
  Angles[3] = int(Angles[3] + dAngle * 0.5 - 150); //45
  dAngle = GetDAngle(Angles, 0, 1, 0, 0);
  Angles[1] = Angles[1] + dAngle + 150;
  Angles[3] += 150;

  dAngle = GetDAngle(Angles, 0, 0, 1, 0);
  Angles[2] = Angles[2] + dAngle * 0.5;
  SetPlatesAngles(Angles[0], Angles[1], Angles[2], Angles[3]);
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;



}

void GetPoints(int *points, int Angles[4], byte t0, byte t1, byte t2, byte t3){
  //int points[3] = {0, 0, 0};
  SetPlatesAngles(Angles[0], Angles[1], Angles[2], Angles[3]);
  GetSignalsLevels();
  points[0] = Stand.NoiseLevel1;
  Angles[0] = Angles[0] + 75 * t0;
  Angles[1] = Angles[1] + 150 * t1;
  Angles[2] = Angles[2] + 75 * t2;
  Angles[3] = Angles[3] + 150 * t3;
  SetPlatesAngles(Angles[0], Angles[1], Angles[2], Angles[3]);
  GetSignalsLevels();
  points[1] = Stand.NoiseLevel1;
  Angles[0] = Angles[0] + 75 * t0;
  Angles[1] = Angles[1] + 150 * t1;
  Angles[2] = Angles[2] + 75 * t2;
  Angles[3] = Angles[3] + 150 * t3;
  SetPlatesAngles(Angles[0], Angles[1], Angles[2], Angles[3]);
  GetSignalsLevels();
  points[2] = Stand.NoiseLevel1;
  //return points;

}

int GetDAngle(int Angles[4], byte t0, byte t1, byte t2, byte t3){
  int points[3] = {0, 0, 0};
  int k10, k21, k20,
      dAngle, preset;
  double dDiv, dTan;
  GetPoints(points, Angles, t0, t1, t2, t3);


  k10 = points[1] - points[0];
  k21 = points[2] - points[1];
  k20 = points[2] - points[0];
  int abs_k10 = abs(k10);
  int abs_k21 = abs(k21);
  dDiv = k20/(2*max(abs_k10, abs_k21));
  
  if(dDiv > 0.5){
    preset = 150;
  }
  else if(dDiv < -0.5){
    preset = -150;
  }
  else if(k21 >k10){
    preset = 300;
  }else{
    preset = 0;
  }
  if(preset != 0){
    Angles[0] = Angles[0] + t0 * preset * 0.5;
    Angles[1] = Angles[1] + t1 * preset;
    Angles[2] = Angles[2] + t2 * preset * 0.5;
    Angles[3] = Angles[3] + t3 * preset;
    GetPoints(points, Angles, t0, t1, t2, t3);
    k10 = points[1] - points[0];
    k21 = points[2] - points[1];
    k20 = points[2] - points[0];

    abs_k10 = abs(k10);
    abs_k21 = abs(k21);
    dDiv = k20 / (2 * max(abs_k10, abs_k21));
  }

  if (k10 < k21){
    dAngle = int(-(dDiv + 1) * 150);
  } 
  else {
    dAngle = int ((dDiv + 1) * 150);
  }
  Angles[0] = Angles[0] + t0 * dAngle * 0.5;
  Angles[1] = Angles[1] + t1 * dAngle;
  Angles[2] = Angles[2] + t2 * dAngle * 0.5;
  Angles[3] = Angles[3] + t3 * dAngle;

  //GetPoints(points, Angles, t0, t1, t2, t3);
  //dTan = (points[1] - points[0]) / (points[2] - points[1]);
  //dAngle = int(dAngle + preset - 300 * atan((dTan - 1) / (dTan + 1)) / M_PI);


  return dAngle;
}

//min и max массива из 3 элементов по ссылке на него
int Get_max_points(int *points){
  int res = max(points[0], points[1]);
  return max(res, points[2]);
}
int Get_min_points(int *points){
  int res = min(points[0], points[1]);
  return min(res, points[2]);
}

void SetBaseAngleWSens(){
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;

  int MaxSDelta = 0, Delta1, Delta2, SDelta,
      DirPlate2 = 1, RotStep2 = 150,   
      MaxSLevel1 = 0, MaxSLevel2 = 0,
      Step4_1 = 0,    Step4_2 = 0, flag = 0, SLevel1, SLevel2;
  SetLaserState(1);
  do{
    do{
      GetSignalsLevels();
      if(GetSLevel(1) > MaxSLevel1){
        MaxSLevel1 = GetSLevel(1);
        Step4_1 = Stand.Angle4;
      }
      if( GetSLevel(2) > MaxSLevel2){
        MaxSLevel2 =  GetSLevel(2);
        Step4_2 = Stand.Angle4;
      }
      SetPlateAngle(Stand.Angle4 + 1, 4);    //Поворот 4 пластины на 1 шаг
    } while (Stand.Angle4 < 600);                //Пока пластина 4 не пройдет 180 градусов (600 шагов)

    //Stand.MaxNoiseLevel2 = MaxSLevel2;
    //Stand.MaxNoiseLevel1 = MaxSLevel1;
    //delay(5000);
    SetPlateAngle(Step4_1, 4);                // Установка пластины 4 в положение Step4_1
    GetSignalsLevels();
    Delta1 = abs( GetSLevel(1) -  GetSLevel(2));
    delay(30);
    SetPlateAngle(Step4_2, 4);                // Установка пластины 4 в положение Step4_2
    GetSignalsLevels();
    delay(30);
    Delta2 = abs( GetSLevel(1) -  GetSLevel(2));
    
    SDelta = Delta1 + Delta2;
    SetPlateAngle(0, 4);
    if (SDelta > MaxSDelta){ 
      MaxSDelta = SDelta;
      SetPlateAngle(Stand.Angle2 + DirPlate2 * RotStep2, 2);

    }
    else {
      if(RotStep2 > 7){
        RotStep2 = RotStep2 / 2;
        DirPlate2 = -DirPlate2;
        SetPlateAngle(Stand.Angle2 + DirPlate2 * RotStep2, 2);
      } 
      else{
        flag = 1;
      }
    }
  } while(flag != 1);
  SetPlateAngle(Step4_1,4);
  SetPlatesAngles(Stand.Angle1, Stand.Angle2 + 150, Stand.Angle3, Stand.Angle4 + 150); // Поворот пластин 2 и 4 на 150 шагов
  MaxSLevel1 = 0;
  int Step3 = 0;
  do{
    GetSignalsLevels();
    SetPlateAngle(Stand.Angle3 + 1, 3);
    if( GetSLevel(1) > MaxSLevel1){
      MaxSLevel1 =  GetSLevel(1);
      Step3 = Stand.Angle3;
    }    
  } while(Stand.Angle3 < 300); // Пока палстина 3 не повернулась на 90 градусов (не прошла 300 шагов)
  SetPlateAngle(Step3, 3);
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;
}
double GetSLevel(unsigned int num){
  if (num == 1){
    return ((double)Stand.NoiseLevel1 * Stand.SensPD1)/((double)Stand.NoiseLevel1 * Stand.SensPD1 + (double)Stand.NoiseLevel2 * Stand.SensPD2);
  }
  else{
    return ((double)Stand.NoiseLevel2 * Stand.SensPD2)/((double)Stand.NoiseLevel1 * Stand.SensPD1 + (double)Stand.NoiseLevel2 * Stand.SensPD2);
  }
}
// измеряет уровни засветки
void GetLightNoises(){
  bool flag = false;    //флаг для возвращения лазера в исходное состояние
    if(Stand.LaserState == 1){
    flag = true;    
  }
   // ожидание для реакции фотодетекторов 
  SetLaserState(0);
     
  GetSignalsLevels();
  Stand.StartNoiseLevel1 = Stand.NoiseLevel1;
  Stand.StartNoiseLevel2 = Stand.NoiseLevel2;
  if(flag){
    SetLaserState(1);
    delay(150);
  }
}
void Init(){
  SetLaserPower(Stand.MaxLaserPower);
  GetLightNoises();
  Stand.StartNoiseLevel1 = Stand.NoiseLevel1;
  Stand.StartNoiseLevel2 = Stand.NoiseLevel2;
  SetLaserPowerDown();
  ToLineal();
  //SetBaseAngle_v2(); -- Вторая версия инита, пока нерабочая
  SetBaseAngle();
  FindMaxSignals();
  return;
}

// поиск максимальных показателей на фотодетекторах 
  /*
  Лучше сделай для этого отдельную процедуру, которая будет в Init запускаться самой последней. 
  Если углы устанавливаются корректно, достаточно будет провернуть 1 пластину пошагово с поиском 
  максимумов в диапазоне углов от -9 до +9, потом в диапазоне от 36 до 54.
  Если углы будут с меньшим разбросом находиться, чем сейчас, то диапазоны поиска максимумов можно сузить
  */
//
void FindMaxSignals(){
  SetPlatesAngles(-30, 0, 0, 0);//1170 = 360/0.3 - 9/0.3
  GetSignalsLevels();
  Stand.MaxSignalLevel1 = Stand.NoiseLevel1;
  Stand.MaxSignalLevel2 = Stand.NoiseLevel2;

  
  while(Stand.Angle1 != 30) {//1230 = 360/0.3 + 9/0.3
    GetSignalsLevels();
    
    if(Stand.MaxSignalLevel2 < Stand.NoiseLevel2){
      Stand.MaxSignalLevel2 = Stand.NoiseLevel2;
    }  
    if(Stand.MaxSignalLevel1 < Stand.NoiseLevel1){
      Stand.MaxSignalLevel1 = Stand.NoiseLevel1;
    }
    SetPlatesAngles(Stand.Angle1 + 1, 0, 0, 0);
    
  }
  SetPlatesAngles(120, 0, 0, 0);//120 = 36/0.3
  while(Stand.Angle1 != 180) {//180 = 54/0.3
    GetSignalsLevels();
    if(Stand.MaxSignalLevel2 < Stand.NoiseLevel2){
      Stand.MaxSignalLevel2 = Stand.NoiseLevel2;
    }  
    if(Stand.MaxSignalLevel1 < Stand.NoiseLevel1){
      Stand.MaxSignalLevel1 = Stand.NoiseLevel1;
    }
    SetPlatesAngles(Stand.Angle1 + 1, 0, 0, 0);
    
  }
  SetPlatesAngles(0, 0, 0, 0);

}

void Init2(unsigned int Steps1, unsigned int Steps2, unsigned int Steps3, unsigned int Steps4){
  
  SetLaserPowerDown();
  GetLightNoises();
  SetBasePlatesAngles2();
 
  SetPlatesAngles(Steps1, Steps2, Steps3, Steps4);
  

}
void SetBasePlatesAngles2(){
  //    targets
  bool  flag1 = true,      flag2 = true,
        flag3 = true,      flag4 = true;
  int   t1 = Stand.Angle1, t2 = Stand.Angle2,
        t3 = Stand.Angle3, t4 = Stand.Angle3; 
  while(flag1 or flag2 or flag3 or flag4){
    
    if(analogRead(analogPin1) == 0 && flag1){                     // Шаг двигателя 1
      t1 = -1;
    }
    else{
      t1 = 0;
      flag1=false;
    }
    if(analogRead(analogPin2) == 0 && flag2){                     // Шаг двигателя 1
      
      t2 = -1;
    }
    else{
      t2 = 0;
      
      flag2=false;
    }

    if(analogRead(analogPin3) == 0 && flag3){                     // Шаг двигателя 1
      
      t3 = -1;
    }
    else{
      t3 = 0;
      
      flag3=false;
    }
    
    if(analogRead(analogPin4) == 0 && flag4){                     // Шаг двигателя 1
      
      t4 = -1;
    }
    else{
      t4 = 0;
      
      flag4=false;
    }
    SetPlatesAngles(Stand.Angle1 + t1, Stand.Angle2 + t2, Stand.Angle3 + t3, Stand.Angle4 + t4);

  }

  flag1 = true,      flag2 = true,
        flag3 = true,      flag4 = true;
  while(flag1 or flag2 or flag3 or flag4){

    if(analogRead(analogPin1) != 0 && flag1){                     // Шаг двигателя 1
      
      t1 = 1;
    }
    else{
      t1 = 0;
      flag1=false;
    }
    if(analogRead(analogPin2) != 0 && flag2){                     // Шаг двигателя 1
      
      t2 = 1;
    }
    else{
      t2 = 0;
      flag2=false;
    }

    if(analogRead(analogPin3) != 0 && flag3){                     // Шаг двигателя 1
      
      t3 = 1;
    }
    else{
      t3 = 0;
      
      flag3=false;
    }
    
    if(analogRead(analogPin4) != 0 && flag4){                     // Шаг двигателя 1
      
      t4 = 1;
    }
    else{
      t4 = 0;
      flag4=false;
    }
    SetPlatesAngles(Stand.Angle1 + t1, Stand.Angle2 + t2, Stand.Angle3 + t3, Stand.Angle4 + t4);
  }

  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  Stand.Angle1 = 0;
  Stand.Angle2 = 0;
  Stand.Angle3 = 0;
  Stand.Angle4 = 0;
}
// узнать режим работы фотодетекторов: линейный или нет
bool IsLineal(){
  GetLightNoises();
  while(Stand.NoiseLevel1 < 2 * Stand.NoiseLevel2){
    SetPlateAngle(Stand.Angle1 + 1, 1);
    GetLightNoises();
  }
  int LS2temp = Stand.NoiseLevel2;
  int LS1temp = Stand.NoiseLevel1;

  while(Stand.NoiseLevel1 * 2 > LS1temp){
    SetLaserPower(Stand.LaserPower - 1);
    GetLightNoises();
  }
  double ratio1;
  ratio1 = Stand.NoiseLevel1/LS1temp;
  double ratio2;
  ratio2 = Stand.NoiseLevel2/LS2temp;
  if(abs(ratio1 - ratio2) < 0.05){
    return true;

  }
  return false;
}

bool RotateWhileNotLineal(){
	int Counter = 0;
	while (((double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 > 0.5 or (double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 < 0.3 ) ){
		GetSignalsLevels(); 
		SetPlateAngle(Stand.Angle1 + 1 , 1); 
		Counter++;
		if (Counter == 300) break;
	}
	if((double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 > 0.5 or (double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 < 0.3){
		SetPlateAngle(Stand.Angle2 + 150 , 2);
		RotateWhileNotLineal();
	} 
	if (((double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 > 0.5 or (double)Stand.NoiseLevel1/(double)Stand.NoiseLevel2 < 0.3 ) ){
		return false;
	} else return true;
	
}
	
// Настройка мощности лазера для работы фотодетекторов в линейном режиме
void ToLineal(){
	SetLaserState(1);
	Stand.MaxLaserPower = 80;
	int StartPD1, StartPD2, MaxPD, TempPD1, TempPD2;
	SetLaserPower(Stand.MaxLaserPower);
	GetSignalsLevels();
	int Counter = 0;
	while(!RotateWhileNotLineal()){
		SetLaserPower(Stand.LaserPower - 1);
		
		if(Stand.LaserPower == 0){
			break;
		}
	}
  
  
  StartPD1 = Stand.NoiseLevel1;
  StartPD2 = Stand.NoiseLevel2;
  SetPlateAngle(Stand.Angle1 + 150 , 1);
  GetSignalsLevels();
  if(StartPD2 < Stand.NoiseLevel1){
    SetPlateAngle(Stand.Angle1 + 150 , 1);
    GetSignalsLevels();
  }

  do{
    StartPD1 = Stand.NoiseLevel1;
    StartPD2 = Stand.NoiseLevel2;
    do{
      GetSignalsLevels();
      TempPD1 = Stand.NoiseLevel1;
      TempPD2 = Stand.NoiseLevel2;
      if(2*min(Stand.NoiseLevel1, Stand.NoiseLevel2) > min(StartPD1, StartPD2)){
        SetLaserPower(Stand.LaserPower-1);
        delay(100);
        GetSignalsLevels();
        double PowerAcc = (double)abs((double)TempPD2 - (double)Stand.NoiseLevel2)/(double)Stand.NoiseLevel2;
        while(PowerAcc > 0.05){
          delay(100);
          TempPD1 = Stand.NoiseLevel1;
          TempPD2 = Stand.NoiseLevel2;
          GetSignalsLevels();         
          PowerAcc = (double)abs((double)TempPD2 - (double)Stand.NoiseLevel2)/(double)Stand.NoiseLevel2;
        }
      }
    }while(2*min(Stand.NoiseLevel1, Stand.NoiseLevel2) > min(StartPD1, StartPD2));
    if(abs((double)Stand.NoiseLevel1/(double)StartPD1 - (double)Stand.NoiseLevel2/(double)StartPD2) > 0.05){
      Stand.MaxLaserPower = Stand.MaxLaserPower - 1;
      SetLaserPower(Stand.MaxLaserPower);
      delay(100);
      GetSignalsLevels();
    }
  }while(abs((double)Stand.NoiseLevel1/(double)StartPD1 - (double)Stand.NoiseLevel2/(double)StartPD2) > 0.05 and Stand.LaserPower != 0);
  
  MaxPD = max(StartPD1, StartPD2);
  
}
//метод записывает в структуру стенда сигналы с вычетом засветки
void GetClearSignals(){
  SetLaserState(0);
  GetSignalsLevels();
  Stand.StartNoiseLevel1 = Stand.NoiseLevel1;
  Stand.StartNoiseLevel1 = Stand.NoiseLevel2;  
  SetLaserState(1);
  GetSignalsLevels();  
  Stand.NoiseLevel1 = Stand.NoiseLevel1 - Stand.StartNoiseLevel1;
  Stand.NoiseLevel2 = Stand.NoiseLevel2 - Stand.StartNoiseLevel2;
  return;
}
void GetSens(){
  int SensPD1, SensPD2, StartPD1, StartPD2, AllPowerPD1, AllPowerPD2;
  SetLaserPower(Stand.MaxLaserPower);
  GetSignalsLevels();
  StartPD1 = Stand.NoiseLevel1;
  StartPD2 = Stand.NoiseLevel2;
  SetPlateAngle(Stand.Angle1 + 150 , 1);
  GetSignalsLevels();
  AllPowerPD1 = StartPD1 + Stand.NoiseLevel1;
  AllPowerPD2 = StartPD2 + Stand.NoiseLevel2;
  Stand.SensPD1 = (double)AllPowerPD1/max((double)AllPowerPD1, (double)AllPowerPD2);
  Stand.SensPD2 = (double)AllPowerPD2/max((double)AllPowerPD1, (double)AllPowerPD2);
  Serial.print(StartPD1);
  Serial.print(StartPD2);
  Serial.print(AllPowerPD1);
  Serial.print(AllPowerPD2);
}


// In Progress
void RunTests(){
  bool laser_test = TestLaser();


}

// In Progress
bool TestLaser(){
  SetLaserPower(0);
  SetLaserState(0);
  SetLaserState(1);
  SetLaserPower(100);
  GetSignalsLevels();
  if(Stand.NoiseLevel1 > 16 || Stand.NoiseLevel2 > 16){
    return true;
  }
  else{
    return false;
  }
  
}
// In Progress
bool TestPlates(){
  SetLaserPower(0);
  SetLaserState(0);
  SetLaserState(1);
  SetLaserPower(60);
  GetSignalsLevels();
  int signal1 = Stand.NoiseLevel1, signal2 = Stand.NoiseLevel2;
  SetPlatesAngles(0,0,0,0);

}
