//MASTER
#include <SPI.h>
#include "MPU9250.h"
#include <esp_now.h>
#include <WiFi.h>
//#include "BluetoothSerial.h"

//BluetoothSerial bt;
#define RXD_RS485 33
#define TXD_RS485 25
#define RS485_E 26

int state;
int print_count;
int yaw_print[20];
int pitch_print[20];
int roll_print[20];
////////////////////////////////////////////////////////////////////////////////////////////////////////////////WiFi
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char data_print[50];
char data_print_bt[50];

typedef struct struct_message {
    int id; // must be unique for each sender board
    float q0;
    float q1;
    float q2;
    float q3;
} struct_message;

//Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;

// Create an array with all the structures
struct_message boardsStruct[4] = {board1, board2, board3, board4};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(RS485_E, HIGH);
  print_count++;
  
  boardsStruct[myData.id-1].q0 = myData.q0;
  boardsStruct[myData.id-1].q1 = myData.q1;
  boardsStruct[myData.id-1].q2 = myData.q2;
  boardsStruct[myData.id-1].q3 = myData.q3;
  
  sprintf(data_print, "s%d%03d,%03d,%03d,%03d", myData.id, (int)(((boardsStruct[myData.id-1].q0)+1)*100), (int)(((boardsStruct[myData.id-1].q1)+1)*100), (int)(((boardsStruct[myData.id-1].q2)+1)*100), (int)(((boardsStruct[myData.id-1].q3)+1)*100));
  Serial.println(data_print);
  
  roll_print[myData.id] = atan2f(boardsStruct[myData.id-1].q0*boardsStruct[myData.id-1].q1 + boardsStruct[myData.id-1].q2*boardsStruct[myData.id-1].q3, 0.5f - boardsStruct[myData.id-1].q1*boardsStruct[myData.id-1].q1 - boardsStruct[myData.id-1].q2*boardsStruct[myData.id-1].q2) * 57.29578f;
  pitch_print[myData.id] = asinf(-2.0f * (boardsStruct[myData.id-1].q1*boardsStruct[myData.id-1].q3 - boardsStruct[myData.id-1].q0*boardsStruct[myData.id-1].q2)) * 57.29578f;
  yaw_print[myData.id] = atan2f(boardsStruct[myData.id-1].q1*boardsStruct[myData.id-1].q2 + boardsStruct[myData.id-1].q0*boardsStruct[myData.id-1].q3, 0.5f - boardsStruct[myData.id-1].q2*boardsStruct[myData.id-1].q2 - boardsStruct[myData.id-1].q3*boardsStruct[myData.id-1].q3) * 57.29578f + 180.0f;
  sprintf(data_print_bt, "s%03d,%03d,%03d,%03d,%03d,%03d", yaw_print[1]+500, pitch_print[1]+500, roll_print[1]+500, yaw_print[2]+500, pitch_print[2]+500, roll_print[2]+500);
  if(print_count == 5)
  {
    Serial1.println(data_print_bt);    
    Serial2.println(data_print_bt);
    print_count = 0;
  }

/*
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes", myData.id, len);
  Serial.println();
  // Update the structures with the new incoming data

  boardsStruct[myData.id-1].q0 = myData.q0;
  boardsStruct[myData.id-1].q1 = myData.q1;
  boardsStruct[myData.id-1].q2 = myData.q2;
  boardsStruct[myData.id-1].q3 = myData.q3;
  Serial.print("q0 value: ");
  Serial.print(boardsStruct[myData.id-1].q0, 6);
  Serial.print(", ");
  Serial.print("q1 value: ");
  Serial.print(boardsStruct[myData.id-1].q1, 6);
  Serial.print(", ");  
  Serial.print("q2 value: ");
  Serial.print(boardsStruct[myData.id-1].q2, 6);
  Serial.print(", ");
  Serial.print("q3 value: ");
  Serial.print(boardsStruct[myData.id-1].q3, 6);
  Serial.println(); 
*/  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////Timer
/*
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  bt.println(data_print_bt);
  portEXIT_CRITICAL_ISR(&timerMux);
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define CHIP_SELECT_PIN         13

#define IMU_POLL_DELAY_MS         2
#define MFILTER_SAMPLE_FREQ_HZ    480
#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_184HZ

#define CALIB_DISABLE
#define CALIB_DISABLE
//#define ACC_CALIB_DONE
//#define ACC_CALIB_Disable
//#define MAG_CALIB_DONE
#define MAG_CALIB_DISABLE

float q_value[3][4];
const byte gpio_input = 5;
int count;
#define RXD1 34
#define TXD1 32
//#define RXD1 11
//#define TXD1 34

#define ACC 0xF0
#define MAG 0xF1
#define GYRO 0xF2

String data_read;

int revision(float GYRO_X, float GYRO_Y, float GYRO_Z, float ACC_X_H, float ACC_X_L, float ACC_Y_H, float ACC_Y_L, float ACC_Z_H, float ACC_Z_L, float MAG_X_H, float MAG_X_L, float MAG_Y_H, float MAG_Y_L, float MAG_Z_H, float MAG_Z_L)
{
  unsigned long settime = millis();
  while(1)
  {
    Send_revision(GYRO, GYRO_X, GYRO_X, GYRO_Y, GYRO_Y, GYRO_Z, GYRO_Z);
    delay(50);
    Send_revision(ACC, ACC_X_H, ACC_X_L, ACC_Y_H, ACC_Y_L, ACC_Z_H, ACC_Z_L);
    delay(50);
    Send_revision(MAG, MAG_X_H, MAG_X_L, MAG_Y_H, MAG_Y_L, MAG_Z_H, MAG_Z_L);
    delay(50);
    if(((millis() - settime) > 5000)){
      return 1;
    }
  }  
}

void Send_revision(byte id, float X_H_data, float X_L_data, float Y_H_data, float Y_L_data, float Z_H_data, float Z_L_data)
{
  typedef struct revision_message {
      int id;
      float X_H;
      float X_L;
      float Y_H;
      float Y_L;
      float Z_H;
      float Z_L;
  } struct_message;
  revision_message myrevisionData;

  myrevisionData.id = id;
  myrevisionData.X_H = X_H_data;
  myrevisionData.X_L = X_L_data;
  myrevisionData.Y_H = Y_H_data;
  myrevisionData.Y_L = Y_L_data;
  myrevisionData.Z_H = Z_H_data;
  myrevisionData.Z_L = Z_L_data;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myrevisionData, sizeof(myrevisionData));
  if (result == ESP_OK) {
    Serial.print(id);
    Serial.println(", Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }  
}

MPU9250 IMU(SPI, gpio_input);
Madgwick AHRSFilter;

int status;

typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
  };
  uint8_t valArray[sizeof(float) * 3];
} NotifBytesBunch;

NotifBytesBunch AHRSValues;

void setup() {
  Serial.begin(2000000);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    Serial2.begin(115200, SERIAL_8N1, RXD_RS485, TXD_RS485);
//  bt.begin("ESP32_SIBAR");  

//  timer = timerBegin(0, 80, true);
//  timerAttachInterrupt(timer, &onTimer, true);
//  timerAlarmWrite(timer, 500000, true);  //1s = 1000000, 1ms = 10000 0.1ms = 10000 

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  pinMode(RS485_E, OUTPUT);
/*  
  while (!Serial) {}
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(IMU_LOWPASSFILTER_BANDWIDTH);

  while (! Serial.available() ) {
      Serial.println("Start calibration");
      delay(10);
  }
   while (Serial.available() ) {
      Serial.read();    
      delay(5);
    }
  Serial.println(F("********** GYRO calib **************"));
  float gyrodir[3][3];
  for (uint8_t num = 0; num < 2; num++) {
    
    Serial.print(F("Don't move until I tell you.  "));
    Serial.print(num+1);
    Serial.println("...");  
    delay(500);
    state = IMU.calibrateGyro();
    if(state == 1){
      Serial.println(F("GYRO calib done"));
    }
    else{
      Serial.println(F("GYRO calib fail"));
    }
    Serial.println(F("Vals: "));
    Serial.print(F("X: "));
    gyrodir[0][num] = IMU.getGyroBiasX_rads();
    Serial.print(gyrodir[0][num], 6);
    Serial.println("rad/s");
    Serial.print(F("Y: "));
    gyrodir[1][num] = IMU.getGyroBiasY_rads();
    Serial.print(gyrodir[1][num], 6);
    Serial.println("rad/s");
    Serial.print(F("Z: "));
    gyrodir[2][num] = IMU.getGyroBiasZ_rads();
    Serial.print(gyrodir[2][num], 6);
    Serial.println("rad/s");
    if(num == 1){
      gyrodir[0][2] = (gyrodir[0][0] + gyrodir[0][1])/2;
      gyrodir[1][2] = (gyrodir[1][0] + gyrodir[1][1])/2;
      gyrodir[2][2] = (gyrodir[2][0] + gyrodir[2][1])/2;
      Serial.println(F("GYRO calib ALL done"));
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      Serial.print(gyrodir[0][2], 6);
      Serial.println("rad/s");
      
      Serial.print(F("Y: "));
      Serial.print(gyrodir[1][2], 6);
      Serial.println("rad/s");
      
      Serial.print(F("Z: "));
      Serial.print(gyrodir[2][2], 6);      \
      Serial.println("rad/s");
      Serial.println();
      }
    }

  Serial.println(F("********** ACC calib **************"));
  float accdir[6][3];
  char * dirs[6] = { "X+", "X-", "Y+", "Y-", "Z+", "Z-"};

  for (uint8_t num = 0; num < 2; num++) {
      for (uint8_t i = 0; i < 6; i++) {
        Serial.print(F("Enter "));
        Serial.print((int)(num + 1));
        Serial.print(F(" when ready for dir "));
        Serial.print((int)(i + 1));
        Serial.print(' ');
        Serial.print(dirs[i]);
        while (! Serial.available() ) {
          delay(10);
        }
    
        while (Serial.available()) {
          Serial.read();
          delay(10);
          Serial.print('.');
        }
        Serial.println();
        state = IMU.calibrateAccel();
      }
      if(state == 1){
        Serial.println(F("Acc calib done"));
      }
      else{
        Serial.println(F("Acc calib fail"));        
      }
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      accdir[0][num] = IMU.getAccelBiasX_mss();      
      Serial.print(IMU.getAccelBiasX_mss(), 6);
      Serial.print("m/s^2");
      Serial.print(" / ");
      accdir[1][num] = IMU.getAccelScaleFactorX();      
      Serial.print(IMU.getAccelScaleFactorX(), 6);
      Serial.println("m/s^2");
      Serial.print(F("Y: "));
      accdir[2][num] = IMU.getAccelBiasY_mss();
      Serial.print(IMU.getAccelBiasY_mss(), 6);
      Serial.print("m/s^2");      
      Serial.print(" / ");
      accdir[3][num] = IMU.getAccelScaleFactorY();      
      Serial.print(IMU.getAccelScaleFactorY(), 6);
      Serial.println("m/s^2");
      Serial.print(F("Z: "));
      accdir[4][num] = IMU.getAccelBiasZ_mss();
      Serial.print(IMU.getAccelBiasZ_mss(), 6);      
      Serial.print("m/s^2");
      Serial.print(" / ");
      accdir[5][num] = IMU.getAccelScaleFactorZ();      
      Serial.print(IMU.getAccelScaleFactorZ(), 6);
      Serial.println("m/s^2");      
      if(num == 1){
        accdir[0][2] = (accdir[0][0] + accdir[0][1])/2;
        accdir[1][2] = (accdir[1][0] + accdir[1][1])/2;
        accdir[2][2] = (accdir[2][0] + accdir[2][1])/2;
        accdir[3][2] = (accdir[3][0] + accdir[3][1])/2;
        accdir[4][2] = (accdir[4][0] + accdir[4][1])/2;
        accdir[5][2] = (accdir[5][0] + accdir[5][1])/2;
        
        Serial.println(F("Acc calib ALL done"));
        Serial.println(F("Vals: "));
        Serial.print(F("X: "));
        Serial.print(accdir[0][2], 6);
        Serial.print("m/s^2");
        Serial.print(" / ");
        Serial.print(accdir[1][2], 6);
        Serial.println("m/s^2");
        
        Serial.print(F("Y: "));
        Serial.print(accdir[2][2], 6);
        Serial.print("m/s^2");        
        Serial.print(" / ");
        Serial.print(accdir[3][2], 6);
        Serial.println("m/s^2");
        
        Serial.print(F("Z: "));
        Serial.print(accdir[4][2], 6);
        Serial.print("m/s^2");        
        Serial.print(" / ");
        Serial.print(accdir[5][2], 6);
        Serial.println("m/s^2");
        Serial.println();        
      }
  }

  Serial.println(F("********** MAG calib **************"));  
  float magdir[6][3];
  for (uint8_t num = 0; num < 2; num++) {
    Serial.print(F("CALIB MAG "));
    Serial.print((int)(num + 1));
    Serial.println(F(" -- move in figure 8s until I say stop!!! "));
    delay(500);
    state = IMU.calibrateMag();
    Serial.print((int)(num + 1));    
    if(state == 1){
        Serial.println(F("Mag calib done"));
    }
    else{
        Serial.println(F("Mag calib fail"));
    }
    Serial.println(F("Vals: "));    
    Serial.print(F("X: "));
    magdir[0][num] = IMU.getMagBiasX_uT();
    Serial.print(magdir[0][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[1][num] = IMU.getMagScaleFactorX();
    Serial.print(magdir[1][num], 6);
    Serial.println("uT");    
    Serial.print(F("Y: "));
    magdir[2][num] = IMU.getMagBiasY_uT();
    Serial.print(magdir[2][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[3][num] = IMU.getMagScaleFactorY();
    Serial.print(magdir[3][num], 6);
    Serial.println("uT");        
    Serial.print(F("Z: "));
    magdir[4][num] = IMU.getMagBiasZ_uT();
    Serial.print(magdir[4][num], 6);
    Serial.print("uT");
    Serial.print(" / ");
    magdir[5][num] = IMU.getMagScaleFactorZ();
    Serial.print(magdir[5][num], 6);
    Serial.println("uT");        
    if (num == 1) {
      magdir[0][2] = (magdir[0][0] + magdir[0][1]) / 2;
      magdir[1][2] = (magdir[1][0] + magdir[1][1]) / 2;
      magdir[2][2] = (magdir[2][0] + magdir[2][1]) / 2;
      magdir[3][2] = (magdir[3][0] + magdir[3][1]) / 2;
      magdir[4][2] = (magdir[4][0] + magdir[4][1]) / 2;
      magdir[5][2] = (magdir[5][0] + magdir[5][1]) / 2;
      Serial.println(F("Mag calib ALL done"));
      Serial.println(F("Vals: "));      
      Serial.print(F("X: "));
      Serial.print(magdir[0][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[1][2], 6);
      Serial.println("uT");          
      Serial.print(F("Y: "));
      Serial.print(magdir[2][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[3][2], 6);
      Serial.println("uT");          
      Serial.print(F("Z: "));
      Serial.print(magdir[4][2], 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(magdir[5][2], 6);
      Serial.println("uT");          
    }
  }
  AHRSFilter.begin(MFILTER_SAMPLE_FREQ_HZ);

  IMU.setGyroBiasX_rads(gyrodir[0][2]);
  IMU.setGyroBiasY_rads(gyrodir[1][2]);
  IMU.setGyroBiasZ_rads(gyrodir[2][2]);

  IMU.setAccelCalX(accdir[0][2], accdir[1][2]);
  IMU.setAccelCalY(accdir[2][2], accdir[3][2]);
  IMU.setAccelCalZ(accdir[4][2], accdir[5][2]);
  
  IMU.setMagCalX(magdir[0][2], magdir[1][2]);
  IMU.setMagCalY(magdir[2][2], magdir[3][2]);
  IMU.setMagCalZ(magdir[4][2], magdir[5][2]);
//  revision(accdir[0][2], accdir[1][2], accdir[2][2], accdir[3][2], accdir[4][2], accdir[5][2], magdir[0][2], magdir[1][2], magdir[2][2], magdir[3][2], magdir[4][2], magdir[5][2]);
  revision(gyrodir[0][2], gyrodir[1][2], gyrodir[2][2], accdir[0][2], accdir[1][2], accdir[2][2], accdir[3][2], accdir[4][2], accdir[5][2], magdir[0][2], magdir[1][2], magdir[2][2], magdir[3][2], magdir[4][2], magdir[5][2]);
*/  
  esp_now_register_recv_cb(OnDataRecv);
//  timerAlarmEnable(timer);
}
float past_gx;
float past_gy;
float past_gz;

uint8_t loopCount = 0;
unsigned long ret_time;
void loop() {
  
}
