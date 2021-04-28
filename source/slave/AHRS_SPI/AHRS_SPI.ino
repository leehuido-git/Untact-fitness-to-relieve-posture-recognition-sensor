//SLAVE
#include <SPI.h>
#include "MPU9250.h"
#include <esp_now.h>
#include <WiFi.h>

#define CHIP_SELECT_PIN         13

#define IMU_POLL_DELAY_MS         2
#define MFILTER_SAMPLE_FREQ_HZ    480
#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_184HZ

const byte gpio_input = 5;

int use_IMU = 2;

#define GYRO 0xF2
#define ACC 0xF0
#define MAG 0xF1

int count;
char data_print[50];

//////////////////////////////////////////////////////////////////////////////////////////////////////////WiFi
typedef struct struct_revision_message {
  int id;
  float X_H;
  float X_L;
  float Y_H;
  float Y_L;
  float Z_H;
  float Z_L;
}struct_revision_message;

// Create a struct_message called myData
struct_revision_message myData_revision;

//struct_revision_message board;
//struct_revision_message boardStruct[1] = {board};

void OnDataRecv_revision(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  memcpy(&myData_revision, incomingData, sizeof(myData_revision));  
}

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
    int id; // must be unique for each sender board
    float q0;
    float q1;
    float q2;
    float q3;
} struct_message;

//Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
  Serial.print("ID: ");
  Serial.println(use_IMU);
  Recive_revision();

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
}

uint8_t loopCount = 0;

void loop() {
  IMU.readSensor(AHRSFilter);
  if (loopCount++ > 10) {
    loopCount = 0;
    float rolly = AHRSFilter.getRoll();
    if (rolly < 0) {
      // I like them positive
      rolly = 360.0 + rolly;
    }
    AHRSValues.roll = rolly;
    AHRSValues.pitch = AHRSFilter.getPitch();
    AHRSValues.yaw = AHRSFilter.getYaw();
    
    myData.id = use_IMU;
    myData.q0 =  AHRSFilter.getq0();
    myData.q1 = AHRSFilter.getq1();
    myData.q2 = AHRSFilter.getq2();
    myData.q3 = AHRSFilter.getq3();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  delay(IMU_POLL_DELAY_MS);
}

int Recive_revision(void)
{
  int GYRO_CHECK = 0;
  int ACC_CHECK = 0;  
  int MAG_CHECK = 0;  
  esp_now_register_recv_cb(OnDataRecv_revision);
  while(1)
  {
    if((myData_revision.id == GYRO) && !GYRO_CHECK)
    {
      IMU.setGyroBiasX_rads(myData_revision.X_H);
      IMU.setGyroBiasY_rads(myData_revision.Y_H);
      IMU.setGyroBiasZ_rads(myData_revision.Z_H);
      Serial.println(F("********** GYRO **************"));
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      Serial.print(myData_revision.X_H, 6);
      Serial.println("rad/s");
      Serial.print(F("Y: "));
      Serial.print(myData_revision.Y_H, 6);
      Serial.println("rad/s");
      Serial.print(F("Z: "));
      Serial.print(myData_revision.Z_H, 6);
      Serial.println("rad/s");
      Serial.println();
      GYRO_CHECK = 1;
    }
    else if((myData_revision.id == ACC) && !ACC_CHECK)
    {
      IMU.setAccelCalX(myData_revision.X_H, myData_revision.X_L);
      IMU.setAccelCalY(myData_revision.Y_H, myData_revision.Y_L);
      IMU.setAccelCalZ(myData_revision.Z_H, myData_revision.Z_L);
      Serial.println(F("********** ACC  **************"));
      Serial.println(F("Vals: "));
      Serial.print(F("X: "));
      Serial.print(myData_revision.X_H, 6);
      Serial.print("m/s^2");
      Serial.print(" / ");
      Serial.print(myData_revision.X_L, 6);
      Serial.println("m/s^2");
      Serial.print(F("Y: "));
      Serial.print(myData_revision.Y_H, 6);
      Serial.print("m/s^2");        
      Serial.print(" / ");
      Serial.print(myData_revision.Y_L, 6);
      Serial.println("m/s^2");
      Serial.print(F("Z: "));
      Serial.print(myData_revision.Z_H, 6);
      Serial.print("m/s^2");        
      Serial.print(" / ");
      Serial.print(myData_revision.Z_L, 6);
      Serial.println("m/s^2");
      Serial.println();
      ACC_CHECK = 1;
    }
    else if((myData_revision.id == MAG) && !MAG_CHECK)
    {
      IMU.setMagCalX(myData_revision.X_H, myData_revision.X_L);
      IMU.setMagCalY(myData_revision.Y_H, myData_revision.Y_L);
      IMU.setMagCalZ(myData_revision.Z_H, myData_revision.Z_L);
      Serial.println(F("********** MAG  **************"));        
      Serial.println(F("Vals: "));      
      Serial.print(F("X: "));
      Serial.print(myData_revision.X_H, 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(myData_revision.X_L, 6);
      Serial.println("uT");          
      Serial.print(F("Y: "));
      Serial.print(myData_revision.Y_H, 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(myData_revision.Y_L, 6);
      Serial.println("uT");          
      Serial.print(F("Z: "));
      Serial.print(myData_revision.Z_H, 6);
      Serial.print("uT");
      Serial.print(" / ");
      Serial.print(myData_revision.Z_L, 6);
      Serial.println("uT");             
      MAG_CHECK = 1;
    }
    if(GYRO_CHECK && ACC_CHECK && MAG_CHECK){
      return 1;
    }
  }
}
