/**
 * A simple Azure IoT example for sending telemetry.
 * Watchdog
 * Interupt on GPIO
 * Read almost all parameters from M5StickC
 * Commands: start | stop, ledon | ledoff, delay {"ms":1000}
 */
#include <Arduino.h>
#include <M5StickC.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "Esp32MQTTClient.h"
#include "esp32_rmt.h"
#include "esp_task_wdt.h"

//extern static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
// Please input the SSID and password of WiFi
char *ssid = "xxxxxxxxxxxx";
char *password = "xxxxxxxxxxxx";

static const char *connectionString = "xxxxxxxxxxxx";

static bool hasIoTHub = false;

int messageCount = 1;
static bool hasWifi = false;
static bool messageSending = true;
static uint64_t send_interval_ms;

int IntervalMs = 10000;//10000;
#define DEVICE_ID "m5stickc01"
#define MESSAGE_MAX_LEN 512

ESP32_RMT rem;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
static int callbackCounter;
IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
IOTHUB_MESSAGE_HANDLE msg;
int receiveContext = 0;
static char propText[1024];

static IOTHUBMESSAGE_DISPOSITION_RESULT ReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void *userContextCallback)
{
  int *counter = (int *)userContextCallback;
  const char *buffer;
  size_t size;
  MAP_HANDLE mapProperties;
  const char *messageId;
  const char *correlationId;
  const char *userDefinedContentType;
  const char *userDefinedContentEncoding;

  // Message properties
  if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
  {
    messageId = "<null>";
  }

  if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
  {
    correlationId = "<null>";
  }

  if ((userDefinedContentType = IoTHubMessage_GetContentTypeSystemProperty(message)) == NULL)
  {
    userDefinedContentType = "<null>";
  }

  if ((userDefinedContentEncoding = IoTHubMessage_GetContentEncodingSystemProperty(message)) == NULL)
  {
    userDefinedContentEncoding = "<null>";
  }

  // Message content
  if (IoTHubMessage_GetByteArray(message, (const unsigned char **)&buffer, &size) != IOTHUB_MESSAGE_OK)
  {
    (void)printf("unable to retrieve the message data\r\n");
  }
  else
  {
    (void)printf("Received Message [%d]\r\n Message ID: %s\r\n Correlation ID: %s\r\n Content-Type: %s\r\n Content-Encoding: %s\r\n Data: <<<%.*s>>> & Size=%d\r\n",
                 *counter, messageId, correlationId, userDefinedContentType, userDefinedContentEncoding, (int)size, buffer, (int)size);
  }

  // Retrieve properties from the message
  mapProperties = IoTHubMessage_Properties(message);
  if (mapProperties != NULL)
  {
    const char *const *keys;
    const char *const *values;
    size_t propertyCount = 0;
    if (Map_GetInternals(mapProperties, &keys, &values, &propertyCount) == MAP_OK)
    {
      if (propertyCount > 0)
      {
        size_t index;

        printf(" Message Properties:\r\n");
        for (index = 0; index < propertyCount; index++)
        {
          (void)printf("\tKey: %s Value: %s\r\n", keys[index], values[index]);
        }
        (void)printf("\r\n");
      }
    }
  }

  /* Some device specific action code goes here... */
  (*counter)++;
  return IOTHUBMESSAGE_ACCEPTED;
}

static int DeviceMethodCallback(const char *method_name, const unsigned char *payload, size_t size, unsigned char **response, size_t *resp_size, void *userContextCallback)
{
  (void)userContextCallback;

  printf("\r\nDevice Method called\r\n");
  printf("Device Method name:    %s\r\n", method_name);
  printf("Device Method payload: %.*s\r\n", (int)size, (const char *)payload);

  int status = 200;
  char *RESPONSE_STRING = "{ \"Response\": \"OK\" }";

    if (strcmp(method_name, "start") == 0)
    {
      messageSending = true;
    }
    else if (strcmp(method_name, "stop") == 0)
    {
      messageSending = false;
    }
    else if (strcmp(method_name, "delay") == 0)
    {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);

      // Test if parsing succeeds.
      if (error)
      {
        LogError("deserializeJson() failed: ");
        LogError(error.c_str());
        RESPONSE_STRING = "\"deserializeJson() failed\"";
        status = 500;
      }
      else
      {
        IntervalMs = doc["ms"];
        LogInfo("IntervalMs:%d", IntervalMs);
      }
    }
    else if (strcmp(method_name, "ledon") == 0)
    {
      digitalWrite(M5_LED, LOW);
    }
    else if (strcmp(method_name, "ledoff") == 0)
    {
      digitalWrite(M5_LED, HIGH);
    }
    else
    {
      LogInfo("No method %s found", method_name);
      RESPONSE_STRING = "\"No method found\"";
      status = 404;
    }


  printf("\r\nResponse status: %d\r\n", status);
  printf("Response payload: %s\r\n\r\n", RESPONSE_STRING);

  *resp_size = strlen(RESPONSE_STRING);
  if ((*response = (unsigned char *)malloc(*resp_size)) == NULL)
  {
    status = -1;
  }
  else
  {
    (void)memcpy(*response, RESPONSE_STRING, *resp_size);
  }
  return status;
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *userContextCallback)
{
  //Setup watchdog
  esp_task_wdt_reset();
  //Passing address to IOTHUB_MESSAGE_HANDLE
  IOTHUB_MESSAGE_HANDLE *msg = (IOTHUB_MESSAGE_HANDLE *)userContextCallback;
  (void)printf("Confirmation %d result = %s\r\n", callbackCounter, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
  /* Some device specific action code goes here... */
  callbackCounter++;
  if (result != IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    esp_restart();
  }

  //TK:Or caller
  IoTHubMessage_Destroy(*msg);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
double vbat = 0.0;
double discharge, charge;
double temp = 0.0;
double bat_p = 0.0;
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
int16_t tempg = 0;
int16_t coIn = 0, coOut = 0;
double coD = 0, vin = 0, iin = 0;
volatile int16_t bRST = 0, bHOME = 0; //Interupt



void IRAM_ATTR isrHOME() {
  bHOME = 1;
}

void IRAM_ATTR isrRST() {
  bRST = 1;
}

void setup()
{
  // initialize the M5StickC object
  M5.begin();
  M5.Axp.begin();
  M5.Axp.EnableCoulombcounter();
  M5.Imu.Init();
  M5.Lcd.begin();

  rem.begin(M5_IR, true);


  pinMode(M5_BUTTON_HOME, INPUT_PULLUP);
  attachInterrupt(M5_BUTTON_HOME, isrHOME, FALLING);
  pinMode(M5_BUTTON_RST, INPUT_PULLUP);
  attachInterrupt(M5_BUTTON_RST, isrRST, FALLING);

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);

  M5.Lcd.setCursor(0, 0, 1);
  M5.Lcd.println("WiFi");
  Serial.println("Starting connecting WiFi.");
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    M5.Lcd.print(".");
  }
  hasWifi = true;
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  M5.Lcd.println(WiFi.localIP());

  randomSeed(analogRead(0));

  Serial.println(" > IoT Hub");

  printf("Before platform_init\n");
  if (platform_init() != 0)
  {
    (void)printf("Failed to initialize the platform.\r\n");
    M5.Lcd.println("IoT Hub - error");
  }
  else
  {
    printf("Before IoTHubClient_LL_CreateFromConnectionString\n");
    if ((iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol)) == NULL)
    {
      (void)printf("ERROR: iotHubClientHandle is NULL!\r\n");
    }
    else
    {
      if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, ReceiveMessageCallback, &receiveContext) != IOTHUB_CLIENT_OK)
      {
        (void)printf("ERROR: IoTHubClient_LL_SetMessageCallback..........FAILED!\r\n");
      }

      if (IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, DeviceMethodCallback, &receiveContext) != IOTHUB_CLIENT_OK)
      {
        (void)printf("ERROR: IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!\r\n");
      }
    }
  }

  hasIoTHub = true;

  M5.Lcd.println("IoT Hub - OK!");
  delay(2000);
  send_interval_ms = millis();
}

void loop()
{

  if (hasWifi && hasIoTHub)
  {
    if (messageSending &&
        (int)(millis() - send_interval_ms) >= IntervalMs)
    {
      //Read
      vbat = M5.Axp.GetVbatData() * 1.1 / 1000;
      charge = M5.Axp.GetIchargeData() / 2;
      discharge = M5.Axp.GetIdischargeData() / 2;
      temp = -144.7 + M5.Axp.GetTempData() * 0.1;
      bat_p = M5.Axp.GetPowerbatData() * 1.1 * 0.5 / 1000;
      coIn = M5.Axp.GetCoulombchargeData();
      coOut = M5.Axp.GetCoulombdischargeData();
      coD = M5.Axp.GetCoulombData();
      vin = M5.Axp.GetVinData() * 1.7;
      iin = M5.Axp.GetIinData() * 0.625;

      M5.Lcd.setCursor(0, 0, 1);
      M5.Lcd.printf("vbat:%.3fV\r\n", vbat);
      M5.Lcd.printf("icharge:%fmA\r\n", charge);
      M5.Lcd.printf("idischg:%fmA\r\n", discharge);
      M5.Lcd.printf("temp:%.1fC\r\n", temp);
      M5.Lcd.printf("pbat:%.3fmW\r\n", bat_p);
      M5.Lcd.printf("CoIn :%d\r\n", coIn);
      M5.Lcd.printf("CoOut:%d\r\n", coOut);
      M5.Lcd.printf("CoD:%.2fmAh\r\n", coD);
      M5.Lcd.printf("Vin:%.3fmV\r\n", vin);
      M5.Lcd.printf("Iin:%.3fmA\r\n", iin);

      M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
      M5.IMU.getAccelData(&accX, &accY, &accZ);
      M5.IMU.getTempData(&tempg);
      M5.Lcd.printf("%.2f|%.2f\r\n%.2f\r\n", ((float)gyroX) * M5.IMU.gRes, ((float)gyroY) * M5.IMU.gRes, ((float)gyroZ) * M5.IMU.gRes);
      M5.Lcd.printf("%.2f|%.2f\r\n%.2f\r\n", ((float)accX) * M5.IMU.aRes, ((float)accY) * M5.IMU.aRes, ((float)accZ) * M5.IMU.aRes);
      M5.Lcd.printf("Tempg:%.2fC\r\n", ((float)tempg) / 333.87 + 21.0);
      M5.Lcd.printf("CNT:%d\r\n", messageCount);

      //bRST = (digitalRead(M5_BUTTON_RST) == LOW);
      //bHOME = (digitalRead(M5_BUTTON_HOME) == LOW);
      M5.Lcd.printf("%d %d\r\n", bRST, bHOME);

      // Send teperature data
      char messagePayload[MESSAGE_MAX_LEN];
      snprintf(
          messagePayload,
          MESSAGE_MAX_LEN,
          "{"
          "\"deviceId\":\"%s\", \"messageId\":%d, "
          "\"vbat\":%.3f, \"icharge\":%f, \"idischg\":%f, \"temp\":%.1f, "
          "\"pbat\":%.3f, \"CoIn\":%d, \"CoOut\":%d, "
          "\"CoD\":%f, \"Vin\":%.3f, \"Iin\":%.3f, "
          "\"gyroX\":%.2f, \"gyroY\":%.2f, \"gyroZ\":%.2f, "
          "\"accX\":%.2f, \"accY\":%.2f, \"accZ\":%.2f, "
          "\"Tempg\":%.2f, "
          "\"bRTS\":%d, \"bHOME\":%d"
          "}",
          DEVICE_ID, messageCount++,
          vbat, charge, discharge, temp,
          bat_p, coIn, coOut,
          coD, vin, iin,
          ((float)gyroX) * M5.IMU.gRes, ((float)gyroY) * M5.IMU.gRes, ((float)gyroZ) * M5.IMU.gRes,
          ((float)accX) * M5.IMU.aRes, ((float)accY) * M5.IMU.aRes, ((float)accZ) * M5.IMU.aRes,
          ((float)tempg) / 333.87 + 21.0,
          bRST, bHOME);
      Serial.println(messagePayload);
      if ((msg = IoTHubMessage_CreateFromByteArray((const unsigned char *)messagePayload, strlen(messagePayload))) == NULL)
      {
        (void)printf("ERROR: iotHubMessageHandle is NULL!\r\n");
      }
      else
      {

        (void)IoTHubMessage_SetMessageId(msg, "MSG_ID");
        //(void)IoTHubMessage_SetCorrelationId(msg, "CORE_ID");
        (void)IoTHubMessage_SetContentTypeSystemProperty(msg, "application%2Fjson");
        (void)IoTHubMessage_SetContentEncodingSystemProperty(msg, "utf-8");

        MAP_HANDLE propMap = IoTHubMessage_Properties(msg);
        (void)sprintf_s(propText, sizeof(propText), (bRST==1 && bHOME==1)  ? "true" : "false");
        if (Map_AddOrUpdate(propMap, "valAlert", propText) != MAP_OK)
        {
          (void)printf("ERROR: Map_AddOrUpdate Failed!\r\n");
        }

        if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, msg, SendConfirmationCallback, &msg) != IOTHUB_CLIENT_OK)
        //if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, msg, NULL, NULL) != IOTHUB_CLIENT_OK)
        {
          (void)printf("ERROR: IoTHubClient_SendEventAsync..........FAILED!\r\n");
        }
      }
      IOTHUB_CLIENT_STATUS status;
      IoTHubClient_LL_DoWork(iotHubClientHandle);
      ThreadAPI_Sleep(100);
      //Wait till
      while ((IoTHubClient_LL_GetSendStatus(iotHubClientHandle, &status) == IOTHUB_CLIENT_OK) && (status == IOTHUB_CLIENT_SEND_STATUS_BUSY))
      {
        IoTHubClient_LL_DoWork(iotHubClientHandle);
        ThreadAPI_Sleep(100);
      }
      //Callback is responsible for destroying message
      //IoTHubMessage_Destroy(msg);
      ThreadAPI_Sleep(100);
      send_interval_ms = millis();
      bRST = bHOME = 0;
    }
    else
    {
    }
  }
  IoTHubClient_LL_DoWork(iotHubClientHandle);
  ThreadAPI_Sleep(100);
}