#include <Arduino.h>
#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PS4Controller.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Webpage.h"

void SendWebsite();
void SendXML();
void printWifiStatus();
void handleSetMode();
void handleMotorMove();
void emergencyStop();
void enableAllMotors();
void disableAllMotors();

void onPS4Connect()
{
  Serial.println("PS4 Controller Connected!");
  Serial.printf("Battery Level: %d%%\n", PS4.Battery());
}

void onPS4Disconnect()
{
  Serial.println("PS4 Controller Disconnected!");
}


char XML[2048];

char buf[32];

IPAddress Actual_IP;
IPAddress PageIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ip;

WebServer server(80);

// Motors Setup
unsigned long lastTimeStamp = 0;
int Mode = 1;

#define STEP_PIN1 17
#define DIR_PIN1 16

#define STEP_PIN2 27
#define DIR_PIN2 26

#define STEP_PIN3 5
#define DIR_PIN3 18

#define STEP_PIN4 19
#define DIR_PIN4 21

#define ENABLE_PIN1 25
#define ENABLE_PIN2 20
#define ENABLE_PIN3 22
#define ENABLE_PIN4 23


#define MAGNET_ENA 14 

#define MOTOR_TIMEOUT_MS 30000
unsigned long lastMotorActivity = 0;

volatile bool magnetState = false;

FastAccelStepperEngine Robot;
FastAccelStepper *motor1 = NULL;
FastAccelStepper *motor2 = NULL;
FastAccelStepper *motor3 = NULL;
FastAccelStepper *motor4 = NULL;

const int FULL_STEPS_PER_REV = 200;
const int MICROSTEP_FACTOR = 16;
const int STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP_FACTOR;

const float MOTOR1_RPM = 180.0f;
const float MOTOR1_SPEED_HZ = (MOTOR1_RPM * STEPS_PER_REV) / 60.0f;

const float MOTOR1_RAMP_TIME = 0.6;
const float MOTOR1_ACCEL = MOTOR1_SPEED_HZ / MOTOR1_RAMP_TIME;

const float MOTOR2_RPM = 250.0f;
const float MOTOR2_SPEED_HZ = (MOTOR2_RPM * STEPS_PER_REV) / 60.0f;

const float MOTOR2_RAMP_TIME = 0.5;
const float MOTOR2_ACCEL = MOTOR2_SPEED_HZ / MOTOR2_RAMP_TIME;

const float MOTOR3_RPM = 250.0f;
const float MOTOR3_SPEED_HZ = (MOTOR3_RPM * STEPS_PER_REV) / 60.0f;

const float MOTOR3_RAMP_TIME = 0.5;
const float MOTOR3_ACCEL = MOTOR3_SPEED_HZ / MOTOR3_RAMP_TIME;

const float MOTOR4_RPM = 200.0f;
const float MOTOR4_SPEED_HZ = (MOTOR4_RPM * STEPS_PER_REV) / 60.0f;

const float MOTOR4_RAMP_TIME = 0.5;
const float MOTOR4_ACCEL = MOTOR4_SPEED_HZ / MOTOR4_RAMP_TIME;

const uint32_t CONTROL_TICK_MS = 50;
const float STEPS_PER_TICK_M1 = MOTOR1_SPEED_HZ * (CONTROL_TICK_MS / 1000.0f);
const float STEPS_PER_TICK_M2 = MOTOR2_SPEED_HZ * (CONTROL_TICK_MS / 1000.0f);
const float STEPS_PER_TICK_M3 = MOTOR3_SPEED_HZ * (CONTROL_TICK_MS / 1000.0f);
const float STEPS_PER_TICK_M4 = MOTOR4_SPEED_HZ * (CONTROL_TICK_MS / 1000.0f);

float DistanceinY = 0;
float DistanceinZ = 0;
float DistanceinX = 0;
float RotationAngle = 0;

long Motor1_Steps = 0;
long Motor2_Steps = 0;
long Motor3_Steps = 0;
long Motor4_Steps = 0;

volatile float currentXPosition = 0.0f;
volatile float currentYPosition = 0.0f;
volatile float currentZPosition = 0.0f;
volatile float currentRotation = 0.0f;
volatile bool Move_Motor1 = false;
volatile bool Move_Motor2 = false;
volatile bool Move_Motor3 = false;
volatile bool Move_Motor4 = false;

int microstepMode = 16;

float mmPerStepX = 0.00179f;
float mmPerStepY = 0.0075f;
float mmPerStepZ = 0.0074f;
float mmPerStepRotation = 0.0067f;

volatile float targetX = 0;
volatile float targetY = 0;
volatile float targetZ = 0;
volatile float targetR = 0;
volatile int targetMode = 1;

int no_of_commands = 0;

String serialBuffer = "";

bool lastMotorState = false; // false = stopped, true = moving
unsigned long lastStatusSend = 0;
const unsigned long STATUS_INTERVAL = 100; // Send status every 100ms

inline long mm_to_steps(float mm, float mmPerStep)
{
  return (long)round((mm / mmPerStep));
}

inline long deg_to_steps(float deg)
{
  return (long)round((deg / mmPerStepRotation));
}

inline float steps_to_mm(long steps, float mmPerStep)
{
  return (float)steps / microstepMode * mmPerStep;
}

inline float steps_to_deg(long steps)
{
  return (float)steps / microstepMode * mmPerStepRotation;
}
void TaskSerial(void *pvParameters)
{
  (void)pvParameters;
  static char buf[128];
  static size_t idx = 0;

  for (;;)
  {
    while (Serial.available())
    {
      char c = Serial.read();

      // ignore CR
      if (c == '\r')
        continue;

      if (c == '\n')
      {
        buf[idx] = '\0';
        idx = 0;

        if (buf[0] == '\0')
          break;

        if (strncmp(buf, "MODE:", 5) == 0)
        {
          int m = atoi(buf + 5);
          Mode = m;
          Serial.printf("Mode for control:%d\n", m);
          break;
        }

        if (strncmp(buf, "MAG:", 4) == 0)
        {
          int magVal = atoi(buf + 4);
          if (magVal == 1)
          {
            digitalWrite(MAGNET_ENA, LOW);
            magnetState = true;
            Serial.println("MAGNET:ON");
          }
          else
          {
            digitalWrite(MAGNET_ENA, HIGH);
            magnetState = false;
            Serial.println("MAGNET:OFF");
          }
          break;
        }

        if (strncmp(buf, "POS:", 4) == 0)
        {
          char *s = buf + 4;
          int vals[5] = {0, 0, 0, 0, 0};
          int i = 0;
          char *tok = strtok(s, ",");
          while (tok != NULL && i < 5)
          {
            vals[i++] = atoi(tok);
            tok = strtok(NULL, ",");
          }
          if (i == 5)
          {
            targetX = (float)vals[0];
            targetY = (float)vals[2];
            targetZ = (float)vals[1];
            targetR = (float)vals[3];
            int targetMode = vals[4];
            Mode = targetMode;
            // DistanceinX = targetX - currentXPosition;
            // DistanceinY = targetY - currentYPosition;
            // DistanceinZ = targetZ - currentZPosition;
            // RotationAngle = targetR - currentRotation;
            Motor1_Steps = mm_to_steps((float)vals[0], mmPerStepX);
            Motor3_Steps = mm_to_steps((float)vals[1], mmPerStepZ);
            Motor2_Steps = mm_to_steps((float)vals[2], mmPerStepY);
            Motor4_Steps = deg_to_steps((float)vals[3]);
            Move_Motor1 = true;
            Move_Motor2 = true;
            Move_Motor3 = true;
            Move_Motor4 = true;
            // currentXPosition = targetX;
            // currentYPosition = targetY;
            // currentZPosition = targetZ;
            // currentRotation = targetR;

            Serial.printf("Commands:%.2f,%.2f,%.2f,%.2f,%d\n",
                          targetX, targetY, targetZ, targetR, targetMode);
            Serial.printf("Motor 1, Motor2, Motor3, Motor4, Steps: %d, %d, %d, %d\n", Motor1_Steps, Motor2_Steps, Motor3_Steps, Motor4_Steps);
          }

          else
          {
            Serial.println("ERR:BAD_POS_FORMAT");
          }
          break;
        }
        Serial.printf("ERR:UNKNOWN_CMD:%s\n", buf);
        break;
      }

      // Normal character -> append if space
      if (idx < sizeof(buf) - 1)
      {
        buf[idx++] = c;
      }
      else
      {
        // Buffer overflow — reset and warn
        idx = 0;
        Serial.println("ERR:LINE_TOO_LONG");
      }
    } // while Serial.available()

    vTaskDelay(pdMS_TO_TICKS(5)); // yield
  } // for
}

void TaskMotor1(void *pvParameters) // x-axis
{
  for (;;)
  {
    if (Mode == 1)
    {
      if (Move_Motor1)
      {
        enableAllMotors();
        lastMotorActivity = millis();
        motor1->moveTo(-Motor1_Steps);
        Motor1_Steps = 0;
        Move_Motor1 = false;
        while (motor1->isRunning())
          vTaskDelay(1);
        Serial.printf("Motor1 Distance:%d \n", motor1->getCurrentPosition());
      }
      // mm_to_steps(350.0f, mmPerStepX);
      // // Serial.println("[MOTOR 1] Moving -195000");
      // motor1->moveTo(-mm_to_steps(350.0f, mmPerStepX));
      // while (motor1->isRunning())
      //   vTaskDelay(1);
      // Serial.print(motor1->getCurrentPosition());
      // vTaskDelay(4000 / portTICK_PERIOD_MS);
      // motor1->moveTo(0);
      // while (motor1->isRunning())
      // vTaskDelay(1);
      // Serial.print(motor1->getCurrentPosition());
      // Serial.println("[MOTOR 1] Stopped");

      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    if (Mode == 2)
    {
      if (PS4.Right())
      {
        Serial.println("Right pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor1->moveTo(motor1->getCurrentPosition() - (long)round(STEPS_PER_TICK_M1));
        // Serial.println("Motor1");
        // Serial.print(motor1->getCurrentPosition());
      }
      else if (PS4.Left())
      {
        Serial.println("Left pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor1->moveTo(motor1->getCurrentPosition() + (long)round(STEPS_PER_TICK_M1));
      }
      else if (PS4.Square())
      {
        Serial.println("Square pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor2->moveTo(motor2->getCurrentPosition() + (long)round(STEPS_PER_TICK_M2));
        // Serial.println("Motor2");
        // Serial.print(motor2->getCurrentPosition());
      }
      else if (PS4.Circle())
      {
        Serial.println("Circle pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor2->moveTo(motor2->getCurrentPosition() - (long)round(STEPS_PER_TICK_M2));
        // Serial.println("Motor2");
        // Serial.print(motor2->getCurrentPosition());
      }
      else if (PS4.Up())
      {
        Serial.println("Up pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor3->moveTo(motor3->getCurrentPosition() + (long)round(STEPS_PER_TICK_M3));
        // Serial.println("Motor3");
        // Serial.print(motor3->getCurrentPosition());
      }
      else if (PS4.Down())
      {
        Serial.println("Down pressed");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor3->moveTo(motor3->getCurrentPosition() - (long)round(STEPS_PER_TICK_M3));
        // Serial.println("Motor3");
        // Serial.print(motor3->getCurrentPosition());
      }
      else if (PS4.Triangle())
      {
        Serial.println("Right stick pressed X");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor4->moveTo(motor4->getCurrentPosition() + (long)round(STEPS_PER_TICK_M4));
        // Serial.println("Motor4");
        // Serial.print(motor4->getCurrentPosition());
      }
      else if (PS4.Cross())
      {
        Serial.println("Right stick pressed Y");
        // enableAllMotors();
        // lastMotorActivity = millis();
        motor4->moveTo(motor4->getCurrentPosition() - (long)round(STEPS_PER_TICK_M4));
        // Serial.println("Motor4");
        // Serial.print(motor4->getCurrentPosition());
      }
      else if (PS4.Share())
      {
        Serial.println("Share pressed - Emergency Stop!");
        emergencyStop();
      }
      else if(PS4.R2())
      {
        if(!magnetState)
        {
          digitalWrite(MAGNET_ENA, HIGH);
          magnetState = true;
          Serial.println("MAGNET:ON");
        }
      }
      else if(PS4.L2())
      {
        if(magnetState)
        {
          digitalWrite(MAGNET_ENA, LOW);
          magnetState = false;
          Serial.println("MAGNET:OFF");
        }
      }
    }
  }
}

void TaskMotor2(void *pvParameters) // y-axis
{
  for (;;)
  {
    float move2 = 0;
    if (Mode == 1)
    {
      // DistanceinY = currentYPosition - targetY;
      // StepsY= DistanceinY / mmPerStepY;
      // Motor2_Steps = StepsY* microstepMode;
      // Serial.println("[MOTOR 2] Moving +200000");
      //  Serial.println("[MOTOR 2]");
      if (Move_Motor2)
      {
        enableAllMotors();
        lastMotorActivity = millis();
        motor2->moveTo(-Motor2_Steps);
        Motor2_Steps = 0;
        Move_Motor2 = false;
        while (motor2->isRunning())
        {
          vTaskDelay(1);
        }
        Serial.printf("Motor2 Distance:%d \n", motor2->getCurrentPosition());
      }
      // motor2->moveTo(-mm_to_steps(280.0, mmPerStepY));
      // while (motor2->isRunning())
      //   vTaskDelay(1);
      // vTaskDelay(7000 / portTICK_PERIOD_MS);

      // // Serial.println("[MOTOR 2] Back to 0");
      // motor2->moveTo(0);
      // Serial.println("[MOTOR 2] Stopped");

      vTaskDelay(60 / portTICK_PERIOD_MS);
    }
    else if (Mode == 2)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

void TaskMotor3(void *pvParameters) // z-axis
{
  for (;;)
  {
    if (Mode == 1)
    {
      // DistanceinZ = currentZPosition - targetZ;
      // StepsZ= DistanceinZ / mmPerStepZ;
      // Motor3_Steps = StepsZ* microstepMode;
      //  Serial.println("[MOTOR 3]");
      if (Move_Motor3)
      {
        enableAllMotors();
        lastMotorActivity = millis();
        motor3->moveTo(Motor3_Steps);
        Motor3_Steps = 0;
        Move_Motor3 = false;
        while (motor3->isRunning())
          vTaskDelay(1);
        Serial.printf("Motor3 Distance:%d \n", motor3->getCurrentPosition());
      }
      // motor3->moveTo(mm_to_steps(260.0f, mmPerStepZ));
      // while (motor3->isRunning())
      //   vTaskDelay(1);

      // vTaskDelay(5000 / portTICK_PERIOD_MS);
      // motor3->moveTo(25000);

      // Serial.println("[MOTOR 3] Stopped");

      vTaskDelay(70 / portTICK_PERIOD_MS);
    }
    else if (Mode == 2)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

void TaskMotor4(void *pvParameters) // rotation
{
  for (;;)
  {
    if (Mode == 1)
    {
      if (Move_Motor4)
      {
        enableAllMotors();
        lastMotorActivity = millis();
        motor4->moveTo(Motor4_Steps);
        Motor4_Steps = 0;
        Move_Motor4 = false;
        while (motor4->isRunning())
        {
          vTaskDelay(1);
        }
        Serial.printf("Motor4 Distance:%d \n", motor4->getCurrentPosition());
      }
      // motor4->moveTo(mm_to_steps(180.0f, mmPerStepRotation));
      // while (motor4->isRunning())
      //   vTaskDelay(1);
      // vTaskDelay(6000 / portTICK_PERIOD_MS);
      // motor4->moveTo(0);
      // Serial.println("[MOTOR 4] Stopped");
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    else if (Mode == 2)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  PS4.begin();

  Robot.init();

  Serial.println("FastAccelStepper Dual Motor Example");
  motor1 = Robot.stepperConnectToPin(STEP_PIN1);
  motor2 = Robot.stepperConnectToPin(STEP_PIN2);
  motor3 = Robot.stepperConnectToPin(STEP_PIN3);
  motor4 = Robot.stepperConnectToPin(STEP_PIN4);

  if (!motor1 || !motor2 || !motor3 || !motor4)
  {
    Serial.println("ERROR: Stepper connection failed!");
    while (1)
      ;
  }

  // Motor Settings - Manual enable control for safety
  motor1->setDirectionPin(DIR_PIN1);
  motor1->setAutoEnable(false); // Manual control for safety
  motor1->setAcceleration(MOTOR1_ACCEL);
  motor1->setSpeedInHz((uint32_t)MOTOR1_SPEED_HZ);
  motor1->setEnablePin(ENABLE_PIN1, false);
  motor1->disableOutputs(); // Start disabled

  motor2->setDirectionPin(DIR_PIN2);
  motor2->setAutoEnable(false); // Manual control for safety
  motor2->setAcceleration(MOTOR2_ACCEL);
  motor2->setSpeedInHz((uint32_t)MOTOR2_SPEED_HZ);
  motor2->setEnablePin(ENABLE_PIN2, false);
  motor2->disableOutputs(); // Start disabled

  motor3->setDirectionPin(DIR_PIN3);
  motor3->setAutoEnable(false); // Manual control for safety
  motor3->setAcceleration(MOTOR3_ACCEL);
  motor3->setSpeedInHz((uint32_t)MOTOR3_SPEED_HZ);
  motor3->setEnablePin(ENABLE_PIN3, false);
  motor3->disableOutputs(); // Start disabled

  motor4->setDirectionPin(DIR_PIN4);
  motor4->setAutoEnable(false); // Manual control for safety
  motor4->setAcceleration(MOTOR4_ACCEL);
  motor4->setSpeedInHz((uint32_t)MOTOR4_SPEED_HZ);
  motor4->setEnablePin(ENABLE_PIN4, false);
  motor4->disableOutputs(); // Start disabled

  // L298N Magnet Setup
  pinMode(MAGNET_ENA, OUTPUT);
  // pinMode(MAGNET_IN1, OUTPUT);
  // pinMode(MAGNET_IN2, OUTPUT);
  digitalWrite(MAGNET_ENA, LOW);
  // digitalWrite(MAGNET_IN1, LOW);
  // digitalWrite(MAGNET_IN2, LOW);
  Serial.println("Magnet control initialized (OFF)");

  lastMotorActivity = millis();

  // Serial.println("Creating Access Point...");
  // WiFi.softAP(AP_SSID, AP_PASS);
  // WiFi.softAPConfig(PageIP, gateway, subnet);
  // Actual_IP = WiFi.softAPIP();
  // Serial.println("Access Point Created.");
  // Serial.print("Network Name (SSID): ");
  // Serial.println(AP_SSID);
  // Serial.print("Password: ");
  // Serial.println(AP_PASS);
  // Serial.print("Access web interface at: http://");
  // Serial.println(Actual_IP);

  // printWifiStatus();

  // server.on("/", SendWebsite);
  // server.on("/xml", SendXML);
  // server.on("/status", SendXML);
  // server.on("/MODE", handleSetMode);
  // server.on("/MOVE", handleMotorMove);
  // server.on("/STOP", []()
  //           {
  //   emergencyStop();
  //   server.send(200, "text/plain", "Emergency stop activated."); }); // Emergency stop

  // server.begin();
  // Serial.println("HTTP Web Server started.");

  xTaskCreatePinnedToCore(
      TaskMotor1,
      "Motor1_Task",
      4096,
      NULL,
      1,
      NULL,
      1);

  xTaskCreatePinnedToCore(
      TaskMotor2,
      "Motor2_Task",
      4096,
      NULL,
      1,
      NULL,
      0 // Run on core 0
  );

  xTaskCreatePinnedToCore(
      TaskMotor3,
      "Motor3_Task",
      4096,
      NULL,
      1,
      NULL,
      1 // Run on core 1
  );
  xTaskCreatePinnedToCore(
      TaskMotor4,
      "Motor4_Task",
      4096,
      NULL,
      1,
      NULL,
      0 // Run on core 0
  );
  xTaskCreatePinnedToCore(
      TaskSerial,
      "SerialTask",
      4096,
      NULL,
      2,
      NULL,
      0);
}

void loop()
{
  server.handleClient();

  bool anyMotorMoving = motor1->isRunning() || motor2->isRunning() ||
                        motor3->isRunning() || motor4->isRunning();

  if (anyMotorMoving != lastMotorState)
  {
    if (anyMotorMoving)
    {
      Serial.println("MOVING");
    }
    else
    {
      Serial.println("REACHED");
    }
    lastMotorState = anyMotorMoving;
  }

  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void emergencyStop()
{
  motor1->forceStopAndNewPosition(motor1->getCurrentPosition());
  motor2->forceStopAndNewPosition(motor2->getCurrentPosition());
  motor3->forceStopAndNewPosition(motor3->getCurrentPosition());
  motor4->forceStopAndNewPosition(motor4->getCurrentPosition());
  motor1->disableOutputs();
  motor2->disableOutputs();
  motor3->disableOutputs();
  motor4->disableOutputs();
  Serial.println("EMERGENCY STOP");
}

void enableAllMotors()
{
  motor1->enableOutputs();
  motor2->enableOutputs();
  motor3->enableOutputs();
  motor4->enableOutputs();
  lastMotorActivity = millis();
}

void disableAllMotors()
{
  motor1->disableOutputs();
  motor2->disableOutputs();
  motor3->disableOutputs();
  motor4->disableOutputs();
}

void SendXML()
{
  char motor_status_buf[256];

  strcpy(XML, "<?xml version = '1.0'?>\n<Data>\n");
  sprintf(buf, "<MODE>%d</MODE>\n", Mode);
  strcat(XML, buf);

  sprintf(buf, "<POS1>%ld</POS1>\n", motor1 ? motor1->getCurrentPosition() : 0L);
  strcat(XML, buf);
  sprintf(buf, "<POS2>%ld</POS2>\n", motor2 ? motor2->getCurrentPosition() : 0L);
  strcat(XML, buf);
  sprintf(buf, "<POS3>%ld</POS3>\n", motor3 ? motor3->getCurrentPosition() : 0L);
  strcat(XML, buf);
  sprintf(buf, "<POS4>%ld</POS4>\n", motor4 ? motor4->getCurrentPosition() : 0L);
  strcat(XML, buf);

  strcat(XML, "</Data>\n");

  server.send(200, "text/xml", XML);
}

void SendWebsite()
{
  Serial.println("Sending main web page...");
  server.send(200, "text/html", PAGE_MAIN);
}

void handleSetMode()
{
  if (server.hasArg("value"))
  {
    int newMode = server.arg("value").toInt();
    if (newMode >= 1 && newMode <= 2)
    {
      Mode = newMode;
      Serial.printf("Server: Control Mode set to %d\n", Mode);
      server.send(200, "text/plain", "Mode set successfully.");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid mode value.");
}

// Handler for directly commanding a motor
// URL format: /MOVE?m=1&s=5000 (Motor 1, 5000 steps)
void handleMotorMove()
{
  if (server.hasArg("m") && server.hasArg("s"))
  {
    int motor_id = server.arg("m").toInt();
    long steps = server.arg("s").toInt();

    if (Mode != 1)
    {
      server.send(400, "text/plain", "Cannot move motor, Mode is not 1.");
      return;
    }

    enableAllMotors();
    lastMotorActivity = millis();
    switch (motor_id)
    {
    case 1:
      Motor1_Steps = steps;
      break;
    case 2:
      Motor2_Steps = steps;
      break;
    case 3:
      Motor3_Steps = steps;
      break;
    case 4:
      Motor4_Steps = steps;
      break;
    default:
      server.send(400, "text/plain", "Invalid motor ID.");
      return;
    }

    Serial.printf("Server: Motor %d commanded to move %ld steps.\n", motor_id, steps);
    server.send(200, "text/plain", "Motor command accepted.");
    return;
  }
  server.send(400, "text/plain", "Missing motor (m) or steps (s) argument.");
}

void printWifiStatus()
{
  Serial.println("--- WiFi Status ---");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connected as STA. IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.print("AP Mode IP: ");
    Serial.println(WiFi.softAPIP());
  }
  Serial.print("Control Mode: ");
  Serial.println(Mode);
}
