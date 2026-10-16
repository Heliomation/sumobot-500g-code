#include "usbh_helper.h"
#include "CytronMotorDriver.h"
#include "hardware/watchdog.h"

//code for 500g with 7 sensors.
//warning Don't max the speed, your gears will die

// ==================== PIN DEFINITIONS ====================
const int BTN_MANUAL_PIN = 20;
const int BTN_AUTO_PIN   = 21;

// Sharp IR Distance Sensors (Digital Pins)
const int SENSOR_LEFT_PIN       = 26;
const int SENSOR_FRONT_PIN      = 27;
const int SENSOR_RIGHT_PIN      = 28;
const int SENSOR_SIDE_LEFT_PIN  = 19; 
const int SENSOR_SIDE_RIGHT_PIN = 29; 

// Bottom Edge Sensors (Digital Pins)
const int SENSOR_BOTTOM_LEFT_PIN  = 2;
const int SENSOR_BOTTOM_RIGHT_PIN = 3;

// Active Logic States
const int SENSOR_DETECT_STATE = HIGH;  // Front/Angled sensors
const int SIDE_DETECT_STATE   = HIGH;  // Active HIGH logic
const int BOTTOM_DETECT_STATE = HIGH;  // Bottom sensors

// Speeds for Autonomous Mode
const int SPEED_SLOW  = 70;   
const int SPEED_FULL  = 240;  
const int SPEED_TURN  = 180;  

// Timings for Recovery & Side Maneuvers (ms)
const unsigned long REVERSE_TIME       = 190;
const unsigned long RECOVERY_TURN_TIME = 220;
const unsigned long SIDE_TURN_TIME     = 100;

// ==================== SYSTEM MODES ====================
enum SystemMode {
  MODE_IDLE,
  MODE_MANUAL,
  MODE_AUTO
};

SystemMode currentMode = MODE_IDLE;

// ==================== MOTOR DEFINITIONS (4WD) ====================
CytronMD motorL(PWM_PWM, 12, 13);   // Rear Left motor
CytronMD motorFL(PWM_PWM, 9, 8);    // Front Left motor
CytronMD motorFR(PWM_PWM, 11, 10);  // Front Right motor
CytronMD motorR(PWM_PWM, 14, 15);   // Rear Right motor

volatile uint8_t yL = 127;
volatile uint8_t xR = 127;

volatile bool controller_connected = false;
volatile unsigned long lastReportTime = 0;

int outL = 0;
int outR = 0;

const unsigned long CONTROL_TIMEOUT   = 200;   
const unsigned long USB_STALL_TIMEOUT = 1000;  

const int DEADZONE = 20;
const int RAMP_STEP = 12; 
const float TURN_SCALE = 0.75;

// Timed Maneuver Flags
bool isEscaping = false;
unsigned long escapeStartTime = 0;

bool isSideTurning = false;
unsigned long sideTurnStartTime = 0;
int sideTurnLeftDir = 0;
int sideTurnRightDir = 0;

int mapCentered(uint8_t v)
{
  int centered = (int)v - 127;
  return (centered * 255) / 127;
}

int applyDeadzone(int v)
{
  return (abs(v) < DEADZONE) ? 0 : v;
}

int ramp(int current, int target)
{
  if (current < target) return min(current + RAMP_STEP, target);
  if (current > target) return max(current - RAMP_STEP, target);
  return target;
}

void setTankDrive(int leftSpeed, int rightSpeed)
{
  outL = ramp(outL, leftSpeed);
  outR = ramp(outR, rightSpeed);

  motorL.setSpeed(outL);
  motorFL.setSpeed(outL);

  motorR.setSpeed(outR);
  motorFR.setSpeed(outR);
}

void stopMotors()
{
  motorL.setSpeed(0);
  motorFL.setSpeed(0);
  motorFR.setSpeed(0);
  motorR.setSpeed(0);

  outL = 0;
  outR = 0;
}

void hardReset()
{
  watchdog_enable(1, 1); 
  while (1); 
}

void checkButtons()
{
  if (digitalRead(BTN_MANUAL_PIN) == LOW)
  {
    if (currentMode != MODE_MANUAL)
    {
      stopMotors();
      isEscaping = false;
      isSideTurning = false;
      currentMode = MODE_MANUAL;
      Serial.println("Mode Switched: MANUAL (USB Controller)");
      delay(200); 
    }
  }
  else if (digitalRead(BTN_AUTO_PIN) == LOW)
  {
    if (currentMode != MODE_AUTO)
    {
      stopMotors();
      isEscaping = false;
      isSideTurning = false;
      currentMode = MODE_AUTO;
      Serial.println("Mode Switched: AUTONOMOUS");
      delay(200); 
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(BTN_MANUAL_PIN, INPUT_PULLUP);
  pinMode(BTN_AUTO_PIN, INPUT_PULLUP);

  // Configure Digital Pins for Distance Sensors
  pinMode(SENSOR_LEFT_PIN, INPUT);
  pinMode(SENSOR_FRONT_PIN, INPUT);
  pinMode(SENSOR_RIGHT_PIN, INPUT);
  
  pinMode(SENSOR_SIDE_LEFT_PIN, INPUT);
  pinMode(SENSOR_SIDE_RIGHT_PIN, INPUT);

  // Configure Digital Pins for Bottom Surface Sensors
  pinMode(SENSOR_BOTTOM_LEFT_PIN, INPUT);
  pinMode(SENSOR_BOTTOM_RIGHT_PIN, INPUT);

  stopMotors();

  rp2040_configure_pio_usb();
  USBHost.begin(1);

  Serial.println("System Ready. Waiting for mode button press...");
}

void runAutonomous()
{
  unsigned long now = millis();

  // 1. READ BOTTOM SENSORS (Highest Priority)
  bool detectBottomLeft  = (digitalRead(SENSOR_BOTTOM_LEFT_PIN) == BOTTOM_DETECT_STATE);
  bool detectBottomRight = (digitalRead(SENSOR_BOTTOM_RIGHT_PIN) == BOTTOM_DETECT_STATE);

  if ((detectBottomLeft || detectBottomRight) && !isEscaping)
  {
    isEscaping = true;
    isSideTurning = false; 
    escapeStartTime = now;
  }

  if (isEscaping)
  {
    unsigned long elapsedTime = now - escapeStartTime;

    if (elapsedTime < REVERSE_TIME)
    {
      setTankDrive(SPEED_FULL, SPEED_FULL); 
    }
    else if (elapsedTime < (REVERSE_TIME + RECOVERY_TURN_TIME))
    {
      setTankDrive(SPEED_TURN, -SPEED_TURN); 
    }
    else
    {
      isEscaping = false;
    }
    return;
  }

  // 2. READ 90-DEGREE SIDE DISTANCE SENSORS
  bool detectSideLeft  = (digitalRead(SENSOR_SIDE_LEFT_PIN) == SIDE_DETECT_STATE);
  bool detectSideRight = (digitalRead(SENSOR_SIDE_RIGHT_PIN) == SIDE_DETECT_STATE);

  if (!isSideTurning)
  {
    if (detectSideLeft)
    {
      isSideTurning = true;
      sideTurnStartTime = now;
      sideTurnLeftDir  = SPEED_TURN;   // Inverted: Turn Left/Right corrected
      sideTurnRightDir = -SPEED_TURN;
    }
    else if (detectSideRight)
    {
      isSideTurning = true;
      sideTurnStartTime = now;
      sideTurnLeftDir  = -SPEED_TURN;  // Inverted: Turn Left/Right corrected
      sideTurnRightDir = SPEED_TURN;
    }
  }

  if (isSideTurning)
  {
    if (now - sideTurnStartTime < SIDE_TURN_TIME)
    {
      setTankDrive(sideTurnLeftDir, sideTurnRightDir);
      return;
    }
    else
    {
      isSideTurning = false;
    }
  }

  // 3. READ FRONT & ANGLED SENSORS
  bool detectLeft  = (digitalRead(SENSOR_LEFT_PIN) == SENSOR_DETECT_STATE);
  bool detectFront = (digitalRead(SENSOR_FRONT_PIN) == SENSOR_DETECT_STATE);
  bool detectRight = (digitalRead(SENSOR_RIGHT_PIN) == SENSOR_DETECT_STATE);

  if (detectFront)
  {
    setTankDrive(-SPEED_FULL, -SPEED_FULL); 
  }
  else if (detectLeft)
  {
    setTankDrive(SPEED_TURN, -SPEED_TURN);
  }
  else if (detectRight)
  {
    setTankDrive(-SPEED_TURN, SPEED_TURN);
  }
  else
  {
    setTankDrive(-SPEED_SLOW, -SPEED_SLOW);
  }
}

void loop()
{
  USBHost.task();
  checkButtons();

  unsigned long now = millis();

  switch (currentMode)
  {
    case MODE_IDLE:
      stopMotors();
      break;

    case MODE_MANUAL:
      if (!controller_connected)
      {
        stopMotors();
        return;
      }

      if (now - lastReportTime > CONTROL_TIMEOUT)
      {
        stopMotors();
      }

      if (now - lastReportTime > USB_STALL_TIMEOUT)
      {
        hardReset();
      }

      {
        int forward = mapCentered(yL);              
        int turn    = -mapCentered(xR) * TURN_SCALE; 

        int targetL = constrain(applyDeadzone(forward + turn), -255, 255);
        int targetR = constrain(applyDeadzone(forward - turn), -255, 255);

        setTankDrive(targetL, targetR);
      }
      break;

    case MODE_AUTO:
      runAutonomous();
      break;
  }
}

void updateControl(uint8_t leftY, uint8_t rightX)
{
  yL = leftY;
  xR = rightX;
  lastReportTime = millis();
}

extern "C"
{

void tuh_hid_mount_cb(uint8_t dev_addr,
                      uint8_t instance,
                      uint8_t const *desc_report,
                      uint16_t desc_len)
{
  controller_connected = true;
  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  controller_connected = false;
  stopMotors();
}

void tuh_hid_report_received_cb(uint8_t dev_addr,
                                uint8_t instance,
                                uint8_t const *report,
                                uint16_t len)
  {
    if (len >= 7)
    {
      uint8_t leftY  = report[4];
      uint8_t rightX = report[5];

      updateControl(leftY, rightX);
    }

    tuh_hid_receive_report(dev_addr, instance);
  }
}

