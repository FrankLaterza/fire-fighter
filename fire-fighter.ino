// code generated after following this guide:  https://racheldebarros.com/esp32-projects/connect-your-game-controller-to-an-esp32/
#include <Arduino.h>
#include <Bluepad32.h>

#define LED 2
#define LEFT_FRONT_MOTOR_PIN_1 26 
#define LEFT_FRONT_MOTOR_PIN_2 27 
#define LEFT_FRONT_MOTOR_PIN_PWM 14


#define LEFT_BACK_MOTOR_PIN_1 5 
#define LEFT_BACK_MOTOR_PIN_2 17 
#define LEFT_BACK_MOTOR_PIN_PWM 16

// TODO: map these pins
#define RIGHT_FRONT_MOTOR_PIN_1 33 
#define RIGHT_FRONT_MOTOR_PIN_2 25 
#define RIGHT_FRONT_MOTOR_PIN_PWM 32 

#define RIGHT_BACK_MOTOR_PIN_1 19 
#define RIGHT_BACK_MOTOR_PIN_2 18 
#define RIGHT_BACK_MOTOR_PIN_PWM 21 

#define ULTRA_SONIC_ECHO_PIN 2
#define ULTRA_SONIC_TRIG_PIN 4

#define LEFT_FIRE_TRIG_PIN 35 
#define RIGHT_FIRE_TRIG_PIN 34 

#define SQUIRT_PIN 23

#define ANALOG_THRESHOLD 4095 // Threshold of analog IR output
#define FLAME_MAGNITUDE_THRESHOLD 2000 // Threshold of flame magnitude (calibrate as needed)
#define IR_DIFF_THRESHOLD 200 // Threshold of input diff

#define INPUT_COUNT 30 // Length of cache

#define MOTOR_UPPER_BOUND 256 // Upper bound of exerted motor power
#define MOTOR_LOWER_BOUND -255 // Lower bound of exerted motor power

#define ONE_SECOND 1000 // One second in milliseconds
#define TICK_TIME 500 // Delay time in milliseconds (ms)

//dThese are used for the ultrasonic logic
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
// DEL //
// typedef enum {
//     MANUAL_MODE,
//     AVOIDANCE_MODE,
//     HOT_PURSUIT_MODE, 
// } modes_e;
// DEL ^

typedef enum {
    MANUAL_MODE,
    AUTOMATIC_MODE, 
} modes_e;

modes_e current_mode = MANUAL_MODE;

typedef enum {
    FORWARD,
    BACKWARD
} direction_e;

// Define the motor type
typedef enum {
    FRONT_LEFT,
    BACK_LEFT,
    FRONT_RIGHT,
    BACK_RIGHT
} motor_e;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Left IR sensor
int leftInputSum; // Records sum of [INPUT_COUNT] number of inputs
int leftInputMean; // Stores mean calculation: leftInputSum / INPUT_COUNT

// Right IR sensor
int rightInputSum; // Records sum of [INPUT_COUNT] number of inputs
int rightInputMean; // Stores mean calculation: leftInputSum / INPUT_COUNT

int inputDiff; // leftInputMean - rightInputMean
int robotMode; // Controls mode of robot

int inc;

// Resets symbols to zero
void resetSymbols() {
  leftInputSum = 0;
  leftInputMean = 0;
  rightInputSum = 0;
  rightInputMean = 0;
  inputDiff = 0;

  // Seed randomizer
  randomSeed(inc);
}

// Records an instance of sensor data
void calcSensorData() {
  // Set cache
  for(int i = 0; i < INPUT_COUNT; i++) {
    leftInputSum += analogRead(LEFT_FIRE_TRIG_PIN);
    rightInputSum += analogRead(RIGHT_FIRE_TRIG_PIN);
  }

  leftInputMean = leftInputSum / INPUT_COUNT;
  rightInputMean = rightInputSum / INPUT_COUNT;
  inputDiff = leftInputMean - rightInputMean;

  // ----- Collect ultrasonic data ------ //
  digitalWrite(ULTRA_SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(2);
  //set trigger pin to high for 10 ms
  digitalWrite(ULTRA_SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_SONIC_TRIG_PIN, LOW);

  duration = pulseIn(ULTRA_SONIC_ECHO_PIN, HIGH);
  //calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
}

void moveYAxis(int rate) {
  move_motor(FRONT_LEFT, rate);
  move_motor(FRONT_RIGHT, rate);
  move_motor(BACK_LEFT, rate);
  move_motor(BACK_RIGHT, rate);
}

void rotate(int rate) {
  move_motor(FRONT_LEFT, rate);
  move_motor(FRONT_RIGHT, -rate);
  move_motor(BACK_LEFT, rate);
  move_motor(BACK_RIGHT, -rate);
}

// positive is left
void strafe(int rate) {
  move_motor(FRONT_LEFT, -rate);
  move_motor(FRONT_RIGHT, -rate);
  move_motor(BACK_LEFT, rate);
  move_motor(BACK_RIGHT, rate);
}

void stop_all(){
  move_motor(FRONT_LEFT, 0);
  move_motor(FRONT_RIGHT, 0);
  move_motor(BACK_LEFT, 0);
  move_motor(BACK_RIGHT, 0);
}

// Preforms a blind search of the area around the robot by
// rotating in place, then moving forward.
void searchMode() {
  Serial.println("Searching for flames...");

  // int rand = random(MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND); // DEL

  // rotate(rand); // DEL

  rotate(random(MOTOR_LOWER_BOUND, MOTOR_UPPER_BOUND));

  // Serial.println(rand); // DEL

  moveYAxis(50);
  
}

// Performs a targeted pursuit of a detected flame by aiming and moving
// straight towards the precise area of detection. This
// is determined when the sensor input breaks the threshold.
void pursuitMode() {
  Serial.println("In pursuit of flames...");
  
  // If right sensor detects more IR, then turn right
  if(inputDiff > IR_DIFF_THRESHOLD) {
    rotate(50);
  }
  // If left sensor detects more IR, then turn left
  else if (inputDiff < -IR_DIFF_THRESHOLD) {
    rotate(-50);
  }
  moveYAxis(100);

}

// Goal: Get around obstacle and continue search
void avoidanceMode() {
  Serial.println("Avoiding detected obstacle...");
  
  // ---- TODO ----- //
}

// Goal: Get around obstacle and set robot in general direction of
// detected flame
void workaroundMode() {
  Serial.println("Working around detected obstacle...");
  
  // ---- TODO ----- //
}




// Performs a sprinkler-style spray of the flame until it is no longer
// detected by the flame sensor. Extra water is ejected to ensure
// the flame is gone. Once in this mode, the robot stays in this mode.
void sprayNPrayMode() {
  Serial.println("Spraying and praying...");

  // If right sensor detects more IR, then turn right
  if(inputDiff > IR_DIFF_THRESHOLD) {
    rotate(25);
  }
  // If left sensor detects more IR, then turn left
  else if (inputDiff < -IR_DIFF_THRESHOLD) {
    rotate(-25);
  }

  // ----- Code for spraying the water gun ------- //

}

// Logic block for deciding which mode the robot is in
void modeLogic() {
  // Determines robot robotMode
  if(robotMode != 4) { // Has not arrived at flame
    if((1)/*No obstructions detected*/) {
      if(abs(inputDiff) <= 0)
      {robotMode = 0;} // No flame, no obstruction
      else if(abs(inputDiff) > 0) {
        robotMode = 1; // Yes flame, no obstruction

        if(leftInputMean < FLAME_MAGNITUDE_THRESHOLD
        && rightInputMean < FLAME_MAGNITUDE_THRESHOLD)
        {robotMode = 4;} // Arrived at flame
      }
      else
      {robotMode = -1;}
    }
    else if((0)/*Obstructions detected*/) {
      if(abs(inputDiff) == 0)
      {robotMode = 2;} // No flame, yes obstruction
      else if(abs(inputDiff) > 0)
      {robotMode = 3;} // Yes flame, yes obstruction
      else
      {robotMode = -1;}
    }
    else
    {robotMode = -1;}

  }
}

// Switch for robotMode (determines next action)
void modeSwitch() {
  switch(robotMode)
  {
    case 0:
      // Goes into searchMode()
      searchMode();
      break;
    case 1:
      // Goes into pursuitMode()
      pursuitMode();
      break;
    case 2:
      // Goes into avoidanceMode()
      avoidanceMode();
      break;
    case 3:
      // Goes into workaroundMode()
      workaroundMode();
      break;
    case 4:
      // Goes into sprayNPrayMode()
      sprayNPrayMode();
      break;
    default: // Error
      Serial.println("Error: No granular mode specified.");
      break;
  }
}

void spray_and_pray() {
    // turn on the sprayer
    digitalWrite(SQUIRT_PIN, HIGH);
    delay(500);
    strafe(255);
    delay(500);
    strafe(-255);
    delay(1000);
    strafe(255);
    delay(1000);
    strafe(-255);
    delay(500);
    stop_all();
    digitalWrite(SQUIRT_PIN, LOW);
}

int search_res = 100; // ms of pausing for each search
// note: the search could potentially find the flame
void delay_and_search(int ms){
    for(int i = 0; i < ms/search_res; i++) {
        resetSymbols();
        calcSensorData();
        // it is very likely that there is a flame in front
        if (leftInputMean <= 2000 || rightInputMean <= 2000) {
            stop_all();
            delay(500);
            spray_and_pray();
            break;
        }
        delay(search_res);
    }
}

void delay_and_avoid(int ms){
    resetSymbols();
    calcSensorData(); // collected at the end to save delay for calculation delay
    for(int i = 0; i < ms/search_res; i++) {
        resetSymbols();
        calcSensorData();
        // it is very likely that there is something infront
        if (distanceCm <= 20) {
            moveYAxis(-255);
            delay(500);
            rotate(255);
            delay(2000);
            stop_all();
            break;
        }
        delay(search_res);
    }
}


int rotate_max = 5; // jerk rotate 10 times
int rotate_count = 0;

void basic_flame_search(){

    // basic serach loop
    rotate(255);
    delay_and_search(200);
    stop_all();
    delay(300);
    rotate_count++;
    if (rotate_count >= rotate_max) {
        // we are done going in circles so just move forward as long as there are not obsticals
        moveYAxis(255); // move forward
        delay_and_avoid(2000);
        stop_all();
        delay(300);
        // TODO: check for obsticals
        rotate_count = 0;
    }
} 

// This callback gets called any timd at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID34, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }

    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(), // Controller Index
        ctl->dpad(), // D-pad
        ctl->buttons(), // bitmask of pressed buttons
        ctl->axisX(), // (-511 - 512) left X Axis
        ctl->axisY(), // (-511 - 512) left Y axis
        ctl->axisRX(), // (-511 - 512) right X axis
        ctl->axisRY(), // (-511 - 512) right Y axis
        ctl->brake(), // (0 - 1023): brake button
        ctl->throttle(), // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(), // bitmask of pressed "misc" buttons
        ctl->gyroX(), // Gyro X
        ctl->gyroY(), // Gyro Y
        ctl->gyroZ(), // Gyro Z
        ctl->accelX(), // Accelerometer X
        ctl->accelY(), // Accelerometer Y
        ctl->accelZ() // Accelerometer Z
    );
}

void move_motor(motor_e motor, int16_t val) {
    // Determine direction based on the sign of 'val'
    direction_e dir = (val >= 0) ? FORWARD : BACKWARD;
    // Ensure 'val' is positive for PWM (0-255)
    uint8_t pwm_val = abs(val);

    switch (motor) {
    case FRONT_LEFT:
        analogWrite(LEFT_FRONT_MOTOR_PIN_PWM, pwm_val);
        digitalWrite(LEFT_FRONT_MOTOR_PIN_1, dir);
        digitalWrite(LEFT_FRONT_MOTOR_PIN_2, !dir);
        break;
    case BACK_LEFT:
        analogWrite(LEFT_BACK_MOTOR_PIN_PWM, pwm_val);
        digitalWrite(LEFT_BACK_MOTOR_PIN_1, dir);
        digitalWrite(LEFT_BACK_MOTOR_PIN_2, !dir);
        break;
    case FRONT_RIGHT:
        analogWrite(RIGHT_FRONT_MOTOR_PIN_PWM, pwm_val);
        digitalWrite(RIGHT_FRONT_MOTOR_PIN_1, !dir);  // reverse motor wiring
        digitalWrite(RIGHT_FRONT_MOTOR_PIN_2, dir);   // reverse motor wiring
        break;
    case BACK_RIGHT:
        analogWrite(RIGHT_BACK_MOTOR_PIN_PWM, pwm_val);
        digitalWrite(RIGHT_BACK_MOTOR_PIN_1, !dir);   // reverse motor wiring
        digitalWrite(RIGHT_BACK_MOTOR_PIN_2, dir);    // reverse motor wiring
        break;
    }
}
// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
    //== PS4 X button = 0x0001 ==//
    if (ctl->buttons() == 0x0001) {
        digitalWrite(LED, HIGH); // Turn the LED on
    }
    if (ctl->buttons() != 0x0001) {
        digitalWrite(LED, LOW); // Turn the LED on
    }

    //== PS4 Square button = 0x0004 ==//
    if (ctl->buttons() == 0x0004) {
        // current_mode = HOT_PURSUIT_MODE;
        current_mode = AUTOMATIC_MODE;
    }
    if (ctl->buttons() != 0x0004) {
    }

    //== PS4 Triangle button = 0x0008 ==//
    if (ctl->buttons() == 0x0008) {
        current_mode = MANUAL_MODE;
    }
    if (ctl->buttons() != 0x0008) {
    }

    //== PS4 Circle button = 0x0002 ==//
    if (ctl->buttons() == 0x0002) {
        // current_mode = AVOIDANCE_MODE;
        current_mode = AUTOMATIC_MODE;
    }
    if (ctl->buttons() != 0x0002) {
    }

    //== PS4 Dpad UP button = 0x01 ==//
    if (ctl->buttons() == 0x01) {
        // code for when dpad up button is pushed
    }
    if (ctl->buttons() != 0x01) {
        // code for when dpad up button is released
    }

    //==PS4 Dpad DOWN button = 0x02==//
    if (ctl->buttons() == 0x02) {
        // code for when dpad down button is pushed
    }
    if (ctl->buttons() != 0x02) {
        // code for when dpad down button is released
    }

    //== PS4 Dpad LEFT button = 0x08 ==//
    if (ctl->buttons() == 0x08) {
        // code for when dpad left button is pushed
    }
    if (ctl->buttons() != 0x08) {
        // code for when dpad left button is released
    }

    //== PS4 Dpad RIGHT button = 0x04 ==//
    if (ctl->buttons() == 0x04) {
        // code for when dpad right button is pushed
    }
    if (ctl->buttons() != 0x04) {
        // code for when dpad right button is released
    }

    //== PS4 R1 trigger button = 0x0020 ==//
    if (ctl->buttons() == 0x0020) {
        // code for when R1 button is pushed
    }
    if (ctl->buttons() != 0x0020) {
        // code for when R1 button is released
    }

    //== PS4 R2 trigger button = 0x0080 ==//
    if (ctl->buttons() == 0x0080) {
        // code for when R2 button is pushed
        digitalWrite(SQUIRT_PIN, HIGH);
    }
    if (ctl->buttons() != 0x0080) {
        // code for when R2 button is released
        digitalWrite(SQUIRT_PIN, LOW);
    }

    //== PS4 L1 trigger button = 0x0010 ==//
    if (ctl->buttons() == 0x0010) {
        // code for when L1 button is pushed
    }
    if (ctl->buttons() != 0x0010) {
        // code for when L1 button is released
    }

    //== PS4 L2 trigger button = 0x0040 ==//
    if (ctl->buttons() == 0x0040) {
        // code for when L2 button is pushed
    }
    if (ctl->buttons() != 0x0040) {
        // code for when L2 button is released
    }

    // Define a common variable for speed adjustment
    int common_speed = 0;

    //== LEFT JOYSTICK - UP/DOWN (FORWARD/BACKWARD) ==//
    if (ctl->axisY() <= -70) {
        // moving forward
        common_speed = map(ctl->axisY(), -70, -520, 0, 255);
    } else if (ctl->axisY() >= 70) {
        // moving backward
        common_speed = map(ctl->axisY(), 70, 520, 0, -255);
    } else {
        // no forward/backward movement
        common_speed = 0;
    }

    //== LEFT JOYSTICK - LEFT/RIGHT (TURNING) ==//
    int turn_speed = 0;
    if (ctl->axisX() <= -70) {
        // turning left
        turn_speed = map(ctl->axisX(), -70, -520, 0, -255);
    } else if (ctl->axisX() >= 70) {
        // turning right
        turn_speed = map(ctl->axisX(), 70, 520, 0, 255);
    } else {
        // no turning movement
        turn_speed = 0;
    }

    //== RIGHT JOYSTICK - X AXIS (STRAFING LEFT/RIGHT) ==//
    int strafe_speed = 0;
    if (ctl->axisRX() <= -70) {
        // strafing left
        strafe_speed = map(ctl->axisRX(), -70, -520, 0, -255);
    } else if (ctl->axisRX() >= 70) {
        // strafing right
        strafe_speed = map(ctl->axisRX(), 70, 520, 0, 255);
    } else {
        // no strafing movement
        strafe_speed = 0;
    }

    // if current mode mode is manual then skip the motor output
    if (current_mode != MANUAL_MODE) { return; }
    // Combine forward/backward, turning, and strafing inputs for each motor
    move_motor(FRONT_LEFT, common_speed + turn_speed + strafe_speed);   // front-left motor
    move_motor(FRONT_RIGHT, common_speed - turn_speed + strafe_speed);  // front-right motor
    move_motor(BACK_LEFT, common_speed + turn_speed - strafe_speed);    // back-left motor
    move_motor(BACK_RIGHT, common_speed - turn_speed - strafe_speed);   // back-right motor

    //== RIGHT JOYSTICK - Y AXIS ==//
    if (ctl->axisRY()) {
        // code for when right joystick moves along y-axis
    }
    // dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.println("Start reached"); // DEL
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    leftInputSum = 0;
    leftInputMean = 0;
    rightInputSum = 0;
    rightInputMean = 0;
    robotMode = 0; // Initially seeking

    pinMode(LED, OUTPUT);
    pinMode(LEFT_FRONT_MOTOR_PIN_1, OUTPUT);
    pinMode(LEFT_FRONT_MOTOR_PIN_2, OUTPUT);
    pinMode(LEFT_FRONT_MOTOR_PIN_PWM, OUTPUT);
    pinMode(RIGHT_FRONT_MOTOR_PIN_1, OUTPUT);
    pinMode(RIGHT_FRONT_MOTOR_PIN_2, OUTPUT);
    pinMode(RIGHT_FRONT_MOTOR_PIN_PWM, OUTPUT);
    pinMode(LEFT_BACK_MOTOR_PIN_1, OUTPUT);
    pinMode(LEFT_BACK_MOTOR_PIN_2, OUTPUT);
    pinMode(LEFT_BACK_MOTOR_PIN_PWM, OUTPUT);
    pinMode(RIGHT_BACK_MOTOR_PIN_1, OUTPUT);
    pinMode(RIGHT_BACK_MOTOR_PIN_2, OUTPUT);
    pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
    pinMode(ULTRA_SONIC_TRIG_PIN, OUTPUT);
    // pinMode(LEFT_FIRE_TRIG_PIN, OUTPUT);
    // pinMode(RIGHT_FIRE_TRIG_PIN, OUTPUT);
    pinMode(SQUIRT_PIN, OUTPUT);

    // enforce low
    digitalWrite(LED, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_PIN_2, LOW);
    digitalWrite(LEFT_FRONT_MOTOR_PIN_PWM, LOW);
    digitalWrite(RIGHT_FRONT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_FRONT_MOTOR_PIN_2, LOW);
    digitalWrite(RIGHT_FRONT_MOTOR_PIN_PWM, LOW);
    digitalWrite(LEFT_BACK_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_BACK_MOTOR_PIN_2, LOW);
    digitalWrite(LEFT_BACK_MOTOR_PIN_PWM, LOW);
    digitalWrite(RIGHT_BACK_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_BACK_MOTOR_PIN_2, LOW);
    digitalWrite(RIGHT_BACK_MOTOR_PIN_PWM, LOW);
    digitalWrite(SQUIRT_PIN, LOW);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Delay before start to plug ESP32 in (necessary to start with proper input)
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }

    resetSymbols(); // Necessary symbols reset
    calcSensorData(); // Sensor data instance(s) recorded
    // digitalWrite(ULTRA_SONIC_TRIG_PIN, HIGH);
    // *** Edit so that MANUAL_MODE takes over when button press is detected *** //
    // *** Do ^this^ in modeLogic() *** //


    // *** Potentially start by spinning around *** //
    switch (current_mode) {
        case MANUAL_MODE: // Controller takes over
            /* -------------- logic for manual mode -------------- */
            // do nothing 
            break;
        case AUTOMATIC_MODE: // Sensor input
            /* -------------- logic for auto mode -------------- */
            
            // Serial.println(leftInputMean); // DEL
            // Serial.println(rightInputMean); // DEL
            // Serial.println(inputDiff); // DEL
            Serial.println(distanceCm); // DEL

            // modeLogic();
            // Serial.println(robotMode); // DEL
            // modeSwitch();
            basic_flame_search();

            break;
        default:
            Serial.println("Error: No central mode specified.");
    }

    inc++; // for the seed 

    // delay(TICK_TIME);
}