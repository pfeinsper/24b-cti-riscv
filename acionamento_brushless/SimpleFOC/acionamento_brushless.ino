#include <SimpleFOC.h>

#define EN1 6
#define EN2 7
#define EN3 8
#define IN1 9
#define IN2 10
#define IN3 11
/* Hall Sensor Pins */
#define hall_1 3
#define hall_2 4
#define hall_3 5

/* Current Sensor Pins */
#define current_1 A0
#define current_2 A1
#define current_3 A2
 
#define pp 1


// HallSensor sensor instance - SPI
HallSensor sensor = HallSensor(hall_1, hall_2, hall_3, pp); // 1 par de polos

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(pp);
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN1, EN2, EN3);

// Interrupt routine initialization
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}


// CurrentSense instance - SPI
InlineCurrentSense current_sense = InlineCurrentSense(0.33, 20, current_1, current_2, current_3);


// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  // enable the debugging output
  SimpleFOCDebug::enable(&Serial);

  // initialise hall sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  
  // link driver
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);
  current_sense.init();

  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration based on the control type 
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;

  // default voltage_power_supply
  motor.voltage_limit = 12;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle loop controller
  motor.P_angle.P = 20;

  // angle loop velocity limit
  motor.velocity_limit = 50;

  // use monitoring with serial for motor init
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  // set the inital target value
  motor.target = 2;

  // define the motor id
  command.add('A', onMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));
  
  _delay(1000);
}


void loop() {
  // iterative setting of the FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable

  motor.move(3.14);
  
  // user communication
  command.run();
}