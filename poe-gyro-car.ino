//#define MORSE_DEBUG

#ifdef MORSE_DEBUG
#include <morse.h>
#endif

#include <Adafruit_MotorShield.h>
#include <FlySkyIBus.h>

#define PIN_STATUS 6

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_front_motor = AFMS.getMotor(4);
Adafruit_DCMotor *left_back_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_front_motor = AFMS.getMotor(3);
Adafruit_DCMotor *right_back_motor = AFMS.getMotor(2);

#ifdef MORSE_DEBUG
LEDMorseSender sender(PIN_STATUS);
#endif

void setup() {
  // put your setup code here, to run once:
  IBus.begin(Serial);
  AFMS.begin();
#ifdef MORSE_DEBUG
  sender.setup();
#endif
  left_front_motor->setSpeed(0);
  left_front_motor->run(FORWARD);
  left_back_motor->setSpeed(0);
  left_back_motor->run(FORWARD);
  right_front_motor->setSpeed(0);
  right_front_motor->run(FORWARD);
  right_back_motor->setSpeed(0);
  right_back_motor->run(FORWARD);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
#ifdef MORSE_DEBUG
  sender.setMessage(String("hi"));
  sender.startSending();
#endif
}

void move_forward(int16_t speed_value) {
  if (speed_value > 0) {
    left_front_motor->run(FORWARD);
    left_back_motor->run(FORWARD);
    right_front_motor->run(FORWARD);
    right_back_motor->run(FORWARD);
  } else {
    left_front_motor->run(BACKWARD);
    left_back_motor->run(BACKWARD);
    right_front_motor->run(BACKWARD);
    right_back_motor->run(BACKWARD);
    speed_value = -speed_value;
  }

  left_front_motor->setSpeed(speed_value);
  left_back_motor->setSpeed(speed_value);
  right_front_motor->setSpeed(speed_value);
  right_back_motor->setSpeed(speed_value);
}

void set_motor_speeds(int16_t turn, int16_t throttle, boolean killswitch) {
  if (killswitch) {
    left_front_motor->run(FORWARD);
    left_back_motor->run(FORWARD);
    right_front_motor->run(FORWARD);
    right_back_motor->run(FORWARD);
    left_front_motor->setSpeed(0);
    left_back_motor->setSpeed(0);
    right_front_motor->setSpeed(0);
    right_back_motor->setSpeed(0);
    return;
  }
  int16_t speed_value = 0;
  int16_t turn_rate_left = 0;
  int16_t turn_rate_right = 0;
  if (throttle > 8) {
    left_front_motor->run(FORWARD);
    left_back_motor->run(FORWARD);
    right_front_motor->run(FORWARD);
    right_back_motor->run(FORWARD);
    speed_value = throttle - 8;
    turn_rate_left = turn / 2;
    turn_rate_right = -turn_rate_left;
  } else if (throttle < -8) {
    left_front_motor->run(BACKWARD);
    left_back_motor->run(BACKWARD);
    right_front_motor->run(BACKWARD);
    right_back_motor->run(BACKWARD);
    speed_value = 8 - throttle;
    turn_rate_left = turn / -2;
    turn_rate_right = -turn_rate_left;
  } else {
    // throttle dead zone
    // no intended throttle, just turn
    if (turn > 8) {
      left_front_motor->run(FORWARD);
      left_back_motor->run(FORWARD);
      right_front_motor->run(BACKWARD);
      right_back_motor->run(BACKWARD);

      turn_rate_left = (turn - 8) / 2;
      turn_rate_right = turn_rate_left;
    } else if (turn < 8) {
      left_front_motor->run(BACKWARD);
      left_back_motor->run(BACKWARD);
      right_front_motor->run(FORWARD);
      right_back_motor->run(FORWARD);
      turn_rate_left = (8 - turn) / 2;
      turn_rate_right = turn_rate_left;
    } else {
      // do nothing
    }
  }

  left_front_motor->setSpeed(constrain(speed_value + turn_rate_left, 0, 100));
  left_back_motor->setSpeed(constrain(speed_value + turn_rate_left, 0, 100));
  right_front_motor->setSpeed(constrain(speed_value + turn_rate_right, 0, 100));
  right_back_motor->setSpeed(constrain(speed_value + turn_rate_right, 0, 100));
}

// left stick y channel = 2

int16_t convert_to_pos_neg_byte(uint16_t flysky_value) {
  int16_t pos_neg_byte =
      map(constrain(flysky_value, 1000, 2000), 1000, 2000, 0, 255 * 2);

  return pos_neg_byte - 255;
}

int left_stick_y = 0;
void loop() {
  IBus.loop();
  int16_t right_stick_x = convert_to_pos_neg_byte(IBus.readChannel(0));
  int16_t right_stick_y = convert_to_pos_neg_byte(IBus.readChannel(1));
  boolean killswitch = IBus.readChannel(4) < 1500;
  if (killswitch) {
    analogWrite(6, 255);
  } else {
    analogWrite(6, 0);
  }

#ifdef MORSE_DEBUG
  if (!sender.continueSending()) {
    uint16_t switch_value = IBus.readChannel(4);
    sender.setMessage(String(" .") + String(switch_value));
    sender.startSending();
  }
#endif

  //  analogWrite(6, right_stick_x);
  //    move_forward(right_stick_y);
  set_motor_speeds(right_stick_x, right_stick_y, killswitch);
}
