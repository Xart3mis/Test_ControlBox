#include "AccelStepper.h"
#include "Arduino.h"

#define MAX_MOTOR_ACCELERATION 1000L
#define MAX_MOTOR_SPEED 1250L
#define MOTOR_SPEED 750L
#define MOTOR_COUNT 4

#if !defined(MOTOR_COUNT) || !defined(MOTOR_SPEED) || \
    !defined(MAX_MOTOR_SPEED) || !defined(MAX_MOTOR_ACCELERATION)
#error "Please define MAX_MOTOR_ACCELERATION"
#error "Please define MAX_MOTOR_SPEED"
#error "Please define MOTOR_SPEED"
#error "Please define MOTOR_COUNT"
#endif

void _set_hwangles(int angles[MOTOR_COUNT], const long step_count, bool step,
                   const int motor_index);
void _flash_led(const unsigned int interval);

char blk[(MOTOR_COUNT * 8) + 9] = {0};
int angles[MOTOR_COUNT] = {0};
int steps[MOTOR_COUNT] = {0};

const int ENABLE_PINS[MOTOR_COUNT] PROGMEM = {54, 60, 66, 49};
const int STEP_PINS[MOTOR_COUNT] PROGMEM = {50, 56, 62, 53};
const int DIR_PINS[MOTOR_COUNT] PROGMEM = {52, 58, 64, 51};

bool led_state = 0;

unsigned long long currentMillis = millis();
unsigned long long prevMillis = 0;

AccelStepper *sev[MOTOR_COUNT];

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    pinMode(ENABLE_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
  }

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    sev[i] = new AccelStepper(sev[i]->DRIVER, STEP_PINS[i], DIR_PINS[i]);

    sev[i]->setAcceleration(MAX_MOTOR_ACCELERATION);
    sev[i]->setMaxSpeed(MAX_MOTOR_SPEED);
    sev[i]->setSpeed(MOTOR_SPEED);
  }
  delay(400);
}

void loop()
{
  // if (Serial.available() >= MOTOR_COUNT * 2 + 3)
  // {
  //   Serial.readBytesUntil('&', blk, MOTOR_COUNT * 8 + 9);

  //   readBlock(initial, angles, blk, (const char)'#');

  //   for (int i = 0; i < MOTOR_COUNT; i++)
  //   {
  //     Serial.print("angles: ");
  //     Serial.println(angles[i]);
  //   }

  //   Serial.println();
  // }

  _set_hwangles(angles, 20000, 1, -1);
  _flash_led(75);
}

void _flash_led(const unsigned int interval)
{
  currentMillis = millis();
  if (currentMillis - prevMillis >= interval)
  {
    prevMillis = currentMillis;
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
    Serial.println("beyet7arak");
  }
}

void _set_hwangles(int angles[MOTOR_COUNT], const long step_count, bool step,
                   const int motor_index)
{
  if (motor_index < 0)
  {
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      if (step)
      {
        sev[i]->moveTo(step_count);
      }
      else
      {
        sev[i]->moveTo((40ULL * angles[i]) / 9ULL);
      }

      sev[i]->run();
    }
    return;
  }
  else
  {
    if (step)
    {
      sev[motor_index]->moveTo(step_count);
    }

    else
    {
      sev[motor_index]->moveTo((40ULL * step_count) / 9ULL);
    }
    sev[motor_index]->run();
  }
}