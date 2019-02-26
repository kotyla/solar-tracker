#define INPUT_SIZE 30
#define COORDINATES_ADDR 0
#define ANGLE_CORRECTION_ADDR 12

// LIMIT SWITCHES PINS
#define MIN_ANGLE_PIN 2
#define MAX_ANGLE_PIN 3

// MOTOR PINS
#define MOTOR1_ENABLE 10
#define MOTOR1_IN1 9
#define MOTOR1_IN2 8
#define MOTOR2_ENABLE 7
#define MOTOR2_IN1 6
#define MOTOR2_IN2 5

#define MOTOR1_ANGLE_PIN A0

#define MOTOR1_ANGLE_INTERRUPT_PIN 18


int motor1_angle_value = 0;
volatile int motor1_interrupt_hits = 0;
volatile int motor1_interrupt_max = 0;
volatile int motor1_interrupt_current = 0;

volatile int motor2_interrupt_hits = 0;



int tmp = 0;

int min_angle_state = 0;
int max_angle_state = 0;
