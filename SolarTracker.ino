#include "Wire.h"
#include <Helios.h>
#include <DS1107H.h>
#include <SerialCommand.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <avr/wdt.h>

#include "_soft_reset.h"
#include "_helpers.h"

// FUNCTION DECLARATIONS
void motor(byte motor = 1, byte direction = 1, byte speed = 255);

#include "_defines_and_variables.h"



Helios helios;

DS1107H clock;
SerialCommand sCmd; 

struct config_c
{
  double latitude;
  double longitude;
} coordinates;

struct config_p
{
  double azimuth;
  double elevation;
  int angle_correction = 10;
} position;


int dir = 0; // direction


void setup()
{
  Serial.begin(9600);

  
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println("---- SOLAR TRACKER by Lukasz Kotyla ----");
  Serial.println("--------------- WELCOME ----------------");
  Serial.println("----------------------------------------");
  Serial.println();
  Serial.println("... initiating ...");

  #include "_command_definitions.h"

  EEPROM_readAnything(COORDINATES_ADDR, coordinates);
    // empty eeprom read fix
    //if(coordinates.latitude <= 0) coordinates.latitude = 0;
    //if(coordinates.longitude <= 0) coordinates.longitude = 0;
    
  EEPROM_readAnything(ANGLE_CORRECTION_ADDR, position.angle_correction);
    // empty eeprom read fix
    if(position.angle_correction <= 0) position.angle_correction = 0;

  // MOTOR A outputs
  pinMode(MOTOR1_ENABLE, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);

  // MIN MAX limit switch
  pinMode(MIN_ANGLE_PIN, INPUT);
  pinMode(MAX_ANGLE_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_ANGLE_INTERRUPT_PIN), count_motor1, RISING);
  

  Serial.println("Setup done... starting application");
}


unsigned long previousMillis = 0;
const long interval = 250;

void loop()
{
  sCmd.readSerial();

  
  
  
//  Serial.print("Wartosc ADC: "); Serial.println(motor1_position_value);
//  Serial.print("Kat: "); Serial.println(motor1_angle);
//  delay(500);
  


//16.36667,48.2 is for 48°12'N, 16°22'O (latitude and longitude of Vienna)
// odwrotnie || pierwszy parametr LONGINTUDE , drugi LATITUDE
// google maps podaje LAT LON
// CZ-WA Mireckiego - 50.783179, 19.149377
/*
      clock.getDate();  
      helios.calcSunPos(clock.year,clock.month,clock.dayOfMonth,
      clock.hour, clock.minute,clock.second,19.149377,50.783179); 
      
      showTime(clock);
      dAzimuth=helios.dAzimuth;show("dAzimuth",dAzimuth,true);
      dElevation=helios.dElevation;show("dElevation",dElevation,true);
*/
  
    
  
      
  
  
}



void show(char nameStr[], double val, boolean newline) {
  Serial.print(nameStr);  
  Serial.print("=");
  if (newline)
       Serial.println(val);
  else Serial.print(val);
}

void showTime(DS1107H timerChip) {
  Serial.print("UTC ''");
  Serial.print(timerChip.year, DEC);
  Serial.print("-");
  Serial.print(timerChip.month, DEC);
  Serial.print("-");
  Serial.print(timerChip.dayOfMonth, DEC);
  Serial.print(" T ");
  Serial.print(timerChip.hour, DEC);
  Serial.print(":");
  Serial.print(timerChip.minute, DEC);
  Serial.print(":");
  Serial.print(timerChip.second, DEC);
  Serial.print("Z");
  Serial.print(" | Day of week:");
  Serial.println(timerChip.dayOfWeek, DEC);
}


void set_time_command() {
  char *arg;
  String y,m,d,h,i,s,dow;
  byte year,month,day,hour,minute,second,dayofweek;
  char* dateTime[7];
  
  

  Serial.println("SETTING TIME AND DATE");
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    char* command = strtok(arg, "-");
    int counter = 0;
    while(command != 0) {
      dateTime[counter++] = command;
      command = strtok(0, "-");
    }

    if(counter != 7) {
      Serial.println(" # Invalid pattern of parameters");
    } else {

      
      y = dateTime[0];
      m = dateTime[1];
      d = dateTime[2];
      h = dateTime[3];
      i = dateTime[4];
      s = dateTime[5];
      dow = dateTime[6];
      year = y.toInt();
      month = m.toInt();
      day = d.toInt();
      hour = h.toInt();
      minute = i.toInt();
      second = s.toInt();
      dayofweek = dow.toInt();

    }
    clock.getDate();
    Serial.print(" # Current time is: "); showTime(clock);
    Serial.print(" # Setting up new time: UT ''");  Serial.print(year); Serial.print("-"); Serial.print(month); Serial.print("-"); Serial.print(day); Serial.print(" T "); Serial.print(hour); Serial.print(":"); Serial.print(minute); Serial.print(":"); Serial.print(second); Serial.print("Z | Day of week: "); Serial.print(dayofweek);
    clock.setDate(s.toInt(),i.toInt(),h.toInt(),dow.toInt(),d.toInt(),m.toInt(),y.toInt());
    clock.getDate();
    Serial.print(" # Now the time is: "); showTime(clock);
    Serial.println("DONE");
  }
  else {
    Serial.println(" # SetTime command |-> SetTime YY-MM-DD-H-i-s-dow (dow = day of week 1 - 7)");
  }
}

void get_time_command() {
  clock.getDate();
  Serial.print("Current time is: "); showTime(clock);
  Serial.println("DONE");
}

void get_coordinates_command() {
  Serial.println("Getting current cooridnates (saved in memory)");
  Serial.print(" # Current coordinates LAT / LON: "); Serial.print(coordinates.latitude, 6); Serial.print(" / "); Serial.println(coordinates.longitude, 6);
  Serial.println("DONE");
}

void set_coordinates_command() {

  char *arg;
  char* dateTime[2];
  double longitude, latitude;
  
  Serial.println("SETTING NEW COORDINATES");
  arg = sCmd.next();
  if (arg != NULL) {
    char* command = strtok(arg, ",");
    int counter = 0;
    while(command != 0) {
      dateTime[counter++] = command;
      command = strtok(0, ",");
    }
    if(counter != 2) {
      Serial.println(" # Invalid pattern of parameters");
    } else {
      
      double latitude = atof(dateTime[0]);
      double longitude = atof(dateTime[1]);

      Serial.print(" # Current coordinates LAT / LON: "); Serial.print(coordinates.latitude, 6); Serial.print(" / "); Serial.println(coordinates.longitude, 6);
      Serial.print(" # Setting coordinates LAT / LON: "); Serial.print(latitude, 6); Serial.print(" / "); Serial.println(longitude, 6);
      coordinates = { latitude, longitude };
      EEPROM_writeAnything(COORDINATES_ADDR, coordinates);
      EEPROM_readAnything(COORDINATES_ADDR, coordinates);
      Serial.print(" # New coordinates LAT / LON (read from memory): "); Serial.print(coordinates.latitude, 6); Serial.print(" / "); Serial.println(coordinates.longitude, 6);
    }
    
  } else {
    Serial.println(" # SetCoordinates command |-> SetCoordinates 50.783179-19.149377 -> SetCoordinates LATITUDE-LONGITUDE");
  }

  Serial.println("DONE");
}

// ANGLE CORRECTION
void get_angle_correction_command() {
  Serial.println("Angle correction (degrees)");
  EEPROM_readAnything(ANGLE_CORRECTION_ADDR, position.angle_correction);
  Serial.print(" # Current angle correction: "); Serial.print(position.angle_correction); Serial.println(" deg.");
  }
  
void set_angle_correction_command() {
  Serial.println("SETTING NEW ANGLE CORRECTION");
  char *arg;
  int angle;
  
  arg = sCmd.next();
  if (arg != NULL) {
    
    EEPROM_readAnything(ANGLE_CORRECTION_ADDR, position.angle_correction);
    Serial.print(" # Current angle: "); Serial.print(position.angle_correction); Serial.println(" deg.");
    
    angle = atoi( arg );
    Serial.print(" # Setting angle: "); Serial.print(angle); Serial.println(" deg.");
    position.angle_correction = angle;
    EEPROM_writeAnything(ANGLE_CORRECTION_ADDR, position.angle_correction);
    EEPROM_readAnything(ANGLE_CORRECTION_ADDR, position.angle_correction);

    Serial.print(" # New angle (read from memory): "); Serial.print(position.angle_correction); Serial.println(" deg.");
      
  } else {
    Serial.println(" # SetAngleCorrection command |-> SetAngleCorrection 30 -> SetAngleCorrection INT");
  }
  Serial.println("DONE");
  
}
// END ANGLE CORRECTION

// STATUS
void status_command() {
  Serial.println("STATUS");
  clock.getDate();
  Serial.print(" # Time: "); showTime(clock);
  Serial.print(" # Coordinates LAT/LON: "); Serial.print(coordinates.latitude, 6); Serial.print(" / "); Serial.println(coordinates.longitude, 6);
  Serial.print(" # Angle correction: "); Serial.print(position.angle_correction); Serial.println(" deg.");
}
// END STATUS

// POSITION
void get_position_command() {
  int azi_motor_angle;
  clock.getDate();
  helios.calcSunPos(clock.year,clock.month,clock.dayOfMonth, clock.hour, clock.minute,clock.second,coordinates.longitude,coordinates.latitude);

  azi_motor_angle = helios.dAzimuth - position.angle_correction;
  
  Serial.println("POSITION");
  Serial.print(" # at time: UTC "); Serial.print(clock.year); Serial.print("-"); Serial.print(clock.month); Serial.print("-"); Serial.print(clock.dayOfMonth); Serial.print(" T "); Serial.print(clock.hour); Serial.print(":"); Serial.print(clock.minute); Serial.print(":"); Serial.println(clock.second);
  Serial.print(" # at location: "); Serial.print(coordinates.latitude, 6); Serial.print(","); Serial.println(coordinates.longitude, 6);
  Serial.print(" # azimuth is: "); Serial.println(helios.dAzimuth);
  Serial.print(" # elevation is: "); Serial.println(helios.dElevation);
  Serial.print(" # azimuth angle correction: "); Serial.println(position.angle_correction);
  Serial.println(" # ------------");
  Serial.print(" # azimuth motor angle: "); Serial.println(azi_motor_angle);
  Serial.println("DONE");
}
// END POSITION


void reboot_command() {
  Serial.println("REBOOT SYSTEM");
  soft_reset();
}


void set_servo_command() {
  char *arg;
  int steps;
  arg = sCmd.next();
  if (arg != NULL) {
    steps = atoi( arg );

    Serial.println("Motor start");
    motor(1, steps, 10);
    delay(500);
    Serial.println("Motor stop");
    motor(1, 0);

  }
}


void motor(byte motor, byte direction, byte speed) {
  if(motor == 1) {
    analogWrite(MOTOR1_ENABLE, speed);
    if(direction == 0) {
      // stop
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
    } else if(direction == 1) {
      // left
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
    } else if(direction == 2) {
      // right
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
    }
  } else if(motor == 2) {
    analogWrite(MOTOR2_ENABLE, speed);
    if(direction == 0) {
      // stop
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
    } else if(direction == 1) {
      // left
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
    } else if(direction == 2) {
      // right
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
    }
  }
}

void count_motor1() {
  motor1_interrupt_hits++;
}



void calibrate_command() {

  motor_max();

  Serial.println("RESET IMPULSE COUNTER");
  motor1_interrupt_hits = 0;
  
  motor_min();

  motor1_interrupt_max = motor1_interrupt_hits;

  Serial.print("MAX POSITION INTERRUPTS: "); Serial.println(motor1_interrupt_max);
}


void set_azimuth_command() {
  if(motor1_interrupt_max == 0) {
    Serial.println("Calibrate first");
    calibrate_command();
  }

  int how_many_steps = 0;

  
  
  char *arg;
  int steps, angle;
  arg = sCmd.next();
  
  if (arg != NULL) {
    angle = atoi( arg );

    steps = map(angle, 0, 120, 0, motor1_interrupt_max);
    Serial.print("Old current hits "); Serial.println(motor1_interrupt_current);
    how_many_steps = steps - motor1_interrupt_current;
    //Serial.print("How many steps: "); Serial.println(how_many_steps);

    if(angle == 0)
    {
      motor_min();
      motor1_interrupt_current = 0;
    } 
    else if(how_many_steps > 0) 
    {
      motor1_interrupt_hits = 0;
      while(motor1_interrupt_hits < how_many_steps) {
        if(digitalRead(MAX_ANGLE_PIN) == HIGH) break;
        motor(1, 2, 60);
      }
      motor1_interrupt_current += how_many_steps;
      motor(1, 0);
    } 
    else if(how_many_steps < 0) 
    {
      motor1_interrupt_hits = 0;
      while(motor1_interrupt_hits < abs(how_many_steps)) {
        if(digitalRead(MIN_ANGLE_PIN) == HIGH) break;
        motor(1, 1, 60);
      }
      motor1_interrupt_current += how_many_steps;
      motor(1, 0);
    }
    Serial.print("Current hits "); Serial.println(motor1_interrupt_current);
  }
  
}


void motor_min() {
  while(digitalRead(MIN_ANGLE_PIN) == LOW) {
    motor(1, 1, 60);
  }
  motor(1, 0);
  Serial.println("MINIMUM REACHED");
}

void motor_max() {
  while(digitalRead(MAX_ANGLE_PIN) == LOW) {
    motor(1, 2, 60);
  }
  motor(1, 0);
  _spf("Maximum reached");
}



