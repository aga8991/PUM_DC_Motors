void Motor_move(uint8_t motor, uint8_t direct, uint8_t pwm);
void Stop(uint8_t motor);
void Stop_all();
void Clockwise(uint8_t motor, int speed_pwm);
void CounterCW(uint8_t motor, int speed_pwm);
void Forward(int speed_pwm);
void Backward(int speed_pwm);
void Increase_speed(uint8_t motor);
void Decrease_speed(uint8_t motor);
int motor_number(uint8_t motor);
float count_speed_rpm(uint8_t motor_pin);
void Assing_speed_rpm();
void Align_motors_speed(uint8_t motor1, uint8_t motor2);


//motor states
#define BREAK   0
#define CW      1
#define CCW     2

#define MOTOR_1 3
#define MOTOR_2 4
#define MOTOR_3 5
#define MOTOR_4 6

//first motor controller - motors 1 and 2
#define CW1    33   //clockwise
#define CCW1   31   //counter clockwise
#define EN_M1  A2   //enable
#define PWM1   11

#define CW2    37   //clockwise
#define CCW2   35   //under clockwise
#define EN_M2  A3   //enable
#define PWM2   12

//second motor controller - motors 3 and 4
#define CW3    49   //clockwise
#define CCW3   47   //under clockwise
#define EN_M3  A0   //enable
#define PWM3   8

#define CW4    53   //clockwise
#define CCW4   51   //under clockwise
#define EN_M4  A1   //enable
#define PWM4   9

//Motor 1 - encoder
#define MOTOR_A1_PIN  28
#define MOTOR_B1_PIN  26

//Motor 2 - encoder
#define MOTOR_A2_PIN  24
#define MOTOR_B2_PIN  22

//Motor 3 - encoder
#define MOTOR_A3_PIN  52
#define MOTOR_B3_PIN  50

//Motor 4 - encoder
#define MOTOR_A4_PIN  48
#define MOTOR_B4_PIN  46

int Speed_pwm[4] = {0, 0, 0, 0};  //default motor speed
float Speed_rpm[4] = {0.0, 0.0, 0.0, 0.0};
unsigned short Motor_Status[4] = { BREAK, BREAK, BREAK, BREAK };  // Motors are off at the beggining

void setup()
{
  //motors controllers
  pinMode(CW1, OUTPUT);
  pinMode(CW2, OUTPUT);
  pinMode(CW3, OUTPUT);
  pinMode(CW4, OUTPUT);

  pinMode(CCW1, OUTPUT);
  pinMode(CCW2, OUTPUT);
  pinMode(CCW3, OUTPUT);
  pinMode(CCW4, OUTPUT);

  pinMode(EN_M1, OUTPUT);
  pinMode(EN_M2, OUTPUT);
  pinMode(EN_M3, OUTPUT);
  pinMode(EN_M4, OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  //encoders;
  pinMode(MOTOR_A1_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B1_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A2_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B2_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A3_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B3_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A4_PIN, INPUT_PULLUP);
  pinMode(MOTOR_B4_PIN, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

}


void Motor_move(uint8_t motor, uint8_t direct, uint8_t pwm)         //function that controls the variables: motor(1-4), direction (cw/ccw), pwm (entra 0-255);
{
  if (motor == MOTOR_1)
  {
    if (direct == CW)
    {
      digitalWrite(CW1, HIGH);
      digitalWrite(CCW1, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW1, HIGH);
      digitalWrite(CW1, LOW);
    }
    else
    {
      digitalWrite(CW1, LOW);
      digitalWrite(CCW1, LOW);
    }

    analogWrite(PWM1, pwm);
  }
  else if (motor == MOTOR_2)
  {
    if (direct == CW)
    {
      digitalWrite(CW2, HIGH);
      digitalWrite(CCW2, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW2, HIGH);
      digitalWrite(CW2, LOW);
    }
    else
    {
      digitalWrite(CW2, LOW);
      digitalWrite(CCW2, LOW);
    }

    analogWrite(PWM2, pwm);
  }
  else if (motor == MOTOR_3)
  {
    if (direct == CW)
    {
      digitalWrite(CW3, HIGH);
      digitalWrite(CCW3, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW3, HIGH);
      digitalWrite(CW3, LOW);
    }
    else
    {
      digitalWrite(CW3, LOW);
      digitalWrite(CCW3, LOW);
    }

    analogWrite(PWM3, pwm);
  }
  else if (motor == MOTOR_4)
  {
    if (direct == CW)
    {
      digitalWrite(CW4, HIGH);
      digitalWrite(CCW4, LOW);
    }
    else if (direct == CCW)
    {
      digitalWrite(CCW4, HIGH);
      digitalWrite(CW4, LOW);
    }
    else
    {
      digitalWrite(CW4, LOW);
      digitalWrite(CCW4, LOW);
    }
    analogWrite(PWM4, pwm);
  }
}

void Stop(uint8_t motor) // stop a specific motor
{
  Motor_Status[motor_number(motor)] = BREAK;
  Motor_move(motor, Motor_Status[motor_number(motor)], 0);
}

void Stop_all()   // stop all motors
{
  Stop(MOTOR_1);
  Stop(MOTOR_2);
  Stop(MOTOR_3);
  Stop(MOTOR_4);
}

void Clockwise(uint8_t motor, int speed_pwm)   // go clockwise with a specific wheel
{
  Motor_Status[motor_number(motor)] = CW;
  Motor_move(motor, Motor_Status[motor_number(motor)], speed_pwm);
}

void CounterCW(uint8_t motor, int speed_pwm)   // go counter clockwise with a specific wheel
{
  Motor_Status[motor_number(motor)] = CCW;
  Motor_move(motor, Motor_Status[motor_number(motor)], speed_pwm);
}

void Forward(int speed_pwm)   // move straight forward (whole robot)
{
  Clockwise(MOTOR_1, speed_pwm);
  Clockwise(MOTOR_2, speed_pwm);
  CounterCW(MOTOR_3, speed_pwm);
  CounterCW(MOTOR_4, speed_pwm);

  Assing_speed_rpm();
  Align_motors_speed(MOTOR_1,MOTOR_2);
  Align_motors_speed(MOTOR_1,MOTOR_3);
  Align_motors_speed(MOTOR_1,MOTOR_4); 
}

void Backward(int speed_pwm)    // move straght backward
{
  Clockwise(MOTOR_3, speed_pwm);
  Clockwise(MOTOR_4, speed_pwm);
  CounterCW(MOTOR_1, speed_pwm);
  CounterCW(MOTOR_2, speed_pwm);

  Assing_speed_rpm();
  Align_motors_speed(MOTOR_1,MOTOR_2);
  Align_motors_speed(MOTOR_1,MOTOR_3);
  Align_motors_speed(MOTOR_1,MOTOR_4);
}

void Increase_speed(uint8_t motor)
{
  Speed_pwm[motor_number(motor)] = Speed_pwm[motor_number(motor)] + 1;       //increase speed (0,4% of maximum speed)
  if(Speed_pwm[motor_number(motor)] > 255)
  {
    Speed_pwm[motor_number(motor)] = 255;  
  }

  Motor_move(motor, Motor_Status[motor_number(motor)], Speed_pwm[motor_number(motor)]);
  delay(50);
}

void Decrease_speed(uint8_t motor)
{
  Speed_pwm[motor_number(motor)] = Speed_pwm[motor_number(motor)] - 1;       //decrease speed (0,4% of maximum speed)
  if(Speed_pwm[motor_number(motor)] < 0)
  {
    Speed_pwm[motor_number(motor)] = 0;  
  }

  Motor_move(motor, Motor_Status[motor_number(motor)], Speed_pwm[motor_number(motor)]);
  delay(50);
}

int motor_number(uint8_t motor)     // function that returns motor number
{
  int n = 0;
  
  if(motor == MOTOR_1){
    n = 0;
  } else if (motor == MOTOR_2){
    n = 1;
  } else if (motor == MOTOR_3){
    n = 2;
  } else if (motor == MOTOR_4){
    n = 3;
  }
  return n;
}

float count_speed_rpm(uint8_t motor_pin)
{
  float V = 0;
  unsigned long time1 = 0;
  unsigned long time2 = 0;
  int counter = 0;

  time1 = millis();
  do
  {
    if(motor_pin == RISING)
    {
      counter ++;
    }
  } while (counter < 180);  //encoder output 180 imp/obr
  time2 = millis();
  time2 = time1 - time2;
  time2 = time2/3600000;    //time in minutes
  
  return V = 1/time2;       //speed in rpm
}

void Assing_speed_rpm()
{
  Speed_rpm[0] = count_speed_rpm(MOTOR_A1_PIN);
  Speed_rpm[1] = count_speed_rpm(MOTOR_A2_PIN);
  Speed_rpm[2] = count_speed_rpm(MOTOR_A3_PIN);
  Speed_rpm[3] = count_speed_rpm(MOTOR_A4_PIN);
}

void Align_motors_speed(uint8_t motor1, uint8_t motor2)
{
  int a = motor_number(motor1);
  int b = motor_number(motor2);
  float error = 0.02;    //acceptable deviation 5%

  do {
    if(Speed_rpm[b] < ((1-error)*Speed_rpm[a]))
    {
      Increase_speed(motor2);
    } 
    else if(Speed_rpm[b] > ((1+error)*Speed_rpm[a]))
    {
      Decrease_speed(motor2);
    }
  } while (!((Speed_rpm[b] >= ((1-error)*Speed_rpm[a])) && (Speed_rpm[b] >= ((1+error)*Speed_rpm[a]))));
}
