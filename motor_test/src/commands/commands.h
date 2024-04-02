#if !defined(COMMANDS_H)
#define COMMANDS_H


#define SUCTION_FAN 18
#define BLOWER_FAN 19

#define STATE_OFF 0
#define STATE_FAN_1 1
#define STATE_FAN_2 2

#define MOTOR_A 0
#define MOTOR_B 1


   extern int motorA1; // IN1 on the L298N
   extern int motorA2; // IN2 on the L298N
   extern int motorAEnable; // ENA on the L298N

    // Motor B
   extern int motorB1; // IN3 on the L298N
   extern int motorB2; // IN4 on the L298N
   extern int motorBEnable; // ENB on the L298N

   extern int pwmMotorA;
   extern int pwmMotorB;

   extern int currentState;


void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMovement();
void updateFans();
void setPwmMotorA(int pwmValue);
void setPwmMotorB(int pwmValue);


#endif // MOVEMENT_H
