#if !defined(COMMANDS_H)
#define COMMANDS_H


#define SUCTION_FAN 18
#define BLOWER_FAN 19

#define STATE_OFF 0
#define STATE_FAN_1 1
#define STATE_FAN_2 2

extern "C"{
    int motorA1 = 25; // IN1 on the L298N
    int motorA2 = 26; // IN2 on the L298N
    int motorAEnable = 27; // ENA on the L298N

    // Motor B
    int motorB1 = 32; // IN3 on the L298N
    int motorB2 = 33; // IN4 on the L298N
    int motorBEnable = 14; // ENB on the L298N

    int pwmMotorA = 110;
    int pwmMotorB = 130;


    int currentState;
}

void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMovement();
void updateFans();

#endif // MOVEMENT_H
