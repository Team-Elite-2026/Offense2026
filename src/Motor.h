#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    int in1; 
    int in2; 
    int pwmPin; 
    int speed;  

public:
    Motor(int in1, int in2, int pwm);
    void setSpeed(double speed);
    void stop();

};

#endif // MOTOR_H
