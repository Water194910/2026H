#include "motor.h"

void Set_pwm(int motor1,int motor2)
{
	if(motor1>99)motor1=99;
	if(motor1<-99)motor1=-99;
	if(motor2>99)motor2=99;
	if(motor2<-99)motor2=-99;
	
	if(motor1>0)
	{
		AIN1_set;
		AIN2_reset;
		PWMA(motor1); 
	}
	else if(motor1<0)
	{
		AIN1_reset;
		AIN2_set;
		PWMA(-motor1); // 设置PWM值，绝对值转换
	}
	else
	{
		AIN1_reset;
		AIN2_reset;
		PWMA(0);
	}
	
	// 同样处理motor2
	if(motor2>0)
	{
		BIN1_set;
		BIN2_reset;
		PWMB(motor2);
	}
	else if(motor2<0)
	{
		BIN1_reset;
		BIN2_set;
		PWMB(-motor2);
	}
	else
	{
		BIN1_reset;
		BIN2_reset;
		PWMB(0);
	}
}

