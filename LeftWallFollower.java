import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.Image;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class LeftWallFollower extends Thread
{
    	GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
    	final int SW = g.getWidth();
    	final int SH = g.getHeight();
    	final int SETUP_DELAY = 5000;
   	
	double integral;
	double maxintensity;
	double minintensity;
	double amplifier = 100;
	double midpoint;
	
	double error = 0;
	double lasterror;
	double turnangle;
	
	int speed= 100;
	
	double angle1 = 0;
	double angle2 = 0;
	
	Port lightport = LocalEV3.get().getPort("S"+4);
	Port gyroport = LocalEV3.get().getPort("S"+2);
	SensorModes lightsensor = new EV3ColorSensor(lightport);
	SensorModes gyrosensor = new EV3GyroSensor(gyroport);
	
	RegulatedMotor leftmotor = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor rightmotor = new EV3LargeRegulatedMotor(MotorPort.C);
	/*
	SensorMode gyro =gyrosensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
	int gyrosampleSize = gyro.sampleSize();
	float angle[] = new float[gyrosampleSize];
	*/
	SensorMode light =lightsensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient
	int lightsampleSize = light.sampleSize();
	float lightSamples[] = new float[lightsampleSize];
	
	void setup()
	{	
		g.clear();
		g.setFont(Font.getSmallFont());
        	g.drawString("Please press any button", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        	g.drawString("to set maximum intensity", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        	Button.waitForAnyPress();
        	lightsensor.fetchSample(lightSamples, 0);
        	maxintensity = lightSamples[0]*amplifier;
        	g.clear();
        	g.setFont(Font.getSmallFont());
        	g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        	g.drawString("to set minimum intensity", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        	Button.waitForAnyPress();
        	lightsensor.fetchSample(lightSamples, 0);
        	minintensity = lightSamples[0]*amplifier;
        	g.clear();
        	midpoint = (maxintensity-minintensity)*-0.5;
        	LCD.drawString("Min:"+minintensity, 0, 2);
		LCD.drawString("Max:"+maxintensity, 0, 3);
		LCD.drawString("Mid:"+midpoint, 0, 4);
        	Button.waitForAnyPress();
	}
	

	void PID()
	{
		do
		{
			light.fetchSample(lightSamples, 0);
			
		//	gyro.fetchSample(angle, 0);
			
			error = midpoint - lightSamples[0]*amplifier;
			turnangle=error;

			if (turnangle>0)
			{
				rightmotor.setSpeed((int) (speed+(2*turnangle*turnangle)));
				rightmotor.forward();
				leftmotor.setSpeed((int) (speed+(2*turnangle*turnangle)));
				leftmotor.backward();
			}
			
			if (turnangle<0)
			{
				rightmotor.setSpeed((int) (speed-0.5*turnangle*turnangle));
				rightmotor.forward();
				leftmotor.setSpeed((int) (speed+3*turnangle*turnangle));
				leftmotor.forward();
			}
			
			//if (angle[0]<0 && turnangle<0)	break;
			
			LCD.drawString("Turnangle: "+(int) turnangle, 0, 4);
			LCD.drawString("L: "+lightSamples[0], 0 , 5);
			
		}while(Button.ESCAPE.isUp());
		Button.waitForAnyPress();
	}
	
	/*	
	void turncw(double degrees)
	{
	rightmotor.stop();
		do
		{
			sm.fetchSample(samples, 0);
			leftmotor.setSpeed(speed);
			leftmotor.forward();
			angle1=samples[0];
			if(Button.ESCAPE.isDown())
				break;
		}
		while (-angle1+angle2<degrees);
		angle2=angle1;
		leftmotor.stop();
	}
	*/
	void turnccw(double degree)
	{
		do
		{
			SensorMode sm =gyrosensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize = sm.sampleSize();
			float samples[] = new float[sampleSize];
			sm.fetchSample(samples, 0);
			rightmotor.setSpeed(speed);
			rightmotor.forward();
			angle1=samples[0];
			if(Button.ESCAPE.isDown())
				break;
		}
		while (angle1-angle2<degree);
		angle2=angle1;
		rightmotor.stop();
	}
	
	public static void main(String[] args)
	{
		LeftWallFollower follow = new LeftWallFollower();

		follow.setup();
		follow.PID();
	}
}
