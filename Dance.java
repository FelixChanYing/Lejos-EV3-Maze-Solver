import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class Dance 
{
	Port gyroport = LocalEV3.get().getPort("S"+2);
	SensorModes gyrosensor = new EV3GyroSensor(gyroport);
	RegulatedMotor leftmotor = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor rightmotor = new EV3LargeRegulatedMotor(MotorPort.C);
	
	double angle1 = 0;
	double angle2 = 0;
	
	int speed= 200;
	
	void turncw(double degrees)
	{
		do
		{
			
			SensorMode sm =gyrosensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize = sm.sampleSize();
			float samples[] = new float[sampleSize];
			sm.fetchSample(samples, 0);
			leftmotor.setSpeed(speed);
			leftmotor.forward();
			LCD.drawString("Angle1: "+samples[0], 0, 3);
			LCD.drawString("Angle2: "+angle2, 0, 4);
			angle1=samples[0];
			if(Button.ESCAPE.isDown())
				break;
		}
		while (-angle1+angle2<degrees);
		angle2=angle1;
		leftmotor.stop();
	}
	 
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
			LCD.drawString("Angle: "+samples[0], 0, 3);
			LCD.drawString("Angle2: "+angle2, 0, 4);
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
		Dance dance = new Dance();
		dance.turncw(90);
		dance.turnccw(180);
		dance.turncw(90);
		dance.turnccw(180);
	}

}
