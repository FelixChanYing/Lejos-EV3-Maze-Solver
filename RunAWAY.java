import lejos.robotics.RegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class RunAWAY 
{
	static RegulatedMotor leftmotor = new EV3LargeRegulatedMotor(MotorPort.B);
	static RegulatedMotor rightmotor = new EV3LargeRegulatedMotor(MotorPort.C);
	public static void main(String[] args)
	{
		rightmotor.setSpeed(1000);
		rightmotor.forward();
		leftmotor.setSpeed(1000);
		leftmotor.forward();
		Delay.msDelay(2000);
	}
}
