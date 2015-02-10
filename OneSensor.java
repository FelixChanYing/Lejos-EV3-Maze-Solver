
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class OneSensor 
{
	
	private static final int Black = 0;
	private static final int White = 1;
	private static final int Purple = 2;

	@SuppressWarnings({ "resource" })
	public static void main(String[] args)
	{
		int maxspeed = 100;
		int halfspeed = 50;
		int samplingtime = 100;
		
		RegulatedMotor leftmotor = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor rightmotor = new EV3LargeRegulatedMotor(MotorPort.B);
		SensorModes sensor = new EV3ColorSensor(SensorPort.S3);
			
		while (((EV3ColorSensor) sensor).getColorID() != Purple)
		{
			switch (((EV3ColorSensor) sensor).getColorID()) 
			{
				case Black: rightmotor.setSpeed(maxspeed);
							rightmotor.forward();
							leftmotor.setSpeed(maxspeed);
							leftmotor.forward();
							Delay.msDelay(samplingtime);
							break;
				case White: rightmotor.setSpeed(maxspeed);
							rightmotor.forward();
							leftmotor.setSpeed(halfspeed);
							leftmotor.forward();
							Delay.msDelay(samplingtime);
							break;
				default:	break;
			}

		}
	}
}
