import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class GyroTest {

	@SuppressWarnings("resource")
	public static void main(String[] args)
	{
		Port p = LocalEV3.get().getPort("S"+2);
		SensorModes sensor = new EV3GyroSensor(p);
		do
		{
			SensorMode sm =sensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize = sm.sampleSize();
			float samples[] = new float[sampleSize];
			sm.fetchSample(samples, 0);
			LCD.drawString("Angle: "+samples[0], 0, 4);
			Delay.msDelay(500);
		}
		while (Button.ESCAPE.isUp());
	}

}
