import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class ColourIDScanner {

	@SuppressWarnings("resource")
	public static void main(String[] args)
	{
		Port p = LocalEV3.get().getPort("S"+3);
		SensorModes sensor = new EV3ColorSensor(p);
		do
		{
			LCD.drawString("ColorID: "+(((EV3ColorSensor) sensor).getColorID()), 0, 4);
			Delay.msDelay(500);
		}
		while (Button.ESCAPE.isUp());
	}

}
