import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class RGBScanner {

	@SuppressWarnings("resource")
	public static void main(String[] args)
	{
		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		Port p = LocalEV3.get().getPort("S"+4);
		SensorModes sensor = new EV3ColorSensor(p);
		final int SW = g.getWidth();
	    final int SH = g.getHeight();
	    double epsilon = 0.00075;
	    SensorMode sm1 =sensor.getMode(2);
	    
		g.clear();
        g.setFont(Font.getSmallFont());
        g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        g.drawString("to set color", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        g.clear();
        int sampleSize1 = sm1.sampleSize();
		float samples1[] = new float[sampleSize1];
		sm1.fetchSample(samples1, 0);
		g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		g.drawString("to continue", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        g.clear();
		do
		{
			SensorMode sm2 =sensor.getMode(2);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize = sm2.sampleSize();
			float samples[] = new float[sampleSize];
			sm2.fetchSample(samples, 0);
			for(int i =0; i<sampleSize;i++)
			{
				if (Math.abs(samples[i] - samples1[i]) < epsilon)
					{
						g.clear();
						LCD.drawString("SAME: " + i, 0, 4);
						Sound.beep();
					}
				else 
					{
						LCD.drawString("NOT SAME: " + (Math.abs(samples[i] - samples1[i])), 0, 4);
					}
			}
			Delay.msDelay(500);
		}
		while (Button.ESCAPE.isUp());
	}

}
