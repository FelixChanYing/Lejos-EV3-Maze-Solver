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

public class ColourFlip {

	@SuppressWarnings("resource")
	public static void main(String[] args)
	{
		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		Port p = LocalEV3.get().getPort("S"+4);
		SensorModes sensor = new EV3ColorSensor(p);
		final int SW = g.getWidth();
	    final int SH = g.getHeight();
	    double epsilon = 0.01;
	    SensorMode sm1 =sensor.getMode(2);
	    SensorMode sm =sensor.getMode(1);
	    
		g.clear();
        g.setFont(Font.getSmallFont());
        g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        g.drawString("to set color", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        g.clear();
        int sampleSize1 = sm1.sampleSize();
		float samples1[] = new float[sampleSize1];
		
		int sampleSize = sm.sampleSize();
		float samples[] = new float[sampleSize];
		
		sm1.fetchSample(samples1, 0);
		g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
		g.drawString("to continue", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        g.clear();
		do
		{
			SensorMode sm2 =sensor.getMode(2);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize2 = sm2.sampleSize();
			float samples2[] = new float[sampleSize2];
			sm2.fetchSample(samples2, 0);
			for(int i =0; i<sampleSize2;i++)
			{
				if ((Math.abs(samples2[0] - samples1[0]) < epsilon) && 
						(Math.abs(samples2[1] - samples1[1]) < epsilon) && 
						(Math.abs(samples2[2] - samples1[2]) < epsilon))
					{
						g.clear();
						LCD.drawString("SAME: " + i, 0, 4);
					}
				else 
					{
						LCD.drawString("NOT SAME: " + (Math.abs(samples2[i] - samples1[i])), 0, 4);
					}
			}
			
			sm.fetchSample(samples, 0);
			LCD.drawString("Intensity" +samples[0], 0, 5);
			
		}
		while (Button.ESCAPE.isUp());
	}

}
