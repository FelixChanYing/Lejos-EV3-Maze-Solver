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
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
public class Test extends Thread
{
    GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
    final int SW = g.getWidth();
    final int SH = g.getHeight();
    final int SETUP_DELAY = 5000;
   	
	double integral;
	double maxintensity;
	double minintensity;
	double amplifier = 100;
	
	double error = 0;
	double lasterror;
	double turnangle;
	int speed= 100;
		
	Port p = LocalEV3.get().getPort("S"+4);
	SensorModes lightsensor = new EV3ColorSensor(p);	
	RegulatedMotor leftmotor = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor rightmotor = new EV3LargeRegulatedMotor(MotorPort.C);
	Image soup = new Image(64, 69, new byte[] {(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0xc0, (byte) 0xff, (byte) 0xff, 
			(byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0xfe, (byte) 0xff, (byte) 0xff, (byte) 0x7f, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0x7f, (byte) 0x00, 
			(byte) 0x00, (byte) 0xf8, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0xfc, (byte) 0x07, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0x1d, (byte) 0x08, (byte) 0x1c, 
			(byte) 0xfc, (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xff, (byte) 0xff, 
			(byte) 0xff, (byte) 0xff, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xff, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0x1b, 
			(byte) 0xff, (byte) 0xff, (byte) 0xbf, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0x27, (byte) 0xff, (byte) 0x7f, 
			(byte) 0x8e, (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0x93, (byte) 0xff, (byte) 0xbf, (byte) 0xc6, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xf3, (byte) 0xff, 
			(byte) 0x1f, (byte) 0xd7, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0xeb, (byte) 0xff, (byte) 0x9d, (byte) 0xa1, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xab, 
			(byte) 0x82, (byte) 0x88, (byte) 0xaa, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0x4d, (byte) 0x8a, (byte) 0xaa, 
			(byte) 0xaa, (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0x5b, (byte) 0x51, (byte) 0x02, (byte) 0x01, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0x1b, (byte) 0x55, 
			(byte) 0x55, (byte) 0xc0, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0xff, (byte) 0x09, (byte) 0x64, (byte) 0xff, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xff, 
			(byte) 0xbf, (byte) 0xff, (byte) 0xff, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0xff, (byte) 0x9f, (byte) 0xff, 
			(byte) 0xff, (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xff, (byte) 0xaf, 
			(byte) 0x55, (byte) 0xff, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xff, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xff, 
			(byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0xff, (byte) 0xff, (byte) 0xff, 
			(byte) 0xff, (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xff, (byte) 0xff, 
			(byte) 0xff, (byte) 0xff, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0xff, (byte) 0xff, (byte) 0xfc, (byte) 0xff, 
			(byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0xff, 
			(byte) 0xff, (byte) 0xfd, (byte) 0xff, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0xfe, (byte) 0x7f, (byte) 0xef, 
			(byte) 0x7f, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0xf0, (byte) 0xff, (byte) 0xe7, (byte) 0x0f, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0xfc, 
			(byte) 0x2e, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0xd0, (byte) 0x3f, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0xc0, (byte) 0x0f, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0xe0, (byte) 0x1e, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0xc0, (byte) 0x07, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0x48, (byte) 0x04, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0x18, 
			(byte) 0x44, (byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x80, (byte) 0x48, (byte) 0x2c, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x4b, (byte) 0x6d, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x01, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x05, (byte) 0x60, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x27, 
			(byte) 0x59, (byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0xd5, (byte) 0x46, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0xd4, (byte) 0x5a, (byte) 0x01, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x80, (byte) 0x03, (byte) 0x80, (byte) 0x83, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x80, (byte) 0xf5, 
			(byte) 0xd6, (byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x96, (byte) 0x96, (byte) 0x01, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x80, 
			(byte) 0xb7, (byte) 0xd6, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x06, (byte) 0x60, (byte) 0x0c, 
			(byte) 0x40, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x18, (byte) 0x00, (byte) 0x00, (byte) 0x38, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0xc0, (byte) 0x00, 
			(byte) 0x00, (byte) 0x06, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0xf8, (byte) 0x3f, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x00, 
			(byte) 0x00, (byte) 0x00, });
	
	void setup()
	{
	SensorMode sm =lightsensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
	int sampleSize = sm.sampleSize();
	float samples[] = new float[sampleSize];
	g.clear();
	g.drawRegion(soup, 0, 0, soup.getWidth(), soup.getHeight(), 0, SW / 2, SH / 4+10, GraphicsLCD.HCENTER | GraphicsLCD.VCENTER);
        g.drawString("JAVA SOUPBOT!", SW/2, 3*SH/4+10, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
	Button.waitForAnyPress(SETUP_DELAY);
	g.clear();
	g.setFont(Font.getSmallFont());
        g.drawString("Please press any button", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        g.drawString("to set maximum intensity", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        sm.fetchSample(samples, 0);
        maxintensity = samples[0]*amplifier;
        g.clear();
        g.setFont(Font.getSmallFont());
        g.drawString("Please press any button", SW/2, SH/3, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        g.drawString("to set minimum intensity", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.waitForAnyPress();
        sm.fetchSample(samples, 0);
        minintensity = samples[0]*amplifier;
        g.clear();
        LCD.drawString("Min:"+minintensity, 0, 4);
        LCD.drawString("Max:"+maxintensity, 0, 5);
        Button.waitForAnyPress();
        g.clear();
	}
	
	void PID()
	{
		do
		{
			SensorMode sm =lightsensor.getMode(1);//0=colorID 1=RED 2=RGB 3=Ambient 
			int sampleSize = sm.sampleSize();
			float samples[] = new float[sampleSize];
			sm.fetchSample(samples, 0);
			double midpoint = (maxintensity-minintensity)*0.5;//
			error = midpoint - samples[0]*amplifier;
			//integral=error+integral*0.5;
			//turnangle=0.01*error+.8*integral+(lasterror-error);
			//lasterror=error;
			turnangle=error;

			if (turnangle>0)
			{
				rightmotor.setSpeed((int) (speed+2*turnangle*turnangle)/2);
				rightmotor.forward();
				leftmotor.setSpeed((int) (speed+2*turnangle*turnangle)/2);
				leftmotor.backward();
			}

			if (turnangle<0)
			{
				rightmotor.setSpeed((int) (speed-0.1*turnangle*turnangle)/2);
				rightmotor.forward();
				leftmotor.setSpeed((int) (speed+3*turnangle*turnangle)/2);
				leftmotor.forward();
			}
			
			LCD.drawString("Turnangle: "+(int) turnangle, 0, 4);
			LCD.drawString("L: "+samples[0], 0 , 5);
			
		}while(Button.ESCAPE.isUp());
	}
	
	
	public static void main(String[] args)
	{
		Test follow = new Test();
		follow.setup();
		follow.PID();
	}
}
