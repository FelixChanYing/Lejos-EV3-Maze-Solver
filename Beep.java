import lejos.hardware.Sound;
import lejos.utility.Delay;


public class Beep 
{
	public static void main(String[] args)
	{
		Delay.msDelay(5000);
		Sound.beep();
	}

}
