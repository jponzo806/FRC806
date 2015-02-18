
package org.usfirst.frc.team806.robot;


import org.usfirst.frc.team806.robot.XboxController.AxisType;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	
	SpeedController lift1 = new Talon(4);
	SpeedController lift2 = new Talon(5);
	
	SpeedController rearRight = new Victor(0);
    SpeedController rearLeft = new Victor(1);
	SpeedController frontRight = new Victor(2);
	SpeedController frontLeft = new Victor(3);
	
	Drive drive = new Drive(frontLeft, rearLeft, frontRight, rearRight);

	static Encoder encodFL = new Encoder(4, 5, true, EncodingType.k4X);
	static Encoder encodFR = new Encoder(6, 7, true, EncodingType.k4X);
	static Encoder encodRR = new Encoder(2, 3, true, EncodingType.k4X); 
	static Encoder encodRL = new Encoder(0, 1, true, EncodingType.k4X);
	
	Gyro gyro = new Gyro(0);

    private static final Hand LEFTHAND = Hand.kLeft;
	private static final Hand RIGHTHAND = Hand.kRight;
	
	XboxController cont1 = new XboxController(0);
	XboxController cont2 = new XboxController(1);
	
	DigitalInput lowerLimit = new DigitalInput(9);
	DigitalInput upperLimit = new DigitalInput(8);
	
	CameraServer server;
	
	Autonomous autonomous;

    public Robot() {
    	server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
        
        drive.setExpiration(0.1);
        drive.setInvertedMotor(Drive.MotorType.kFrontLeft, false);
		drive.setInvertedMotor(Drive.MotorType.kRearLeft, false);
		drive.setInvertedMotor(Drive.MotorType.kFrontRight, true);
		drive.setInvertedMotor(Drive.MotorType.kRearRight, true);

		encodFL.setMaxPeriod(.1);
		encodFL.setMinRate(10);
		encodFL.setDistancePerPulse(5);
		encodFL.setReverseDirection(true);
		encodFL.setSamplesToAverage(7);
		
		
		encodFR.setMaxPeriod(.1);
		encodFR.setMinRate(10);
		encodFR.setDistancePerPulse(5);
		encodFR.setReverseDirection(true);
		encodFR.setSamplesToAverage(7);
		
		
		encodRL.setMaxPeriod(.1);
		encodRL.setMinRate(10);
		encodRL.setDistancePerPulse(5);
		encodRL.setReverseDirection(true);
		encodRL.setSamplesToAverage(7);
		

		encodRR.setMaxPeriod(.1);
		encodRR.setMinRate(10);
		encodRR.setDistancePerPulse(5);
		encodRR.setReverseDirection(true);
		encodRR.setSamplesToAverage(7);
		
		autonomous = new Autonomous(rearLeft, rearRight, frontLeft, frontRight, lift1, lift2, encodFL, encodFR, encodRL, encodRR, drive, gyro, lowerLimit, upperLimit);
		
		PrintInstructions();
    }
    
    public void disabled() {
    	autonomous.Stop();
        
    }
    public void robotInit(){
    	PrintInstructions();
    }

    public void autonomous() {
    		
      drive.setSafetyEnabled(false);
      if (SmartDashboard.getNumber("DB/Slider 2") > 2)
    	  SmartDashboard.putNumber("DB/Slider 2", 2);
      
      autonomous.start((int)SmartDashboard.getNumber("DB/Slider 0"), (int)SmartDashboard.getNumber("DB/Slider 1"), (int)SmartDashboard.getNumber("DB/Slider 2"));
      
      
      }

	public void operatorControl() {
		autonomous.Stop();
		drive.setSafetyEnabled(true);
		long count = 0;
		double driveMultiplier = .5;
		double liftAxis;
		gyro.reset();
		
		while (isEnabled() && isOperatorControl()) {
			autonomous.Stop();
			Timer.delay(.05);

			if (cont1.getBumper(LEFTHAND) == true) {
				driveMultiplier = .25;
			} else if (cont1.getBumper(RIGHTHAND) == true) {
				driveMultiplier = .9;
			} else {
				driveMultiplier = .5;
			}

			liftAxis = SetDeadZone((-1 * cont1.getAxis(AxisType.kTriggerLeft)) + cont1.getAxis(AxisType.kTriggerRight), .1);
			if (liftAxis < 0 && !lowerLimit.get())
				liftAxis = 0;
			else if (liftAxis > 0 && !upperLimit.get())
				liftAxis = 0;
			
			lift1.set(liftAxis*.75);
			lift2.set(liftAxis*.75);
			
			if (count % 2 == 0) { // adjust as required to drive smoothly
				double axisX = SetDeadZone(cont1.getX(LEFTHAND), .25) * driveMultiplier; //strafing
				double axisY = SetDeadZone(cont1.getY(LEFTHAND), .25) * driveMultiplier; //forward and back
				double rotation = SetDeadZone(cont1.getX(RIGHTHAND), .15) * driveMultiplier; //rotation
				
				drive.mecanumDrive_Cartesian(axisX, axisY, rotation, 0);
			}			
			
			count++;
		}
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
	}

	/**
	 * Rounds a value to a specific number of decimal places.
	 * 
	 * @param value
	 *            The number to be evaluated.
	 * @param decimalPlaces
	 *            The number of decimal places for the number to be shortened
	 *            to.
	 * @return The rounded number.
	 */
	private static double RoundToDecimalPlace(double value, int decimalPlaces) {
		double exponent = 10 ^ decimalPlaces;
		int rounded = (int) (value * exponent);
		return (double) rounded / exponent;
	}
	
	private static double SetDeadZone(double value, double zone){
		if (value > zone) 
			return value;
		else if (value < (zone * -1))
			return value;
		else
			return 0;
	}
	
	private void PrintInstructions()
	{
		SmartDashboard.putString("DB/String 0",  "1: Right to landmark");
		SmartDashboard.putString("DB/String 1",  "2: Center to landmark");
		SmartDashboard.putString("DB/String 2",  "3: Left to landmark");
		SmartDashboard.putString("DB/String 3",  "4: Left drive straight");
		SmartDashboard.putString("DB/String 4",  "5: Straight over ramp");
		SmartDashboard.putString("DB/String 5",  "Slider 0: Autonomous select");
		SmartDashboard.putString("DB/String 6",  "Slider 1: Auto delay in (s)");
		SmartDashboard.putString("DB/String 7",  "Slider 2: Stack height(0-2)");
		SmartDashboard.putString("DB/String 8",  "");
		SmartDashboard.putString("DB/String 9",  "");
	}
}