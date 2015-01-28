
package org.usfirst.frc.team806.robot;


import org.usfirst.frc.team806.robot.XboxController.AxisType;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

public class Robot extends SampleRobot {
	
	SpeedController lift1 = new Talon(4);
	SpeedController lift2 = new Talon(5);
	
	SpeedController rearRight = new Victor(0);
    SpeedController rearLeft = new Victor(1);
	SpeedController frontRight = new Victor(2);
	SpeedController frontLeft = new Victor(3);
	
	RobotDrive drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

	static Encoder encodFL = new Encoder(4, 5, true, EncodingType.k4X);
	static Encoder encodFR = new Encoder(6, 7, true, EncodingType.k4X);
	static Encoder encodRR = new Encoder(2, 3, true, EncodingType.k4X); 
	static Encoder encodRL = new Encoder(0, 1, true, EncodingType.k4X);

	PIDController EFL = new PIDController(0.6, .001, 0.1, encodFL, frontLeft);
	PIDController EFR = new PIDController(0.6, .001, 0.1, encodFR, frontRight);
	PIDController ERR = new PIDController(0.6, .001, 0.1, encodRR, rearRight);
	
    private static final Hand LEFTHAND = Hand.kLeft;
	private static final Hand RIGHTHAND = Hand.kRight;
	
	XboxController cont1 = new XboxController(0);
	XboxController cont2 = new XboxController(1);
	
	CameraServer server;

    public Robot() {
    	server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
        
        drive.setExpiration(0.1);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
		drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

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
    }
    
    public void disabled() {
        
    }
    public void robotInit(){
    	
    }

    public void autonomous() {
    	
    	encodRL.reset();
    	encodRR.reset();
    	encodFR.reset();
    	encodFL.reset();
    	
    	PIDController ERL = new PIDController(SmartDashboard.getNumber("DB/Slider 0"), SmartDashboard.getNumber("DB/Slider 1"), SmartDashboard.getNumber("DB/Slider 2"), encodRL, rearLeft);
    	
    	ERL.enable();
    //	ERR.enable();
    //	EFR.enable();
    //	EFL.enable(); 
    	
  
    	ERL.setPercentTolerance(15);
    //	ERL.setAbsoluteTolerance(200);
    //	ERR.setAbsoluteTolerance(200);
    //	EFR.setAbsoluteTolerance(200);
    //	EFL.setAbsoluteTolerance(200);
    	
     	
    	ERL.setSetpoint(SmartDashboard.getNumber("DB/Slider 3") * 1000);
    //	ERR.setSetpoint(-1000);
    //	EFR.setSetpoint(-1000);
    //	EFL.setSetpoint(1000);
    	
      drive.setSafetyEnabled(false);

      }

	public void operatorControl() {
		drive.setSafetyEnabled(true);
		long count = 0;
		double driveMultiplier = .5;
		double liftAxis;
		
		while (isEnabled() && isOperatorControl()) {
			Timer.delay(.05);

			if (cont1.getBumper(LEFTHAND) == true) {
				driveMultiplier = .25;
			} else if (cont1.getBumper(RIGHTHAND) == true) {
				driveMultiplier = .9;
			} else {
				driveMultiplier = .5;
			}

			liftAxis = (-1 * cont1.getAxis(AxisType.kTriggerLeft)) + cont1.getAxis(AxisType.kTriggerRight);
			lift1.set(SetDeadZone(liftAxis, .1)*.75);
			lift2.set(SetDeadZone(liftAxis, .1)*.75);
			
			if (count % 2 == 0) { // adjust as required to drive smoothly
				double axisX = SetDeadZone(cont1.getX(LEFTHAND), .25) * driveMultiplier; //strafing
				double axisY = SetDeadZone(cont1.getY(LEFTHAND), .25) * driveMultiplier; //forward and back
				double rotation = SetDeadZone(cont1.getX(RIGHTHAND), .15) * driveMultiplier; //rotation
				
				drive.mecanumDrive_Cartesian(axisX, axisY, rotation, 0);
			}
			 
			// SmartDashboard.putString("DB/String 0", "Button 0: " + SmartDashboard.getBoolean("DB/Button 0"));
			// SmartDashboard.putString("DB/Button 0", "Test");
			
			SmartDashboard.putString("DB/String 0", "Encoder FL Raw: ");
			SmartDashboard.putString("DB/String 5",  String.valueOf(encodFL.getRaw()));
			
			SmartDashboard.putString("DB/String 1", "Encoder FR Raw: ");
			SmartDashboard.putString("DB/String 6",  String.valueOf(encodFR.getRaw()));
			
			SmartDashboard.putString("DB/String 2", "Encoder RL Raw: ");
			SmartDashboard.putString("DB/String 7",  String.valueOf(encodRL.getRaw()));

			SmartDashboard.putString("DB/String 3", "Encoder RR Raw: ");
			SmartDashboard.putString("DB/String 8",  String.valueOf(encodRR.getRaw()));
			
			SmartDashboard.putString("DB/String 4", "Loop Count: ");
			SmartDashboard.putString("DB/String 9",  String.valueOf(count));
			
			SmartDashboard.getBoolean("DB/Button 1");
			
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
}