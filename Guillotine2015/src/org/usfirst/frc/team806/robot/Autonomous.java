package org.usfirst.frc.team806.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
	
	private SpeedController leftRearDrive;
	private SpeedController rightRearDrive;
	private SpeedController rightFrontDrive;
	private SpeedController leftFrontDrive;
	
	private SpeedController lift1;
	private SpeedController lift2;
	
	private Encoder leftFrontEnc;
	private Encoder rightFrontEnc;
	private Encoder leftRearEnc;
	private Encoder rightRearEnc;
	
	private Drive drive;
	private Gyro gyro;
	
	private BuiltInAccelerometer accel;
		
	public Autonomous(SpeedController leftR, SpeedController rightR, SpeedController leftF, SpeedController rightF, 
						SpeedController lift1, SpeedController lift2, Encoder leftFEnc, Encoder rightFEnc, Encoder leftREnc, Encoder rightREnc, Drive drive, Gyro gyro)
	{
		this.leftRearDrive = leftR;
		this.rightRearDrive = rightR;
		this.leftFrontDrive = leftF;
		this.rightFrontDrive = rightF;
		this.lift1 = lift1;
		this.lift2 = lift2;
		
		this.leftFrontEnc = leftFEnc;
		this.rightFrontEnc = rightFEnc;
		this.leftRearEnc = leftREnc;
		this.rightRearEnc = rightREnc;
		
		this.drive = drive;
		this.gyro = gyro;
	}
	
	public void start(int select){
		accel = new BuiltInAccelerometer();
		
		this.ResetEncoders();
		
		switch (select){
		case 1:  //far right drive around
			StartLiftTote();
			MoveForward(2300,.30);
			Turn(75, -1, .25);
			Wait(806);
			MoveForward(13500, .25);
			Turn(75, 1, .25);
			Wait(403);
			//MoveForward(7000, .30);
			Lift(2000, -.6);
			
			break;
		case 2:
			StartLiftTote();
			MoveForward(2300,.30);
			LegacyTurn(rightRearEnc, 1700, -.47);
			Wait(806);
			MoveForward(4500, .25);
			LegacyTurn(leftRearEnc, 1650, .47);
			Wait(403);
			//MoveForward(7000, .30);
			Lift(2000, -.625);
			
			break;
		case 3:  // drive over ramp
			StartLiftTote();
			MoveForward(9500, .5);
			Lift(2000, -.625);
			break;
		case 4:
			StartLiftTote();
			MoveForward(7500, .5);
			Lift(2000, -.625);
			break;
		}
		
		
	}
	
	private void ResetEncoders(){
		this.leftFrontEnc.reset();
		this.leftRearEnc.reset();
		this.rightFrontEnc.reset();
		this.rightRearEnc.reset();
	}
	
	private void ResetDrive(){
		rightFrontDrive.set(0);
		leftFrontDrive.set(0);
	    leftRearDrive.set(0);
		rightRearDrive.set(0);
	}
	
	private void StartLiftTote(){
		MoveForward(400, .3);
		Lift(2000, .7);
	}
	
	private void Wait(int msTime){
		long time = System.currentTimeMillis();
		
		while (System.currentTimeMillis() < time + msTime){
		}
	}
	
	private void OutputEncoders(){
		SmartDashboard.putString("DB/String 0", "Encoder FL Raw: ");
		SmartDashboard.putString("DB/String 5",  String.valueOf(leftFrontEnc.getRaw()));
		
		SmartDashboard.putString("DB/String 1", "Encoder FR Raw: ");
		SmartDashboard.putString("DB/String 6",  String.valueOf(rightFrontEnc.getRaw()));
		
		SmartDashboard.putString("DB/String 2", "Encoder RL Raw: ");
		SmartDashboard.putString("DB/String 7",  String.valueOf(leftRearEnc.getRaw()));

		SmartDashboard.putString("DB/String 3", "Encoder RR Raw: ");
		SmartDashboard.putString("DB/String 8",  String.valueOf(rightRearEnc.getRaw()));
		
		SmartDashboard.putString("DB/String 4", "Get Gyro: ");
		SmartDashboard.putString("DB/String 9",  String.valueOf(gyro.getAngle()));
		
	}
	
	// slow speed .3, faster speed .35
	private void MoveForward(int distance, double speed){
    	while (rightRearEnc.getRaw() + leftRearEnc.getRaw() < distance){
    		rightFrontDrive.set(-1*speed);
    		leftFrontDrive.set(speed);
    	    leftRearDrive.set(speed);
    		rightRearDrive.set(-1*speed);
    	}
		/*this.gyro.reset();
		while (rightRearEnc.getRaw() + leftRearEnc.getRaw() < distance){
			if (Math.round(rightRearEnc.getRaw())%10 == 0)
				this.gyro.reset();
			drive.mecanumDrive_Cartesian(0, speed*-1, 0, gyro.getAngle());
		}*/
    	this.ResetEncoders();
    	this.ResetDrive();
	}

	 private void Turn(int distance, int direction, double speed){
		 this.gyro.reset();
		 while (Math.abs(gyro.getAngle()) <= distance){
    		rightFrontDrive.set(direction*speed);
    		leftFrontDrive.set(direction*speed);
    	    leftRearDrive.set(direction*speed);
    		rightRearDrive.set(direction*speed);
    		OutputEncoders();
		 }
    	this.gyro.reset();
    	this.ResetDrive();
	 }
	 
	 
	 private void LegacyTurn(Encoder encoder, int distance, double speed){
		 while (encoder.getRaw() < distance){
			 this.drive.mecanumDrive_Cartesian(0, 0, speed, 0);
			 OutputEncoders();
		 }
    	this.ResetEncoders();
    	this.ResetDrive();
	 }
	 
	 private void Lift(int msTime, double speed){
		 long time = System.currentTimeMillis();
		 
		 while (System.currentTimeMillis() <= (time + msTime)){
			 lift1.set(speed);
			 lift2.set(speed);
		 }
		 
		 lift1.set(0);
		 lift2.set(0);
	 }
	 
	 private void EncoderCorrectSpeeds(){
		 double RRval, LRVal, RFVal, LFVal;
		 double RRStart, LRStart, RFStart, LFStart;
		 double avg;
		 
		 RRStart = rightRearEnc.getRaw();
		 LRStart = leftRearEnc.getRaw();
		 RFStart = rightFrontEnc.getRaw();
		 LFStart = leftFrontEnc.getRaw();
		 
		 avg = (RRStart + LRStart + RFStart + LFStart)/4;
		 
		 RRval = (RRStart + avg)/avg;
		 LRVal = (LRStart + avg)/avg;
		 RFVal = (RFStart + avg)/avg;
		 LFVal = (LFStart + avg)/avg;
	 }
}
