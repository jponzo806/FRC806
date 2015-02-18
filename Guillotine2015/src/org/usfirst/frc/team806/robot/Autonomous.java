package org.usfirst.frc.team806.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
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
	
	private DigitalInput lowerLimit;
	private DigitalInput upperLimit;
	
	private Drive drive;
	private Gyro gyro;
	
	private boolean stop;
		
	public Autonomous(SpeedController leftR, SpeedController rightR, SpeedController leftF, SpeedController rightF, 
						SpeedController lift1, SpeedController lift2, Encoder leftFEnc, Encoder rightFEnc, Encoder leftREnc, Encoder rightREnc, Drive drive, Gyro gyro, 
						DigitalInput lowerLimit, DigitalInput upperLimit)
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
		
		this.lowerLimit = lowerLimit;
		this.upperLimit = upperLimit;
		
		this.stop = false;
	}
	
	public void Stop(){
		this.stop = true;
	}
	
	public void start(int select, int delay, int stackHeight){
		this.stop = false;
		
		Wait(delay * 1000);
		ResetEncoders();
		switch (select){
		
		case 1: //right side center drop
			StartLiftTote();
			MoveForward(10500, .3);
			Wait(403);
			Turn(75, -1, .25);
			Lift(2000*(stackHeight), .8);
			MoveForward(5300, .35);
			Lift(1800, -.625);
			MoveForward(2500, -.3);
			break;
		case 2: //middle position center drop
			StartLiftTote();
			MoveForward(10000, .3);
			Wait(403);
			Turn(75, -1, .25);
			Wait(403);
			MoveForward(800, -.3);
			Lift(1800, -.625);
			MoveForward(2500, -.3);
			break;
		case 3: //left side center drop
			StartLiftTote();
			MoveForward(8500, .4);
			Wait(403);
			Turn(75, 1, .30);
			Lift(2000*(stackHeight), .8);
			MoveForward(5250, .4);
			Lift(1800, -.625);
			MoveForward(2500, -.3);
			break;
		case 4: //left position drive straight
			StartLiftTote();
			MoveForward(8500, .5);
			Lift(1800, -.625);
			break;
		case 5:  // drive over ramp
			StartLiftTote();
			MoveForward(11000, .3);
			Lift(1650, -.625);
			break;
		case 6: // center position drive around ramp
			StartLiftTote();
			MoveForward(2300,.30);
			LegacyTurn(rightRearEnc, 1700, -.47);
			Wait(806);
			MoveForward(4500, .25);
			LegacyTurn(leftRearEnc, 1650, .47);
			Wait(403);
			MoveForward(7000, .30);
			Lift(2000, -.625);
			break;
		case 7:  //far right drive around ramp
			StartLiftTote();
			MoveForward(2300,.30);
			Turn(75, -1, .25);
			Wait(806);
			MoveForward(13500, .25);
			Turn(75, 1, .25);
			Wait(403);
			MoveForward(7000, .30);
			Lift(2000, -.6);		
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
		if (stop)
			return;
		long time = System.currentTimeMillis();
		
		while (System.currentTimeMillis() < time + msTime){
		}
	}
		
	// slow speed .3, faster speed .35
	private void MoveForward(int distance, double speed){
		if (stop)
			return;
		
		this.ResetEncoders();
		while (Math.abs(rightRearEnc.getRaw()) + Math.abs(leftRearEnc.getRaw()) < distance){
    		rightFrontDrive.set(-1*speed);
    		leftFrontDrive.set(speed);
    	    leftRearDrive.set(speed);
    		rightRearDrive.set(-1*speed);
    	}
    	this.ResetEncoders();
    	this.ResetDrive();
	}
	

	 private void Turn(int distance, int direction, double speed){
		 if (stop)
				return;
		 
		 this.gyro.reset();
		 this.ResetEncoders();
		 while (Math.abs(gyro.getAngle()) <= distance){
    		rightFrontDrive.set(direction*speed);
    		leftFrontDrive.set(direction*speed);
    	    leftRearDrive.set(direction*speed);
    		rightRearDrive.set(direction*speed);
    	 }
    	this.gyro.reset();
    	this.ResetDrive();
	 }
	 
	 
	 private void LegacyTurn(Encoder encoder, int distance, double speed){
		 if (stop)
				return;
		 
		 while (encoder.getRaw() < distance){
			 this.drive.mecanumDrive_Cartesian(0, 0, speed, 0);
		 }
    	this.ResetEncoders();
    	this.ResetDrive();
	 }
	 
	 private void Lift(int msTime, double speed){
		 if (stop)
				return;
		 
		 long time = System.currentTimeMillis();
		 
		 while (System.currentTimeMillis() <= (time + msTime)){
			 if ((speed < 0 && !lowerLimit.get()) && (speed > 0 && !upperLimit.get()))
				 break;
			 
			 lift1.set(speed);
			 lift2.set(speed);
		 }
		 
		 lift1.set(0);
		 lift2.set(0);
	 }
	 
}
