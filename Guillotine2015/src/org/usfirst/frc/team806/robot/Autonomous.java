package org.usfirst.frc.team806.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

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
		
	public Autonomous(SpeedController leftR, SpeedController rightR, SpeedController leftF, SpeedController rightF, 
						SpeedController lift1, SpeedController lift2, Encoder leftFEnc, Encoder rightFEnc, Encoder leftREnc, Encoder rightREnc, Drive drive)
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
	}
	
	public void start(int select){
		this.ResetEncoders();
		
		switch (select){
		case 1:  //far right drive around
			StartLiftTote();
			MoveForward(2400,.30);
			LegacyTurn(rightRearEnc, 1500, -.47);
			Wait(806);
			MoveForward(13500, .25);
			LegacyTurn(leftRearEnc, 1600, .47);
			Wait(403);
			MoveForward(7000, .30);
			Lift(1500, -.65);
			
			break;
		case 2:  // drive over ramp
			StartLiftTote();
			MoveForward(9500, .5);
			Lift(1500, -.65);
			break;
		case 3:
			StartLiftTote();
			MoveForward(7500, .5);
			Lift(1500, -.65);
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
		MoveForward(200, .3);
		Lift(1500, .7);
	}
	
	private void Wait(int msTime){
		long time = System.currentTimeMillis();
		
		while (System.currentTimeMillis() < time + msTime){
		}
	}
	
	// slow speed .3, faster speed .35
	private void MoveForward(int distance, double speed){
    	while (rightRearEnc.getRaw() + leftRearEnc.getRaw() < distance){
    		rightFrontDrive.set(-1*speed);
    		leftFrontDrive.set(speed);
    	    leftRearDrive.set(speed);
    		rightRearDrive.set(-1*speed);
    	}
    	this.ResetEncoders();
    	this.ResetDrive();
	}

	 private void Turn(int distance, int direction, double speed){
		 while (rightRearEnc.getRaw() < distance){
    		rightFrontDrive.set(direction*speed);
    		leftFrontDrive.set(direction*speed);
    	    leftRearDrive.set(direction*speed);
    		rightRearDrive.set(direction*speed);
		 }
    	this.ResetEncoders();
    	this.ResetDrive();
	 }
	 
	 private void LegacyTurn(Encoder encoder, int distance, double speed){
		 while (encoder.getRaw() < distance){
			 this.drive.mecanumDrive_Cartesian(0, 0, speed, 0);
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
}
