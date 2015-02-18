package org.usfirst.frc.team806.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class SensorController {
	
	public static class SensorType{
		public static final int FrontLeftEncoder = 0;
		public static final int FrontRightEncoder = 1;
		public static final int RearLeftEncoder = 2;
		public static final int RearRightEncoder = 3;
		public static final int UpperLimitSwitch = 4;
		public static final int LowerLimitSwitch = 5;
	}
	
	private Encoder encodFL;
	private Encoder encodFR;
	private Encoder encodRR; 
	private Encoder encodRL;
	
	DigitalInput lowerLimit;
	DigitalInput upperLimit;
	
	public SensorController()
	
	{
	}
	
	public void setEncoder(int encoder, DigitalSource inputA, DigitalSource inputB, boolean reverse, EncodingType encodingType){
		switch(encoder){
		case SensorType.FrontLeftEncoder:
			this.encodFL = new Encoder(inputA, inputB, reverse, encodingType);
			break;
		case SensorType.FrontRightEncoder:
			this.encodFR = new Encoder(inputA, inputB, reverse, encodingType);
			break;
		case SensorType.RearLeftEncoder:
			this.encodRL = new Encoder(inputA, inputB, reverse, encodingType);
			break;
		case SensorType.RearRightEncoder:
			this.encodRR =  new Encoder(inputA, inputB, reverse, encodingType);
			break;
		}
	}
	
	public void setLimitSwitch(int limit, int input){
		switch(limit){
			case SensorType.LowerLimitSwitch:
				 this.lowerLimit = new DigitalInput(input);
				break;
			case SensorType.UpperLimitSwitch:
				this.upperLimit = new DigitalInput(input);
				break;
		}
	}

	
}
