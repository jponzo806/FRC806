package org.usfirst.frc.team806.robot;

/**
 * 
 * The encoder registry that contains the custom function for retrieving specific encoder data.
 * 
 * @author FRC 806 Brooklyn Blacksmiths
 *
 */

public class EncoderReg extends Robot{

	/**
	 * Encoder registry that is meant to return the RPM for whichever encoder is specified.
	 * 
	 * @param encode The position of the motor corresponding to the encoder to retrieve the RPM for each specific wheel.
	 * @param func An integer that tells the encoder what data to retrieve: RPM, distance, speed, etc.
	 * 
	 * @return The data of the specific encoder that has been called.
	 */
	public static double Call(String encode, int func) {

		if (encode.equals("FL") && func == 1) {

			int FLRaw = encodFL.getRaw() / 50;

			if (FLRaw >= 1) {
				return FLRaw;
			}
		}

		if (encode.equals("FR") && func == 1) {

			int FRRaw = encodFR.getRaw() / 50;

			if (FRRaw >= 1) {
				return FRRaw;
			}
		}
		if (encode.equals("RR") && func == 1) {

			int RRRaw = encodRR.getRaw() / 50;

			if (RRRaw >= 1) {
				return RRRaw;
			}
		}
		if (encode.equals("RL") && func == 1) {

			int RLRaw = encodRL.getRaw() / 50;

			if (RLRaw >= 1) {
				return RLRaw;
			}

		}
		if (encode.equals("FIX") && func == 1) {

			int RLRaw = encodRL.getRaw() / 50;

			return RLRaw;

		} else {
			return 0;
		}

	}
}
