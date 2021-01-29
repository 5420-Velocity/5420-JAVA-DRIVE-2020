/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.														 */
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.PixyAlgo;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class Intake extends SubsystemBase {

	private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.IntakeConstants.DIO.encoderPort);
	private final WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.IntakeConstants.PWM.armMotor);
	private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.PWM.intakeMotor);
	private final NetworkTableEntry ntEncoderValue = NetworkTableInstance.getDefault().getEntry("Ball Lift Encoder Value");
	private final NetworkTableEntry ntPixyStatus = NetworkTableInstance.getDefault().getEntry("Pixy Status");
	private final NetworkTableEntry ntPixyBlocks = NetworkTableInstance.getDefault().getEntry("Pixy Blocks");
	private final Pixy2 pixy = Pixy2.createInstance(IntakeConstants.pixyLink);
	private final PixyAlgo pixyAlgo = new PixyAlgo(pixy);
	private boolean forceUpperlimitDown = false;
	private int pixyCachedLoop = 0;

	public Intake() {

		/**
		 * Register the default value for the network tables
		 */
		this.ntEncoderValue.setDouble(0.0);
		this.ntPixyStatus.setDefaultString("");

		/**
		 * Setup the encoder setting
		 */
		encoder.setConnectedFrequencyThreshold(975);

		// Tries to Communicate with the Pixy at this moment
		this.pixy.init();
		this.pixy.setLamp((byte) 1, (byte) 1);
		this.pixy.setLED(129, 183, 219);
		// this.pixy.init(IntakeConstants.pixyLinkPort);
	}

	public double getEncoderValue() {
		return encoder.get() * 100;
	}

	/**
	 * Get the offset of the low target and current position
	 *
	 * @return Offset from Low Target
	 */
	public double getEncoderFromLowValue() {
		return this.getEncoderValue() - Constants.IntakeConstants.lowTarget;
	}

	/**
	 * Get the offset of the high target and current position
	 * If the flag `forceUpperlimitDown` is true then it returns
	 * the low offset to force the arm down.
	 *
	 * @return Offset from the Height Target
	 */
	public double getEncoderFromHighValue() {

		// When the Override flag is enabled to force the arm in the Middle position
		if (this.forceUpperlimitDown) {
			return this.getEncoderValue() - Constants.IntakeConstants.middleTarget;
		}

		return this.getEncoderValue() - Constants.IntakeConstants.highTarget;
	}

	/**
	 * Returns all of the Objects within the view of the Pixy2
	 *
	 * @return All Blocks Found in the FOV
	 */
	public ArrayList<Block> getPixyBlocks() {
		return this.pixy.getCCC().getBlockCache();
	}

	/**
	 * This function will get the largest (inherently the closest) object
	 * from the list of the blocks in the detected FOV.
	 *
	 * @return Block that is the closest
	 */
	public Block getPixyLargest() {
		AtomicReference<Block> largestBlock = new AtomicReference<>();
		AtomicInteger largestArea = new AtomicInteger(0);

		/**
		 * AtomicReference and AtomicInteger is used to
		 *  access and store variables between the different loops
		 *  within the forEach Consumer Lambda Function.
		 */
		this.getPixyBlocks().forEach((block) -> {
			int area = block.getWidth() * block.getHeight();

			if (area > largestArea.get()) {
				// Found a larger block!
				largestBlock.set(block);
				largestArea.set(area);
			}
		});

		return largestBlock.get();
	}

	public void armSpeed(double power) {
		armMotor.set(power);
	}

	public void intakeMove(double power) {
		intakeMotor.set(power);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// Push Encoder Value to the Dashboard via NetworkTables.
		this.ntEncoderValue.setDouble(this.getEncoderValue());

//		if (this.pixyCachedLoop == 0) {
			/**
			 * Must be called in order for the subsystem to request
			 *  blocks from the pixy.
			 * Calling this function without the given params will
			 *  return the previous results received in this request.
			 *
			 */
			int pixyStatus = this.pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 8);

			/**
			 * Read response from the Pixy Class to update the network tables
			 *  of the current status.
			 * The getBlocks Function returns either an error code or the total
			 *  amount of blocks detected.
			 */
			switch (pixyStatus) {
				case Pixy2.PIXY_RESULT_BUSY:
					this.ntPixyBlocks.setDouble(0.0);
					this.ntPixyStatus.setString("Busy");
					break;
				case Pixy2.PIXY_RESULT_ERROR:
					this.ntPixyBlocks.setDouble(0.0);
					this.ntPixyStatus.setString("Error");
					break;
				default:
					this.ntPixyBlocks.setDouble(pixyStatus);
					this.ntPixyStatus.setString("Operational");
			}
//		}
//		else if (this.pixyCachedLoop == 120) {
//			// Reset the Counter one 120 has been hit
//			this.pixyCachedLoop = 0;
//		}
//		else {
//			// Update the loop that allowed a skip of updating the Pixy Data
//			this.pixyCachedLoop++;
//		}


	}

	/**
	 * This function will set a flag for force the arm target down.
	 *
	 * @param force Force High Offset to the Down Offset
	 */
	public void forceArmDown(boolean force) {
		this.forceUpperlimitDown = force;
	}

}
