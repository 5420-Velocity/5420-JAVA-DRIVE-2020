/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.														 */
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PixyAlgo;
import frc.robot.Constants.IntakeConstants;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class Intake extends SubsystemBase {

	private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.IntakeConstants.encoderPort);
	private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.IntakeConstants.armMotor);
	private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.intakeMotor);
	private NetworkTableEntry ntEncoderValue = NetworkTableInstance.getDefault().getEntry("Ball Lift Encoder Value");
	private NetworkTableEntry ntPixyStatus = NetworkTableInstance.getDefault().getEntry("Pixy Status");
	private NetworkTableEntry ntPixyBlocks = NetworkTableInstance.getDefault().getEntry("Pixy Blocks");
	private Pixy2 pixy = Pixy2.createInstance(IntakeConstants.pixyLinkType);
	private PixyAlgo pixyAlgo = new PixyAlgo(pixy);
	private boolean forceUpperlimitDown = false;

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
		this.pixy.init(IntakeConstants.pixyLinkPort);
	}

	public double getEncoderValue() {
		return encoder.get()*100;
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
	 *   the low offset to force the arm down.
	 * 
	 * @return Offset from the Hight Target
	 */
	public double getEncoderFromHighValue() {
		
		// When the Override flag is enabled to force the arm in the Middle position
		if(this.forceUpperlimitDown == true) {
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
	 *  from the list of the blocks in the detected FOV.
	 * 
	 * @return Block that is the closest
	 */
	public Block getPixyLargest() {
		AtomicReference<Block> largestBlock = new AtomicReference<>();
		AtomicInteger largestArea = new AtomicInteger(0);

		/**
		 * AtomicRefrnce and AtomicInteger is used to
		 *  access and store variables between the different loops
		 *  within the forEach Consumer Lambda Function.
		 */
		this.getPixyBlocks().forEach((block) -> {
			int area = block.getWidth() * block.getHeight();

			if(area > largestArea.get()) {
				// Found a larger block!
				largestBlock.set(block);
				largestArea.set(area);
			}
		});

		return largestBlock.get();
	}

	public void armRun(double power) {
		armMotor.set(power);
	}

	public void intakeMove(double power) {
		intakeMotor.set(power);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		/**
		 * Push Encoder Value to the Dashbaord via NetworkTables.
		 */
		this.ntEncoderValue.setDouble(this.getEncoderValue());

		/**
		 * Must be called in order for the subsystem to request
		 *  blocks from the pixy.
		 * Calling this function without the given params will
		 *  return the previous resutls received in this request.
		 * 
		 */
		int pixyStatus = this.pixy.getCCC().getBlocks(false, 0, 8);

		/**
		 * Read response from the Pixy Class to update the network tables
		 *  of the current status.
		 * The getBlocks Function returns either an error code or the total
		 *  amount of blocks detected.
		 */
		switch(pixyStatus) {
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

	}

	/**
	 * This function will set a flag for force the arm target down.
	 * 
	 * @param boolean Force High Offset to the Down Offset
	 */
	public void forceArmDown(boolean force) {
		this.forceUpperlimitDown = force;
	}

}
