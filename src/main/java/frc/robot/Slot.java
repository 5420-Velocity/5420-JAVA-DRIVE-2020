/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Slot {
	public int position;
	public int startRange, endRange;

	public Slot(int position, int startRange, int endRange) {
		this.position = position;
		this.startRange = startRange;
		this.endRange = endRange;
	}

	public boolean inBetween(double currentPos) {
		return inBetween((int) currentPos);
	}

	public boolean inBetween(int currentPos) {
		return (currentPos < endRange && currentPos > startRange);
	}

}
