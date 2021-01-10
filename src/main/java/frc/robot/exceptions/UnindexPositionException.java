/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.exceptions;

/**
 * Add your docs here.
 */
public class UnindexPositionException extends Exception {

	public UnindexPositionException()
	{
		this(-1);
	}

	public UnindexPositionException(int pos)
	{
		this((double) pos);
	}

	public UnindexPositionException(double pos)
	{
		super("Unindexed Position: " + Double.toString(pos));
	}

}
