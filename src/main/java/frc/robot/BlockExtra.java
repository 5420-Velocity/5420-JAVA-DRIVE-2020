/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * Adds in support for extra options related to getting the area
 * of the item or getting the offset related to the center of the screen.
 *
 * @author Noah Halstead <nhalstead00@gmail.com>
 */
public class BlockExtra extends Block {

	private final Pixy2 pixy;

	/**
	 * Helper Function to create the BlockExtra with Block Object
	 *
	 * @param pixy Pixy Instance
	 * @param block Block Instance
	 */
	public BlockExtra(Pixy2 pixy, Block block) {
		this(pixy, block.getSignature(), block.getX(), block.getY(), block.getWidth(), block.getHeight(), block.getAngle(), block.getIndex(), block.getAge());
	}

	public BlockExtra(Pixy2 pixy, int signature, int x, int y, int width, int height, int angle, int index, int age) {
		super(signature, x, y, width, height, angle, index, age);
		this.pixy = pixy;
	}

	/**
	 * Get the Area of the Given Block
	 *
	 * @return The total area of the object
	 */
	public Integer getArea() {
		return this.getWidth() * this.getHeight();
	}

	/**
	 * Get the X Offset from Center of the Frame to the
	 * center of the given block.
	 *
	 * @return X Offset from the Center of the frame
	 * @link https://stackoverflow.com/a/14880815/5779200
	 */
	public Integer getXOffset() {
		double frameWidth = this.pixy.getFrameWidth() / 2;
		double blockX = this.getX();

		return (int) (blockX + frameWidth / 2);
	}

	/**
	 * Get the Y Offset from Center of the Frame to the
	 * center of the given block.
	 *
	 * @return Y Offset from the Center of the frame
	 * @link https://stackoverflow.com/a/14880815/5779200
	 */
	public Integer getYOffset() {
		double frameHeight = this.pixy.getFrameHeight();
		double blockY = this.getY();

		return (int) (frameHeight / 2 + blockY);
	}

}
