/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * This class, PixyAlgo, takes in the Pixy
 *  and exposes a few functions that make working with the
 *  pixy better in terms of getting the blocks that you want.
 * 
 * @author Noah Halstead <nhalstead00@gmail.com>
 */
public class PixyAlgo {

	private Pixy2 pixy;

	public PixyAlgo(Pixy2 pixy) {
		this.pixy = pixy;
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
	
	/**
	 * This will return the block that is nearest based on the area and
	 *  distance from 0 (center screen).
	 * 
	 * This function will convert the Pixy Blocks objects X and Y values to
	 *  the offset from CENTER of the camera screen.
	 * 
	 * @link https://stackoverflow.com/a/30449464
	 * @return Block X Offset from the Nearest based on Area (distance away) and
	 *   offset from 0 on the X target.
	 */
	public BlockExtra getPixyBest() {
		TreeSet<Integer> areaRank = new TreeSet<Integer>();
		TreeSet<Integer> zeroRank = new TreeSet<Integer>();
		ArrayList<BlockExtra> blocks = this.convertToBlockExtra(this.getPixyBlocks());

		// Add Blocks to the TreeSets for the Area and the Distance from Zero
		for(BlockExtra block : blocks){
			areaRank.add(block.getArea());
			zeroRank.add(Math.abs(block.getX()));
		}

		// Sort the list by the weight of area and the offset from
		//  zero!
		Collections.sort(blocks, new Comparator<BlockExtra>(){
			/**
			 * Setup sorting to rank the blocks by the following conditions:
			 *  - Block has a Larger Area
			 *  - Block is Close to Zero (Center of the target area)
			 */
			@Override
			public int compare(BlockExtra block1, BlockExtra block2) {
				int block1Area = block1.getArea();
				int block2Area = block2.getArea();

				// Using tailSet to get the rankings from the top of the list, get the
				//  area (gets the positiion with the larger area as a good point)
				// So having 5 as an area will score High in the rank while 1 will be lower!
				int areaRank1 = areaRank.tailSet(block1Area).size();
				int areaRank2 = areaRank.tailSet(block2Area).size();

				// Using headSet to get the rankings from the bottom of the list, get the
				//  zeroOffset (get the position with the larger zero offset as a bad point)
				// So having 1 will score High in the rank while 4 will be lower!
				int zeroOffsetRank1 = zeroRank.headSet(block1.getX()).size();
				int zeroOffsetRank2 = zeroRank.headSet(block2.getX()).size();
				
				// Create the Final Ranking Score given the rankings
				int scoreBlock1 = areaRank1 + zeroOffsetRank1;
				int scoreBlock2 = areaRank2 + zeroOffsetRank2;

				// Decide the position of the block in the list
				if(scoreBlock1 > scoreBlock2) {
					// Block 1 is a better choice, Move it up!
					return 1;
				}
				else if(scoreBlock1 < scoreBlock2) {
					// Block 2 is a better choice, Move Block 1 down!
					return -1;
				}
				else {
					// Blocks are equal, Let it be.
					return 0;
				}
			}
		});

		// Grab the "best" block from the list!
		return blocks.get(0);
	}

	/**
	 * See `convertToBlockExtra` for more details on what this function does.
	 * 
	 * @see convertToBlockExtra
	 * @param blocks
	 * @return Blocks that are extended to BlockExtra
	 */
	private ArrayList<BlockExtra> convertToBlockExtra(ArrayList<Block> blocks) {
		// Convert All Blocks Given to BlockExtra
		ArrayList<BlockExtra> blocksOutput = new ArrayList<BlockExtra>();
		
		for(Block block : blocks) {
			blocksOutput.add(this.convertToBlockExtra(block));
		}

		return blocksOutput;
	}

	/**
	 * Creates a block extra element with the extra function to help.
	 * 
	 * @return Creates a new BlockExtra Element that extends Block
	 */
	private BlockExtra convertToBlockExtra(Block block) {
		BlockExtra newElement = new BlockExtra(
			this.pixy,
			block
		);

		return newElement;
	}

}
