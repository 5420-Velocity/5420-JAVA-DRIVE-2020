/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.														 */
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;


public class Limelight extends SubsystemBase {

	private double knownArea = 0;
	private double knownDistance = 0;

	/**
	 * Creates a new Limelight.
	 */
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	public Limelight(double knownArea, double knownDistance) {
		
		this.knownArea = knownArea;
		this.knownDistance = knownDistance;

		CameraServer.getInstance().startAutomaticCapture();
	}

	public Limelight() {
		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
     * Return the Distance from the Target
     *  Both Params should be constant, Both Known Values
     *  The KnownArea is based off of the KnownDistance
     * 
     * Place the Robot down on the ground and measure from the limelight
     *  then look at the limelight's web interface for the `ta` value.
     * Pass the 2 values in as a constant and it will return the % of the
     *  selected KnownDistance.
     * 
     * @link http://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area-to-estimate-distance
     * @return The Aprox Distance away from the Target in View with only 2 decimal places
     */
    public double getDistance(){
		// 0.02 is the small threshold of 0
        if(Math.abs(this.getArea()) < 0.02){
            return 0.0;
        }

        double k = this.knownDistance * Math.sqrt(this.knownArea);
        double v = k / Math.sqrt(this.getArea());
        return (double) Math.round(v * 100) / 100;
    }

	public double getTX(){
		return table.getEntry("tx").getDouble(0.0);
	}

	public double getTY(){
		return table.getEntry("ty").getDouble(0.0);
	}

	public double getArea(){
		return table.getEntry("ta").getDouble(0.0);
	}

	public boolean isValidTarget(){
		double check = table.getEntry("tv").getDouble(0.0);

		if(check == 1.0){
			return true;
		} else if(check == 0.0){
			return false;
		} else{
			return false;
		}
	}

	public void setCamMode(double value){
		table.getEntry("camMode").setDouble(value);
	}

	public void setStreamMode(double value){
		table.getEntry("stream").setDouble(value);
	}

	public void setPipeline(double value){
		table.getEntry("pipeline").setDouble(value);
	}	

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
