package frc.robot;

import java.util.Date;

public class FeedMoment {
	
	public enum MomentType {
		Forward, Reverse, Off
	}

	public final MomentType type;
	private Date deadline;

	public FeedMoment(MomentType type, Date deadline) {
		this.type = type;
		this.deadline = deadline;
	}
	
	public boolean expired() {
		if (new Date().after(this.deadline)) {
			return true;
		}
		return false;
	}

}
