package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoToDS extends CommandBase{
	NetworkTableEntry aprilPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace");
	private int timeUp;
	private int timeDown;
	private double maxSpeed;	
	
	public autoToDS(int timeUp, int timeDown, double maxSpeed) {
		this.timeUp = timeUp;
		this.timeDown = timeDown;
		this.maxSpeed = maxSpeed;
	}

	@Override
	public void execute(){
		double[] coolPos = aprilPos.getDoubleArray(new double [6]);
		new AutoSidewaysDistRampUp(timeUp, Math.sqrt(coolPos[0] * coolPos[0] + coolPos[2] * coolPos[2]), timeDown, maxSpeed, coolPos[4]).schedule();
		new turnDegrees(coolPos[4], .5);
	}
	
	@Override
	public boolean isFinished(){
		return true;
	}
}