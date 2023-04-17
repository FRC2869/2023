package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardsAprilTags extends CommandBase {
	private DrivetrainSubsystem swerve;
	private double distance;
	public AutoForwardsAprilTags(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		distance = dist;
		addRequirements(swerve);
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
		double target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[2];
		System.out.println(distance-target);
		if(distance-target > 0.1)
			swerve.driveDirect(RobotContainer.modifyAxis(0.1)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		else if(distance-target < -0.1)
			swerve.driveDirect(RobotContainer.modifyAxis(-0.1)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		else
			swerve.driveDirect(0, 0, 0);
	}	

	@Override
	public boolean isFinished(){
		double target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[2];
		if(Math.abs(distance-target)<.1){
			swerve.driveDirect(0, 0, 0);
			Constants.locked=true;
			return true;
		}
		else
			return false;
	}
}

