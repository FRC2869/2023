package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardsDist extends CommandBase {
	private DrivetrainSubsystem swerve;
    private double startPose;
    private double dist;
	private double wait;
	private final double speedPercent;
	private final double slowSpeed = .3;
	private double deltaX;

    /**
     * 
     * @param dist_m meters to move forward. The robot will likely exceed this value before stopping.
     */
	public AutoForwardsDist(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
		this.speedPercent = .75;
        this.dist = dist;
	}

	public AutoForwardsDist(double dist, double speedPercent){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
		this.speedPercent = speedPercent;
        this.dist = dist;
	}

	@Override
	public void initialize(){
        startPose = swerve.getDriveEncoder();
		DrivetrainSubsystem.enable();
	}
	@Override
	public void execute(){
		 deltaX = Math.abs(swerve.getDriveEncoder() - startPose);
		// if(wait==15){
		// 	startPose = swerve.getDriveEncoder();
		// 	deltaX = 0;
		// }
		wait++;
        System.out.println(dist-deltaX);
        if (dist - deltaX < 0.75){
            swerve.driveDirect(RobotContainer.modifyAxis(slowSpeed)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
        }   
        else
		    swerve.driveDirect(RobotContainer.modifyAxis(speedPercent)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
	}

    /**
     * 
     * @return true when this command is done and the robot has driven as far forward
     * as requested by the constructor.
     * Note: The distance driven forward will likely exceed the dist requested.
     * 
     */
	@Override
	public boolean isFinished(){
		deltaX = Math.abs(swerve.getDriveEncoder() - startPose);
		if (deltaX>dist){
			System.out.println("DONE");
            swerve.drive(0, 0, 0);
			return true;
		}else{
			return false;
		}
	}
	
}
