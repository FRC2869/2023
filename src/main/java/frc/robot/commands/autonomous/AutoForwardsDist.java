package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardsDist extends CommandBase {
	private DrivetrainSubsystem swerve;
    private double startPose;
    private double dist;

    /**
     * 
     * @param dist_m meters to move forward. The robot will likely exceed this value before stopping.
     */
	public AutoForwardsDist(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
        startPose = swerve.getDriveEncoder();
        this.dist = dist;
	}

	@Override
	public void execute(){
        double deltaX = swerve.getDriveEncoder() - startPose;
        System.out.println(deltaX);
        if (dist - deltaX < 0.1){
            swerve.drive(RobotContainer.modifyAxis(.3)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
        }   
        else
		    swerve.drive(RobotContainer.modifyAxis(.5)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
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
		if (swerve.getDriveEncoder()>=startPose+dist){
			System.out.println("DONE");
            swerve.drive(0, 0, 0);
			return true;
		}else{
			return false;
		}
	}
	
}
