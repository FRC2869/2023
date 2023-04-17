package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRightDist extends CommandBase {
	private DrivetrainSubsystem swerve;
    private Pose2d startPose;
    private double dist;

    /**
     * 
     * @param dist_m meters to move forward. The robot will likely exceed this value before stopping.
     */
	public AutoRightDist(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
        startPose = swerve.getPose();
        this.dist = dist;
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
        double deltaY = swerve.getPose().getY() - startPose.getY();
        if (dist - deltaY < 0.1){
            swerve.drive(0, 0.5, 0);        
        }   
        else
		    swerve.drive(0, 1, 0);
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
		if (swerve.getPose().getY()>=startPose.getY()+dist){
			System.out.println("DONE");
            swerve.drive(0, 0, 0);
			return true;
		}else{
			return false;
		}
	}
	
}
