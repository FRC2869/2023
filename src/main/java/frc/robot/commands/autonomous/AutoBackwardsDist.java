package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBackwardsDist extends CommandBase {
	private DrivetrainSubsystem swerve;
    private Pose2d startPose;
    private double dist;

    /**
     * 
     * @param dist_m meters to move forward. The robot will likely exceed this value before stopping.
     */
	public AutoBackwardsDist(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
        startPose = swerve.getPose();
        this.dist = dist;
	}

	@Override
	public void execute(){
        double deltaX = swerve.getPose().getX() + startPose.getX();
        if (dist - deltaX < 0.1){
            swerve.drive(-0.5, 0, 0);        
        }   
        else
		    swerve.drive(-1, 0, 0);
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
		if (swerve.getPose().getX()>=startPose.getX()+dist){
			System.out.println("DONE");
            swerve.drive(0, 0, 0);
			return true;
		}else{
			return false;
		}
	}
	
}
