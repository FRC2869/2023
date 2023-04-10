package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardsDistRampUp extends CommandBase {
	private DrivetrainSubsystem swerve;
    private int counter = 0;
    private double rampUp;
    private double rampDown;
    private double maxSpeed;
    private double distance; //tick
    private int timeUp;  //tick
    private int timeDown; //tick
    private Pose2d startPose;
    /**
 * Moves the robot (automatically) forwards.
 * @param timeUp Amount of time where the speed increases (ms).
 * @param distance Amount of time where the robot is at max speed (ms).
 * @param timeDown Amount of time where the speed is decreasing (ms).
 * @param maxSpeed The max speed.
 */
	public AutoForwardsDistRampUp(int timeUp, double distance, int timeDown, double maxSpeed){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
        this.maxSpeed = maxSpeed;
        this.timeUp = timeUp / 20;
        this.timeUp = timeUp / 20;
        this.rampUp = this.timeUp / maxSpeed;
        this.timeDown = timeDown / 20;
        this.rampDown = this.timeDown / maxSpeed;
        this.distance = distance;
	}
	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

    /**
     * Will accelerate, stay at max, and then deccelerate.
     */
	@Override
	public void execute(){
        if (timeUp == counter) {
            startPose = swerve.getPose();
        }
        counter++;
        if (counter <= timeUp) 
            swerve.drive(counter * rampUp, 0, 0);
        else if (swerve.getPose().getX()>=startPose.getX() + distance) {
            // swerve.getPose().getX() - startPose.getX();
            swerve.drive(maxSpeed, 0, 0);
        }
        else
            swerve.drive((timeDown - counter) * rampDown, 0, 0);
	}

    /**
     * @return true when this command is done and the robot is finished driving.
     */
	@Override
	public boolean isFinished(){
        System.out.println("DONE");
        if (counter >= timeUp + timeDown) {
            swerve.drive(0, 0, 0);
            return true;
        }
        return false;
	}
	
}
