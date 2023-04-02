package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoSidewaysDistRampUp extends CommandBase {
	private DrivetrainSubsystem swerve;
    private int counter = 0;
    private double rampUpX;
    private double rampUpY;
    private double rampDownX;
    private double rampDownY;
    private double maxSpeedX;
    private double maxSpeedY;
    private double distance;
    private int timeUp;  //ticks (20 ms)
    private int timeDown; //ticks (20 ms)
    private Pose2d startPose;
    /**
 * Moves the robot (automatically) forwards.
 * @param timeUp Amount of time where the speed increases (ms).
 * @param distance The distance you want the robot to go.
 * @param timeDown Amount of time where the speed is decreasing (ms).  ABOVE 0 WILL GO OVER THE DISTANCE.
 * @param maxSpeed The max speed.
 * @param theta Rotation that you want to turn. Radians?
 */
	public AutoSidewaysDistRampUp(int timeUp, double distance, int timeDown, double maxSpeed, double theta){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
        this.maxSpeedX = maxSpeed * Math.cos(theta);
        this.maxSpeedY = maxSpeed * Math.sin(theta);
        this.timeUp = timeUp / 20;
        this.distance = distance * Math.cos(theta);
        this.timeDown = timeDown / 20;
        this.rampUpX = this.timeUp / maxSpeedX;
        this.rampUpY = this.timeUp / maxSpeedY;
        this.rampDownX = this.timeDown / maxSpeedX;
        this.rampDownY = this.timeDown / maxSpeedY;
	}

    /**
     * Will accelerate, stay at max, and then deccelerate.
     */
	@Override
	public void execute(){
        if (timeUp == counter) {
            startPose = swerve.getPose();
        }
        if (counter <= timeUp) {
            swerve.drive(counter * rampUpX, counter * rampUpY, 0);
            counter++;
        }
        else if (swerve.getPose().getX()>=startPose.getX() + distance) {
            swerve.drive(maxSpeedX, maxSpeedY, 0);
        }
        else {
            swerve.drive((timeDown - counter) * rampDownX, (timeDown - counter) * rampDownY, 0);
            counter++;
        }
	}

    /**
     * @return true when this command is done and the robot is finished driving.
     */
	@Override
	public boolean isFinished(){
        if (counter >= timeDown) {
            swerve.drive(0, 0, 0);
            return true;
        }
        return false;
	}
}