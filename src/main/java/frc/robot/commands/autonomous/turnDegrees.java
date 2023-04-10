package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
/**
 * Probably does not work
 */
public class turnDegrees extends CommandBase{
	public double theta;
	public double speed;
	public double distance;
	public int buildUp;
	public DrivetrainSubsystem driveSub = DrivetrainSubsystem.getInstance();

	/**
	 * Turns the angle.  Probably does not work.
	 * @param theta Degree you want to turn.
	 * @param speed Speed you get to theta.
	 * @param dgrsFromTheta Determines how far off of the degrees will be.  If too low the robot might not stop spinning.
	 */
	public turnDegrees(double theta, double speed) {
		this.theta = theta;
		this.speed = speed;
		this.distance = theta / speed;
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute() {
		buildUp++;
		driveSub.drive(0, 0, speed);
	}

	@Override
	public boolean isFinished() {
		if (buildUp >= distance) {
			return true;
		}
		return false;
	}
}