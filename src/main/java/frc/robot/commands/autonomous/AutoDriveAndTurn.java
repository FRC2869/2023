package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveAndTurn extends CommandBase {
	double theta;
	double distance;
	DrivetrainSubsystem swerve;
	Timer time;
	private boolean hasRun;
	private double angle;
	private double error;

	/**
	 * 
	 * @param theta field relative angle (0 away form drivers station)
	 * @param dist timer to drive (seconds)
	 */
	public AutoDriveAndTurn(double theta, double dist){
		this.theta = theta;
		this.distance = dist;
		swerve = DrivetrainSubsystem.getInstance();
		time = new Timer();
		time.reset();
		time.start();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		angle = swerve.getGyroYaw().getDegrees();
		error = angle-theta;
		if(!hasRun){
			System.out.println(Constants.autoTimer.get()+": Auto Forwards Start");
			time.reset();
			hasRun = true;
		}
		System.out.println(time.get());
		if(time.get()>distance && Math.abs(error)<5){
			return;
		}
		double drive = RobotContainer.modifyAxis(.45)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
		double turn = Math.copySign(Math.sqrt(Math.sqrt(Math.abs(error/180.0))), error)*.2 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
		// double turn = -.075 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
		if(time.get()>distance)
			drive = 0;
		if(Math.abs(error)<5)
			turn = 0;

		swerve.driveDirect(drive, 0,turn);

	}

	@Override
	public boolean isFinished(){
		if(time.get()>distance+.1 && Math.abs(error)<5){
			System.out.println("Driving Done");
			swerve.driveDirect(0, 0, 0);
			return true;
		}
		return false;
	}
}
