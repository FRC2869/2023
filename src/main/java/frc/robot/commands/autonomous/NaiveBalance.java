package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class NaiveBalance extends CommandBase {
	private DrivetrainSubsystem swerve;
	// private PIDController pid;

	private int counter = 0;
	public NaiveBalance(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
		// var kP = SmartDashboard.getNumber("Balance kP", 0.15);
		// var kD = SmartDashboard.getNumber("Balance kD", 0.0);
		// pid = new PIDController(kP, 0, kD);
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
		var pitch = swerve.getAdjustedGyroPitch().getDegrees();
		if(pitch>2.5){
			swerve.driveDirect(.1* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		}else if(pitch <-2.5){
			swerve.driveDirect(-.1* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		}
		else{
			swerve.driveDirect(0, 0, 0);
			// Constants.locked = true;
		}
	}

	// @Override
	public void executeOld() {
		System.out.println("Balancing");
		var pitch = swerve.getAdjustedGyroPitch().getDegrees();
		//hold straight code:
	// double angle = swerve.getGyroscopeRotation().getDegrees();
	// if (angle>360){
	// 	angle -= 360;
	// }
	// if(angle > 5) {
	// 	//rotate a bit.
	// 	swerve.drive(0,0,.1*SwerveConstants.kMaxAngularSpeed);
	// }
	// if (angle < -5) {
	// 	//rotate a bit.
	// 	swerve.drive(0,0,-.1*SwerveConstants.kMaxAngularSpeed);
	// }else 
	if(pitch>5){
		counter++;
		// 	angleDiff = pitch/15.0;
		// 	if(angleDiff>0)
		// 		swerve.drive(() -> .15*Math.sqrt(angleDiff)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, ()->0, () -> 0);
		// 	else {
		// 		swerve.drive(() -> -.15*Math.sqrt(-angleDiff)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, ()->0, () -> 0);
		// 	}
		if(counter>60){
			// swerve.driveDirect(.025* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
			Constants.locked = true;
		}else{
			swerve.driveDirect(.1* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		}
	}else if(pitch<-2.5){
		swerve.driveDirect(-.1* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
	}else if(pitch>2.5 && pitch<5){
		swerve.driveDirect(.05* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
	}
	else{
		swerve.driveDirect(0	, 0, 0);
	}
	}

	@Override
	public boolean isFinished(){
		// System.out.println(swerve.getGyroPitch());
		if(Math.abs(swerve.getAdjustedGyroPitch().getDegrees())<2.5){
			counter++;
		}else{
			counter = 0;
		}

		if (counter > Constants.pidCounter) {
			System.out.println("Balanced");
			Constants.locked = true;
			return true;
		} else {
			return false;
		}
	}
}
