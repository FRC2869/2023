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
	public void execute(){
		// System.out.println("Balancing");
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
	if(pitch>2.5){
		// 	angleDiff = pitch/15.0;
		// 	if(angleDiff>0)
		// 		swerve.drive(() -> .15*Math.sqrt(angleDiff)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, ()->0, () -> 0);
		// 	else {
		// 		swerve.drive(() -> -.15*Math.sqrt(-angleDiff)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, ()->0, () -> 0);
		// 	}
		if(pitch<5){
			swerve.driveDirect(.05* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);

		}else{
			swerve.driveDirect(.2* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 0, 0);
		}
		// }else if(pitch<-2.5){
		// 	swerve.drive(() -> RobotContainer.modifyAxis(-.3)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, ()->0, () -> 0);
		}else{
			swerve.driveDirect(0	, 0, 0);
		}
	}

	@Override
	public boolean isFinished(){
		// System.out.println(swerve.getGyroPitch());
		if(Math.abs(swerve.getGyroPitch().getDegrees())<2.5){
			counter++;
		}else{
			counter = 0;
		}

		if (counter > Constants.pidCounter) {
			System.out.println("Balanced");
			return true;
		} else {
			return false;
		}
	}
}
