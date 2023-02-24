package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveDriveAutoBalance extends CommandBase {
	private SwerveSubsystem swerve;

	public SwerveDriveAutoBalance(){
		swerve = SwerveSubsystem.getInstance();
	}

	@Override
	public void execute(){
		var pitch = swerve.getGyroPitch().getDegrees();
		if(pitch>2.5){
			swerve.drive(.1, 0, 0);
		}else if(pitch<-2.5){
			swerve.drive(-.1, 0, 0);
		}
	}

	@Override
	public boolean isFinished(){
		if(Math.abs(swerve.getGyroPitch().getDegrees())<2.5){
			return true;
		}else{
			return false;
		}
	}
}
