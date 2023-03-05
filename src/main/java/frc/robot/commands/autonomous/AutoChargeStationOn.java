package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoChargeStationOn extends CommandBase{
	private SwerveSubsystem swerve;

	public AutoChargeStationOn(){
		swerve = SwerveSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		swerve.drive(0,-.5,0);
	}

	@Override
	public boolean isFinished(){
		if(swerve.getGyroPitch().getDegrees()>10){
			swerve.drive(0,0,0);
			return true;
		}else{
			return false;
		}
	}
}
