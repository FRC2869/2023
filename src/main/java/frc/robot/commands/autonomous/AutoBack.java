package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoBack extends CommandBase {
	private SwerveSubsystem swerve;
	private int counter;

	public AutoBack(){
		swerve = SwerveSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		swerve.drive(0, .1, 0);
		counter++;
	}

	@Override
	public boolean isFinished(){
		if (counter>200){
			swerve.drive(0,0,0);
			return true;
		}else{
			return false;
		}
	}
	
}
