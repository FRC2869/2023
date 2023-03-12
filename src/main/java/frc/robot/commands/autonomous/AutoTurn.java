package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurn extends CommandBase {
	private DrivetrainSubsystem swerve;
	private int counter;

	public AutoTurn(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		swerve.drive(() -> 0,() -> 0,() -> .1*DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND );
		counter++;
	}

	@Override
	public boolean isFinished(){
		if (counter>90){
			swerve.drive(() ->0,() ->0,() ->0);
			return true;
		}else{
			return false;
		}
	}
	
}
