package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwards extends CommandBase {
	private DrivetrainSubsystem swerve;
	private int counter;

	public AutoForwards(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		swerve.drive(() -> .25*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,() -> 0,() -> 0);
		counter++;
	}

	@Override
	public boolean isFinished(){
		if (counter>(200/(.25/.1))){
			swerve.drive(() ->0,() ->0,() ->0);
			return true;
		}else{
			return false;
		}
	}
	
}
