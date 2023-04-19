package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoChargeStationOnBack extends CommandBase{
	private DrivetrainSubsystem swerve;
	private boolean on;

	public AutoChargeStationOnBack(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
	}
	
	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
		swerve.drive(RobotContainer.modifyAxis(-.7)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,0,0);
	}

	@Override
	public boolean isFinished(){
		if(swerve.getAdjustedGyroPitch().getDegrees()>10){
			on = true;
		}
		if(on && swerve.getAdjustedGyroPitch().getDegrees()<5){
			swerve.drive(()->0,()->0,()->0);
			return true;
		}
		return false;
	}
}
