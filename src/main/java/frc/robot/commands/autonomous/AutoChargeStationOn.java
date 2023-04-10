package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoChargeStationOn extends CommandBase{
	private DrivetrainSubsystem swerve;

	public AutoChargeStationOn(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
	}
	
	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
		swerve.drive(RobotContainer.modifyAxis(.5)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,0,0);
	}

	@Override
	public boolean isFinished(){
		System.out.println(swerve.getAdjustedGyroPitch().getDegrees());
		if(swerve.getAdjustedGyroPitch().getDegrees()>10){
			System.out.println("On Charge Station");
			swerve.drive(()->0,()->0,()->0);
			return true;
		}else{
			return false;
		}
	}
}
