package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveResetGyro extends InstantCommand {
	private SwerveSubsystem swerve;

	public SwerveDriveResetGyro(){
		swerve = SwerveSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void execute(){
		swerve.zeroGyro();
	}
}
