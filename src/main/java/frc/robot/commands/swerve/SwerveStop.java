package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveStop extends InstantCommand {
    private DrivetrainSubsystem swerve;

    public SwerveStop(){
        swerve = DrivetrainSubsystem.getInstance();
        addRequirements(swerve);
    }
    
    @Override
    public void execute(){
DrivetrainSubsystem.disable();    }
}
