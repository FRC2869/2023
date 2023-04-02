package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveStop extends CommandBase {
    private DrivetrainSubsystem swerve;

    public SwerveStop(){
        swerve = DrivetrainSubsystem.getInstance();
        addRequirements(swerve);
    }
    
    @Override
    public void execute(){
        swerve.drive(0, 0, 0);
    }
}
