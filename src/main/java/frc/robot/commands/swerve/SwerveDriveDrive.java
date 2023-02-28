package frc.robot.commands.swerve;

import frc.robot.Inputs;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase{
    private SwerveSubsystem swerve;
    public SwerveDriveDrive(){
        this.swerve = SwerveSubsystem.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        swerve.drive(Inputs.getTranslationX(), Inputs.getTranslationY(), Inputs.getRotation());
    }
}
