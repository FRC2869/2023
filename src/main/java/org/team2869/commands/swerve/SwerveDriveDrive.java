package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Inputs;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveDriveDrive extends CommandBase{
    private SwerveSubsystem swerve;
    public SwerveDriveDrive(){
        this.swerve = SwerveSubsystem.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        swerve.drive(Inputs.getTranslationY(), Inputs.getTranslationX(), Inputs.getRotation());
    }
}
