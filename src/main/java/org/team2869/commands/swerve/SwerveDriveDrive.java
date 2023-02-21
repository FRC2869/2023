package org.team2869.commands.swerve;

import org.team2869.Inputs;
import org.team2869.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
