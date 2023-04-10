package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class GoToApril extends CommandBase{
    private Limelight lime;
    private DrivetrainSubsystem swerve;

    public GoToApril(){
        swerve = DrivetrainSubsystem.getInstance();
        lime = Limelight.getInstance();
        addRequirements(swerve);
        addRequirements(lime);
    }
    
    @Override
    public void execute(){
        // System.out.println("START");
        lime.goToTarget();
        // swerve.drive(() ->.1,() -> .1,() -> 0);
    }

    @Override
    public void end(boolean interrupted){
        swerve.drive(() -> 0, () -> 0,() -> 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
