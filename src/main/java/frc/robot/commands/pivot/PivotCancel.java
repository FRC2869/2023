package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PivotCancel extends InstantCommand {
    private PivotSubsystem pivot;
    private WristSubsystem wrist;

    public PivotCancel(){
        pivot = PivotSubsystem.getInstance();
        wrist = WristSubsystem.getInstance();
        addRequirements(wrist);
        addRequirements(pivot);
    }

    @Override
    public void execute(){
        pivot.setEnabled(false);
        wrist.setEnabled(false);
        
    }
}
