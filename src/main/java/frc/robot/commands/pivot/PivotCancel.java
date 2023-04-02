package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;

public class PivotCancel extends InstantCommand {
    private PivotSubsystem pivot;

    public PivotCancel(){
        pivot = PivotSubsystem.getInstance();
        addRequirements(pivot);
    }

    @Override
    public void execute(){
        pivot.setPositionControl(false);
        
    }
}
