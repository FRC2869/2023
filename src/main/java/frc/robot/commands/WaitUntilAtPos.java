package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WaitUntilAtPos extends CommandBase {
	WristSubsystem wrist;
	PivotSubsystem pivot;

	public WaitUntilAtPos(){
		wrist = WristSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
	}

	@Override
	public void execute(){
		
	}

	@Override
	public boolean isFinished(){
		return wrist.isAtPosition() && pivot.isAtPosition();
	}
}
