package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CancelArm extends InstantCommand{
	private WristSubsystem wrist;
	private PivotSubsystem pivot;

	public CancelArm(){
		wrist = WristSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		addRequirements(wrist);
		addRequirements(pivot);
	}
	
	@Override
	public void execute(){
		
	}
}
