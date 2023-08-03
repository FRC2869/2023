package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PivotPosPwrSwitch extends InstantCommand {
	private PivotSubsystem pivot;
	private boolean pos;
	private WristSubsystem wrist;
	public PivotPosPwrSwitch(boolean position){
		this.pivot = PivotSubsystem.getInstance();
		this.wrist = WristSubsystem.getInstance();
		this.pos = position;
	}

	@Override
	public void execute(){
		pivot.setPositionControl(pos);
		wrist.setPositionControl(pos);
	}
}
