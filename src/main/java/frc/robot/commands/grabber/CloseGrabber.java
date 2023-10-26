package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GrabberSubsystem;

public class CloseGrabber extends InstantCommand {
	GrabberSubsystem grab;
	int counter = 0;

	public CloseGrabber(){
		grab = GrabberSubsystem.getInstance();
		addRequirements(grab);
	}

	@Override
	public void execute(){
		grab.closeGrabber();
		counter++;
	}
}
