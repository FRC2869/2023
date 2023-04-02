package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class CloseGrabberFast extends CommandBase {
	GrabberSubsystem grab;
	int counter = 0;

	public CloseGrabberFast(){
		grab = GrabberSubsystem.getInstance();
		addRequirements(grab);
	}

	@Override
	public void execute(){
		grab.closeGrabberFast();
		counter++;
	}

	@Override
	public boolean isFinished(){
		// if(counter>1000){
		// 	return true;
		// }
		return false;
	}
}
