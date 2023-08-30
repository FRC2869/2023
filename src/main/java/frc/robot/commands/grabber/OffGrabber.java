package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class OffGrabber extends InstantCommand {
	GrabberSubsystem grabber;
	public OffGrabber (){
		grabber = GrabberSubsystem.getInstance();
		addRequirements(grabber);
	}

	@Override
	public void execute(){
		System.out.println(Constants.autoTimer.get()+": Off Grabber Start");
		grabber.offGrabber();
	}


}
