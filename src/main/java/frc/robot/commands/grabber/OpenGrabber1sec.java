package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class OpenGrabber1sec extends CommandBase {
	GrabberSubsystem grab;
	int counter = 0;
	Timer time;
	boolean hasRun = false;

	public OpenGrabber1sec(){
		grab = GrabberSubsystem.getInstance();
		addRequirements(grab);
		time = new Timer();
	}

	@Override
	public void execute(){
		if(!hasRun){
			System.out.println(Constants.autoTimer.get()+": Open Grabber Start");
			time.reset();
			time.start();
			hasRun = true;
		}
		grab.openGrabber();
		counter++;
	}

	@Override
	public boolean isFinished(){
		if( time.get()>1 && hasRun){
			System.out.println(Constants.autoTimer.get()+": Open Grabber End");
			return true;
		}else{
			return false;
		}
	}

}
