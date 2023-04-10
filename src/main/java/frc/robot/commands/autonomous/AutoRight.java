package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRight extends CommandBase {
	private DrivetrainSubsystem swerve;
	private Timer time;
	private boolean hasRun = false;
	private double dist;

	public AutoRight(double dist){
		swerve = DrivetrainSubsystem.getInstance();
		// System.out.println("Going To Drive");
		addRequirements(swerve);
		this.dist = dist; 
		time = new Timer();
		time.reset();
		time.start();
	}
	public AutoRight(){
		swerve = DrivetrainSubsystem.getInstance();
		// System.out.println("Going To Drive");
		addRequirements(swerve);
		time = new Timer();
		this.dist = 4.5; 
		time.reset();
		time.start();
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
		if(!hasRun){
			System.out.println(Constants.autoTimer.get()+": Auto Forwards Start");
			time.reset();
			hasRun = true;
		}
		System.out.println(time.get());
		if (time.get()<dist){
		// System.out.println("Driving");
			swerve.driveDirect(0,RobotContainer.modifyAxis(.45)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,0);
		}
	}

	@Override
	public boolean isFinished(){
		// System.out.println(time.get());
		if (time.get()>dist){
			System.out.println(Constants.autoTimer.get()+": Stop Moving");
			// System.out.println("DRIVEN");
			swerve.driveDirect(0, 0, 0);
			return true;
		}
		// }else{
			return false;
		// }
	}
	
}
