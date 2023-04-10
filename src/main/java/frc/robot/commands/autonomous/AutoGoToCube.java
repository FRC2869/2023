package frc.robot.commands.autonomous;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.GripPipeline;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoGoToCube extends CommandBase{
	private DrivetrainSubsystem swerve;
	private GrabberSubsystem grabber;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();

	public AutoGoToCube(){
		swerve = DrivetrainSubsystem.getInstance();
		grabber = GrabberSubsystem.getInstance();
		addRequirements(swerve);
		
	}
	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
		//start the thread.
		visionThread = new VisionThread(new VisionRunner<GripPipeline>(Robot.camera, new GripPipeline(), pipeline -> {
			// System.out.println(pipeline.hslThresholdOutput());
			// if (!pipeline.hslThresholdOutput()) {
				Rect r = Imgproc.boundingRect(pipeline.hslThresholdOutput());
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
					// System.out.println(centerX);
				}
			// }
		}));
		//System.out.println(visionThread);
		visionThread.start();
	}

	@Override
	public void execute(){
		double centerX = 0.0;
		synchronized (imgLock) {
			centerX = this.centerX;
			// System.out.println(centerX);
		}
		double turn = centerX - (160 / 2);
		System.out.println(turn);
		swerve.driveDirect(-.1*SwerveConstants.kMaxAutoSpeed, -	turn*.2*SwerveConstants.kMaxAngularSpeed,0);
	}
	
	@Override
	public boolean isFinished(){
		//if done, kill the thread.
		return grabber.isIntaked();
	}

	@Override
	public void end(boolean i){

	}
}
