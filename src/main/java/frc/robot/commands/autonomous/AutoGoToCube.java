package frc.robot.commands.autonomous;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.GripPipeline;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoGoToCube extends CommandBase{
	private DrivetrainSubsystem swerve;
	private GrabberSubsystem grabber;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	private double cubeSize;

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
					cubeSize = r.width*r.height;
					// System.out.println(centerX);
				}
			// }
		}));
		//System.out.println(visionThread);
		visionThread.start();
	}

	/**
	 * Move towards cube or turn towards cube based on OpenCV thread detecting
	 * center of cube in the camera view.
	 */
	@Override
	public void execute(){
		double centerX = 80.0; 
		//default was 0. Changed to 80 so that if we do not detect cube we just drive straight
		double area = 0.0;
		synchronized (imgLock) {
			centerX = this.centerX;
			area = this.cubeSize;
			// System.out.println(centerX);
		}
		//double turn = centerX - (160 / 2)*.1*SwerveConstants.kMaxAngularSpeed;
		double turn = (centerX - (160 / 2))*.15*SwerveConstants.kMaxAngularSpeed;
		//max is 0.2 rotations / sec or aboud 1.256 radians per second.
		// System.out.println(turn);
		double drive = 0.0;
		//drive defaults to zero.
		// if(Math.abs(turn)<1)  //changed from 0.2 to 0.5
			drive = -RobotContainer.modifyAxis(.3)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
		//drive forward only if we are not turning at all.	
		SmartDashboard.putNumber("Cube Size", area);
		SmartDashboard.putNumber("Center X", centerX);
		swerve.driveDirectRobotRelative(drive,0,-turn);
	}
	
	@Override
	public boolean isFinished(){
		//if done, kill the thread.
		return grabber.isIntaked();
	}

	@Override
	public void end(boolean i){
		swerve.driveDirect(0, 0, 0);
		visionThread.interrupt();
	}
}
