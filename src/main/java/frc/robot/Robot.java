// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

// private AddressableLED led;

private AddressableLEDBuffer led_buffer;
// private CameraServer cam;

public static UsbCamera camera;




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // led = new AddressableLED(0);
	// led_buffer = new AddressableLEDBuffer(120);
	// led.setLength(led_buffer.getLength());
    m_robotContainer = new RobotContainer();
	// for(int i=0;i<led_buffer.getLength();i++){
	// 	led_buffer.setRGB(i, 255, 255, 255);
	// }
	// led.setData(led_buffer);
	// led.start();
	camera = CameraServer.startAutomaticCapture("cam0",0);
    camera.setResolution(160, 120);
	camera.setFPS(10);
	
	RobotContainer.auto.add(camera).withPosition(7, 1).withSize(6, 5);
    // RobotContainer.auto.addCamera("Floor Pickup", "cam", camera.getPath()).withPosition(7, 1).withSize(6, 5);
    // RobotContainer.auto.addCamera("Floor Pickup", "cam", camera.getPath()).withPosition(7, 1).withSize(6, 5);
    // RobotContainer.auto.addCamera("LimeLight", "limelight", "10.28.69.87");
	SmartDashboard.putNumber("Charge Station Dist", 2.2);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
	// double id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("id").getDouble(-1);
	// if(id!=-1){
		double target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6])[2];
			SmartDashboard.putNumber("Distance To April Tags", target);
	// }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.\
    CommandScheduler.getInstance().run();
    if(Inputs.getSwerveReset()){
    }

	// rainbow();
	// led.setData(led_buffer);
	// led.start();
	// double centerX;
    // synchronized (imgLock) {
    //     centerX = this.centerX;
    // }
	// SmartDashboard.putNumber("Center X", centerX);

  }
  public void rainbow(){
	int firstPixel = 0;
	for(var i=0; i<led_buffer.getLength();i++){
		final var hue = (firstPixel + (i+180 / led_buffer.getLength())) %180;
		led_buffer.setHSV(i, hue, 255, 128);
		firstPixel += 3;
		firstPixel %= 180;
	}
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Constants.isAuto = false;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Shuffleboard.selectTab("Auto");
    Constants.isAuto = true;
    // Constants.isEnabled = true;
    System.out.println("AUto");
    Constants.autoTimer.reset();
    Constants.autoTimer.start();
	
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
	// new SwerveStop().schedule();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
    }
}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  // System.out.println(CommandScheduler.getInstance().isScheduled(m_autonomousCommand));
  }
  @Override
  public void teleopInit() {
	if(Constants.autoTimer.get()==0){
		Constants.autoTimer.start();
	}
    // Shuffleboard.selectTab("Teleop");
    Constants.isAuto = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
	
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
	SmartDashboard.putNumber("autoTimer",Constants.autoTimer.get());
	Constants.locked = Inputs.getSwerveLock();
	if(Inputs.cancelDriveButton().getAsBoolean()){
	}
  }

  @Override
  public void testInit() {
    Constants.isAuto = false;
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
