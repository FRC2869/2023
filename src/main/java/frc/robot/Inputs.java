package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Gathers inputs for the controllers so they can be passed to subsystems
 */
public class Inputs {
	
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
	private static final CommandXboxController driverCmd = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    // private static final XboxController operator = new XboxController(OperatorConstants.kOperatorControllerPort);
	// private static final CommandXboxController operatorCmd = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    // // private static final XboxController operator2 = new XboxController(OperatorConstants.kOperatorControllerPort);
	// private static final CommandXboxController operator2Cmd = new CommandXboxController(OperatorConstants.kOperatorController2Port);
    
	/*
	 * 
	 * Inputs:
	 * Driver
	 * 	Left Joystick - Swerve Drive Translation
	 * 	Right Joystick X - Swerve Drive Rotation
	 * 	Y Button - Robot Relative
	 *  X Button - Lock Wheels
	 * 	A Button - Cancel Any Drivetrain Commands
	 * 	B Button - Auto Balance
	 * 	Start - Reset Gyro
	 * 	Left Bumper - Fast Mode
	 *  Right Bumper - Slow Mode
	 * 
	 * Operator
	 *  Controller 1
	 * 	 Left Joystick Y - Move Arm
	 * 	 D-Pad Down - Base Position
	 * 	 D-Pad Left - Low Position (front)
	 * 	 D-Pad Right - Mid Cube Position (front)
	 *   D-Pad Up - Double Substation Position (front)
	 *   Y Button - Double Substation Position (back)
	 *   X Button - Mid Cone Position (back)
	 *   A Button - Low Position (back)
	 *   B Button - High Cube Position (back)
	 *   Right Bumper - Floor Pickup (Hold Down)
	 * 	 Left Bumper - Cancel Arm Command
	 * 	 Back Button - Manual Encoder Override
	 * 	 Start Button - Switch to Power Control	
	 * 
	 * 	Controller 2
	 *   Y Button - Stop Intake
	 * 	 X Button - Outtake
	 * 	 A Button - Fast Intake
	 * 	 B Button - Slow Intake
	 */


	public static double getTranslationX(){
		// return 0.0;
		double speed = driver.getLeftX();
		if(Math.abs(speed) < .05){
			speed = 0;
		}

		if(driver.getRightBumper()){
			speed *= .5;
		}else if(driver.getLeftBumper()){
			speed *= 1;
		}else{
			speed *= 1;
		}
		SmartDashboard.putNumber("X", speed);
        return speed;
    }
    public static double getTranslationY(){
		// return 0.0;
		double speed = driver.getLeftY();
		if(Math.abs(speed) < .05){
			speed = 0;
		}
		if(driver.getRightBumper()){
			speed *= .5;
		}else if(driver.getLeftBumper()){
			speed *= 1;
		}else{
			speed *= 1;
		}
		SmartDashboard.putNumber("Y", speed);
        return speed;
    }
    public static double getRotation(){
        // return	 0.0;
		double speed = driver.getRightX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
		if(driver.getRightBumper()){
			speed *= .25;
		}else if(driver.getLeftBumper()){
			speed *= .6;
		}else{
			speed *= .65;
		}
        return speed;
    }
	public static Trigger getRobotRelative() {
		return driverCmd.y();
	}
	public static Trigger getBalanceButton() {
		return driverCmd.b();
	}
	public static Trigger cancelDriveButton() {
		return driverCmd.a();
	}
	public static Trigger getResetGyroButton() {
		return driverCmd.start();
	}
	public static boolean getSwerveReset() {
		return driver.getStartButton();
	}
	public static boolean getSwerveLock() {
		return driver.getXButton();
	}

	/*
	 * 
	 * OPERATOR CONTROLS
	 * 
	 */

	 /*
	  * Arm
	  */
	// public static double getPivotPower() {
	// 	var speed = operator.getLeftY();
	// 	if(Math.abs(speed)<.05){
	// 		speed = 0;
	// 	}
	// 	return speed;
	// }
	// public static double getPivotPosition() {
	// 	double pos = -operator.getLeftY(); // [-1, 1]
	// 	pos = pos + 1; // [0, 2]
	// 	pos = pos/2.0; // [0, 1]
	// 	pos = pos * (PivotConstants.kMaxAngle-PivotConstants.kMinAngle); // [0, (kMaxAngle-kMinAngle)]
	// 	pos = pos + PivotConstants.kMinAngle; // [kMinAngle, kMaxAngle]
	// 	return pos;
	// }
	// public static Trigger getArmBase(){
	// 	//Down
	// 	return operatorCmd.pov(180);
	// }
	// public static Trigger getArmLowFront(){
	// 	//Left
	// 	return operatorCmd.pov(90);
	// }
	// public static Trigger getArmCubeMid(){
	// 	//Right
	// 	return operatorCmd.pov(270);
	// }
	// public static Trigger getArmDoubleSubStationFront(){
	// 	//Up
	// 	return operatorCmd.pov(0);
	// }
	// public static Trigger getArmDoubleSubStationBack(){
	// 	//Up
	// 	return operatorCmd.y();
	// }
	// public static Trigger getArmConeMid() {
	// 	//Left
	// 	return operatorCmd.x();
	// }
	// public static Trigger getArmLowBack() {
	// 	//Down
	// 	return operatorCmd.a();
	// }
	// public static Trigger getArmCubeHigh() {
	// 	//Right
	// 	return operatorCmd.b();
	// }
	// public static Trigger getPivotCancelButton(){
	// 	return operatorCmd.leftBumper();
	// }
	// public static Trigger getArmFloorPickup() {
	// 	return operatorCmd.rightBumper();
	// }
	// public static boolean getOverrideButton() {
    //     return operator.getBackButton();
    // }
	// public static Trigger getPivotPowerButton(){
	// 	return operatorCmd.start();
	// }
	// /*
	//  * Grabber
	//  */
	// public static Trigger getCloseGrabber(){
	// 	return operator2Cmd.b();
	// }
	// public static Trigger getOpenGrabber(){
	// 	return operator2Cmd.x();
	// }
	// public static Trigger getOffGrabber(){
	// 	return operator2Cmd.y();
	// }
	// public static Trigger getCloseGrabberFast() {
    //     return operator2Cmd.a();
    // }
	// public static Trigger getAutoTrigger() {
    //     return operator2Cmd.leftBumper();
    // }
}
