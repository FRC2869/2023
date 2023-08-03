package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

/**
 * Gathers inputs for the controllers so they can be passed to subsystems
 */
public class Inputs {
	
    private static final Joystick driver1 = new Joystick(OperatorConstants.kDriver1ControllerPort);
    // private static final Joystick driver2 = new Joystick(OperatorConstants.kDriver2ControllerPort);
	private static final CommandJoystick driver1Cmd = new CommandJoystick(OperatorConstants.kDriver1ControllerPort);
	private static final CommandJoystick driver2Cmd = new CommandJoystick(OperatorConstants.kDriver2ControllerPort);
	private static final CommandGenericHID operatorCmd = new CommandGenericHID(OperatorConstants.kOperatorControllerPort);
    // private static final XboxController operator2 = new XboxController(OperatorConstants.kOperatorControllerPort);
	// private static final CommandXboxController operator2Cmd = new CommandXboxController(OperatorConstants.kOperatorController2Port);
    
	/*
	 * 
	 * Inputs:
	 * Driver
	 * 	Joystick 1 - Swerve Drive Translation
	 * 	Joystick 2 X - Swerve Drive Rotation
	 * 	1/2 - Robot Relative
	 *  1/3 - Lock Wheels
	 * 	2/1 - Cancel Any Drivetrain Commands
	 * 	2/0 - Auto Balance
	 * 	1/4 - Reset Gyro
	 *  1/1 - Fast Mode
	 *  1/0 - Slow Mode
	 * 
	 * Operator
	 *  Button Board:
	 * 	 Left Joystick Y - Move Arm
	 * 	 14 - Base Position
	 * 	 1 - Low Position (front)
	 * 	 2 - Mid Cube Position (front)
	 *   3 - Double Substation Position (front)
	 *   4 - Double Substation Position (back)
	 *   5 - Mid Cone Position (back)
	 *   6 - Low Position (back)
	 *   7 - High Cube Position (back)
	 *   9 - Floor Pickup (front)
	 * 	 8 - Cancel Arm Command
	 * 
	 * 	
	 *   12 - Stop Intake
	 * 	 11 - Outtake
	 * 	 13 - Fast Intake	
	 * 	 10 - Slow Intake
	 */


	public static double getTranslationX(){
		// return 0.0;
		double speed = -driver1.getX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}

		if(driver1.getRawButton(1)){
			speed *= .5;
		}else if(driver1.getRawButton(1)){
			speed *= 1;
		}else{
			speed *= 1;
		}
		SmartDashboard.putNumber("X", speed);
        return speed;
    }
    public static double getTranslationY(){
		// return 0.0;
		double speed = -driver1.getY();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
		if(driver1.getRawButton(1)){
			speed *= .5;
		}else if(driver1.getRawButton(1)){
			speed *= 1;
		}else{
			speed *= 1;
		}
		SmartDashboard.putNumber("Y", speed);
        return speed;
    }
    public static double getRotation(){
        // return	 0.0;
		double speed = -driver1.getZ();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
		if(driver1.getRawButton(5)){
			speed *= .25;
		}else if(driver1.getRawButton(1)){
			speed *= .6;
		}else{
			speed *= .65;
		}
        return speed;
    }
	public static Trigger getRobotRelative() {
		return driver1Cmd.button(2);
	}
	public static Trigger getBalanceButton() {
		return driver2Cmd.button(3);
	}
	public static Trigger cancelDriveButton() {
		return driver2Cmd.button(1);
	}
	public static Trigger getResetGyroButton() {
		return driver2Cmd.button(2);
	}
	public static boolean getSwerveReset() {
		return driver1.getRawButton(4);
	}
	public static boolean getSwerveLock() {
		return driver1.getRawButton(3);
	}

	/*
	 * 
	 * OPERATOR CONTROLS
	 * 
	 */

	 /*
	  * Arm
	  */
	public static double getPivotPower() {
		var speed = operatorCmd.getRawAxis(0);
		if(Math.abs(speed)<.05){
			speed = 0;
		}
		return speed;
	}
	// public static double getPivotPosition() {
	// 	double pos = -operator.getLeftY(); // [-1, 1]
	// 	pos = pos + 1; // [0, 2]
	// 	pos = pos/2.0; // [0, 1]
	// 	pos = pos * (PivotConstants.kMaxAngle-PivotConstants.kMinAngle); // [0, (kMaxAngle-kMinAngle)]
	// 	pos = pos + PivotConstants.kMinAngle; // [kMinAngle, kMaxAngle]
	// 	return pos;
	// }
	public static Trigger getArmBase(){
		//Down
		return operatorCmd.button(15);
	}
	public static Trigger getArmLowFront(){
		//Left
		return operatorCmd.button(1);
	}
	public static Trigger getArmCubeMidFront(){
		//Right
		return operatorCmd.button(2);
	}
	public static Trigger getArmDoubleSubStationFront(){
		//Up
		return operatorCmd.button(3);
	}
	public static Trigger getArmDoubleSubStationBack(){
		//Up
		return operatorCmd.button(4);
	}
	public static Trigger getArmConeMidFront() {
		//Left
		return operatorCmd.button(5);
	}
	public static Trigger getArmLowBack() {
		//Down
		return operatorCmd.button(6);
	}
	public static Trigger getArmCubeHigh() {
		//Right
		return operatorCmd.button(7);
	}
	public static Trigger getPivotCancelButton(){
		return operatorCmd.button(8);
	}
	public static Trigger getArmFloorPickup() {
		return operatorCmd.button(9);
	}
	/*
	 * Grabber
	 */
	public static Trigger getCloseGrabber(){
		return operatorCmd.button(10);
	}
	public static Trigger getOpenGrabber(){
		return operatorCmd.button(11);
	}
	public static Trigger getOffGrabber(){
		return operatorCmd.button(12);
	}
	public static Trigger getCloseGrabberFast() {
        return operatorCmd.button(13);
    }
	public static Trigger getAutoTrigger() {
        return operatorCmd.button(14);
    }
}
