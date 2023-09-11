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
	 * 	 1 - Storage 
	 * 	  Scoring:
	 *     Front: 
	 *   2 - Low  
	 *   3 - Mid Cube 
	 *   4 - Mid Cone 
	 *   5 - High Cube 
	 * 	   Back:
	 *   6 - Low
	 *   7 - Mid Cone
	 *   8 - Mid Cube
	 *   9 - High Cone
	 *   10 - High Cube
	 *    Pickup:
	 *   11 - Floor Cube
	 *   12 - Single Sub Cone
	 *   13 - Single Sub Cube
	 *   14 - Double Sub Cone
	 *   15 - Double Sub Cube
	 *   16 - Stop Arm
	 *    Intake:
	 *   17 - Intake Cube/Outtake Cone
	 *   18 - Intake Cone Slow/ Outtake Cube
	 *   19 - Intake Cone Fast
	 *   20 - Stop Intake
	 *    Adjustment:
	 *   21 - Wrist Up
	 *   22 - Wrist Down
	 *   23 - Arm Up
	 *   24 - Arm Down
	 *   25 - Save Position
	 *   
	 */


	public static double getTranslationX(){
		// return 0.0;
		double speed = -driver1.getX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}

		if(driver1.getRawButton(5)){
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
		if(driver1.getRawButton(5)){
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
	public static Trigger getArmBase(){
		return operatorCmd.button(1);
	}

	//Scoring
		//Front
			public static Trigger getArmLowFront(){
				return operatorCmd.button(2);
			}
			public static Trigger getArmCubeMidFront(){
				return operatorCmd.button(3);
			}
			public static Trigger getArmConeMidFront() {
				return operatorCmd.button(4);
			}
			public static Trigger getArmCubeHighFront() {
				return operatorCmd.button(5);
			}
		
		//Back
			public static Trigger getArmLowBack() {
				return operatorCmd.button(6);
			}
			public static Trigger getArmConeMidBack() {
				return operatorCmd.button(6);
			}
			public static Trigger getArmCubeMidBack(){
				return operatorCmd.button(3);
			}
			public static Trigger getArmConeHighBack() {
				return operatorCmd.button(4);
			}
			public static Trigger getArmCubeHighBack() {
				return operatorCmd.button(5);
			}

	//Pickup
		public static Trigger getArmFloorPickupCube() {
			return operatorCmd.button(11);
		}
		public static Trigger getArmSingleSubStationCone(){
			return operatorCmd.button(12);
		}
		public static Trigger getArmSingleSubStationCube(){
			return operatorCmd.button(13);
		}
		public static Trigger getArmDoubleSubStationCone(){
			return operatorCmd.button(14);
		}
		public static Trigger getArmDoubleSubStationCube(){
			return operatorCmd.button(15);
		}
	
	
	public static Trigger getPivotCancelButton(){
		return operatorCmd.button(16);
	}
	
	/*
	 * Intake
	 */
	public static Trigger getIntakeFast() {
        return operatorCmd.button(17);
    }
	public static Trigger getIntakeSlow(){
		return operatorCmd.button(18);
	}
	public static Trigger getOuttake(){
		return operatorCmd.button(19);
	}
	public static Trigger getOffGrabber(){
		return operatorCmd.button(20);
	}

	/*
	 * Adjustment
	 */
	public static Trigger getWristAdjustUp(){
		return operatorCmd.button(21);
	}
	public static Trigger getWristAdjustDown(){
		return operatorCmd.button(22);
	}
	public static Trigger getPivotAdjustUp(){
		return operatorCmd.button(23);
	}
	public static Trigger getPivotAdjustDown(){
		return operatorCmd.button(24);
	}
	public static Trigger getSaveAdjustment(){
		return operatorCmd.button(25);
	}
	
}
