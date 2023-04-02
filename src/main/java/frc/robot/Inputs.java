package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Gathers inputs for the controllers so they can be passed to subsystems
 */
public class Inputs {
	
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
	private static final CommandXboxController driverCmd = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private static final XboxController operator = new XboxController(OperatorConstants.kOperatorControllerPort);
	private static final CommandXboxController operatorCmd = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    
	/*
	 * 
	 * DRIVER CONTROLS
	 * 
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

	public static Trigger getBalanceButton() {
		return driverCmd.b();
	}
	public static Trigger getRobotRelative() {
		return driverCmd.x();
	}
	public static Trigger getResetGyroButton() {
		return driverCmd.start();
	}
	public static boolean getSwerveReset() {
		return driver.getStartButton();
	}

	/*
	 * 
	 * OPERATOR CONTROLS
	 * 
	 */

	public static double getPivotPower() {
		var speed = operator.getLeftY();
		if(Math.abs(speed)<.3){
			speed = 0;
		}
		return speed;
	}
	public static double getPivotPosition() {
		double pos = -operator.getLeftY(); // [-1, 1]
		pos = pos + 1; // [0, 2]
		pos = pos/2.0; // [0, 1]
		pos = pos * (PivotConstants.kMaxAngle-PivotConstants.kMinAngle); // [0, (kMaxAngle-kMinAngle)]
		pos = pos + PivotConstants.kMinAngle; // [kMinAngle, kMaxAngle]
		return pos;
	}
	public static Trigger getArmConeLow(){
		// return operatorCmd.pov(90);
		return null;
	}
	public static Trigger getArmBase(){
		return operatorCmd.pov(180);
	}
	public static Trigger getArmConeMid(){
		return operatorCmd.pov(90);
	}
	public static Trigger getArmCubeMid(){
		return operatorCmd.pov(270);
	}
	public static Trigger getArmDoubleSubStation(){
		return operatorCmd.pov(0);
	}
	public static Trigger getCloseGrabber(){
		return operatorCmd.b();
	}
	public static Trigger getOpenGrabber(){
		return operatorCmd.x();
	}
	public static Trigger getOpenGrabberAlt(){
		return operatorCmd.leftBumper();
	}
	public static Trigger getCloseGrabberAlt(){
		return operatorCmd.rightBumper();
	}
	public static Trigger getOffGrabber(){
		return operatorCmd.y();
	}
    public static boolean getOverrideButton() {
        return operator.getBackButton();
    }
	public static Trigger getPivotPowerButton(){
		return operatorCmd.start();
	}
    public static Trigger getCloseGrabberFast() {
        return operatorCmd.a();
    }
	public static Trigger getPivotCancelButton(){
		return operatorCmd.leftBumper();
	}
	public static Trigger getArmCubeHigh() {
		return operatorCmd.pov(90);
	}
	public static Trigger getArmFloorPickup() {
		return operatorCmd.rightBumper();
	}

}
