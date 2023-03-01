package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Inputs {
	
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
	private static final CommandXboxController driverCmd = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public static double getTranslationX(){
		// return 0.01;
		double speed = -driver.getLeftX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }
    public static double getTranslationY(){
        // return 0.0;
		double speed = driver.getLeftY();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }
    public static double getRotation(){
        // return	 0.0;
		double speed = driver.getRightX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }

	public static Trigger getBalanceButton() {
		return drivercmd.a();
	}

    //Arm
    public static double getExtension(){
        //right trigger goes out, left trigger goes in
        return (driver.getRightTriggerAxis()-driver.getLeftTriggerAxis());
    }

	//Pivot
	public static double getPivotPower() {
		var speed = -driver.getRightY();
		if(Math.abs(speed)<.1){
			speed = 0;
		}
		return speed;
	}
	public static double getPivotPosition() {
		double pos = driver.getRightY(); // [-1, 1]
		pos = pos + 1; // [0, 2]
		pos = pos/2.0; // [0, 1]
		pos = pos * (PivotConstants.kMaxAngle-PivotConstants.kMinAngle); // [0, (kMaxAngle-kMinAngle)]
		pos = pos + PivotConstants.kMinAngle; // [kMinAngle, kMaxAngle]
		return pos;
	}

	/**
	 * 
	 * @return the button that switches to position control on the pivot
	 */
	public static Trigger getPivotPos(){
		return driverCmd.a();
	}

	/**
	 * 
	 * @return the button that switches to position control on the pivot
	 */
	public static Trigger getPivotPwr(){
		return driverCmd.b();
	}

	public static Trigger getArmConeLow(){
		return driverCmd.pov(0);
	}
	public static Trigger getArmConeMid(){
		return driverCmd.pov(90);
	}
	public static Trigger getArmConeHigh(){
		return driverCmd.pov(180);
	}
	public static Trigger getArmCubeLow(){
		return driverCmd.pov(270);
	}
	public static Trigger getArmCubeMid(){
		return driverCmd.x();
	}
	public static Trigger getArmCubeHigh(){
		return driverCmd.y();
	public static Trigger getCloseGrabber(){
		return drivercmd.b();
	}

	public static Trigger getOpenGrabber(){
		return drivercmd.x();
	}
	public static Trigger getOffGrabber(){
		return drivercmd.y();
	}
}
