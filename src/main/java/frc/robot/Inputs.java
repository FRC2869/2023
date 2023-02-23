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
        return -driver.getLeftX();
    }
    public static double getTranslationY(){
        return driver.getLeftY();
    }
    public static double getRotation(){
        return driver.getRightX();
    }
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
}
