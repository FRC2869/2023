package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Inputs {
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
    private static final CommandXboxController drivercmd = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public static double getTranslationX(){
		// return 0.01;
		double speed = -driver.getLeftX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }
    public static double getTranslationY(){
        return 0.0;
		// double speed = driver.getLeftY();
		// if(Math.abs(speed) < .1){
		// 	speed = 0;
		// }
        // return speed;
    }
    public static double getRotation(){
        return	 0.0;
		// double speed = driver.getRightX();
		// if(Math.abs(speed) < .1){
		// 	speed = 0;
		// }
        // return speed;
    }

	public static Trigger getBalanceButton() {
		return drivercmd.a();
	}

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
