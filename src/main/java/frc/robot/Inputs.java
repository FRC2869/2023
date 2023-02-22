package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.XboxController;

public class Inputs {
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
    public static double getTranslationX(){
		return 0.0;
		// double speed = -driver.getLeftX();
		// if(Math.abs(speed) < .1){
		// 	speed = 0;
		// }
        // return speed;
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
        return 0.0;
		// double speed = driver.getRightX();
		// if(Math.abs(speed) < .1){
		// 	speed = 0;
		// }
        // return speed;
    }
}
