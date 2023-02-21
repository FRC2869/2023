package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.XboxController;

public class Inputs {
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
    public static double getTranslationX(){
		double speed = -driver.getLeftX();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }
    public static double getTranslationY(){
        double speed = driver.getLeftY();
		if(Math.abs(speed) < .1){
			speed = 0;
		}
        return speed;
    }
    public static double getRotation(){
        return driver.getRightX();
    }
}
