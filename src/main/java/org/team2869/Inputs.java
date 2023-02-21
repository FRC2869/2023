package org.team2869;

import org.team2869.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.XboxController;

public class Inputs {
    private static final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);
    public static double getTranslationX(){
        return -driver.getLeftX();
    }
    public static double getTranslationY(){
        return driver.getLeftY();
    }
    public static double getRotation(){
        return driver.getRightX();
    }
}
