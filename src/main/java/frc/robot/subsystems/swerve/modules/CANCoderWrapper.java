package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;

public class CANCoderWrapper extends CANCoder implements MotorFeedbackSensor {

    public CANCoderWrapper(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return false;
    }
}
