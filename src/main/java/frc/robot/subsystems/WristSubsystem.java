package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
	private static WristSubsystem instance;
	private WPI_TalonFX wristMotor;
	private double speed = 0;
	private double pos = WristConstants.startingPosition;
	private boolean isPositionControl = false;
	private TalonFXSensorCollection collection;
	private DecimalFormat rounder = new DecimalFormat("#.0");
	// private ArmFeedforward feedForward;

	public static WristSubsystem getInstance(){
		if(instance==null){
			instance = new WristSubsystem();
		}
		return instance;
	}

	public WristSubsystem(){
		wristMotor = new WPI_TalonFX(WristConstants.wristMotorId);
		configureWristMotor();
		new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
	}
	public void resetWrist(){
		collection.setIntegratedSensorPosition(0, 0);
	}

	private void configureWristMotor() {
		// wristMotor.restoreFactoryDefaults();
		
        wristMotor.config_kP(0, WristConstants.kP);
        wristMotor.config_kI(0, WristConstants.kI);
        wristMotor.config_kD(0, WristConstants.kD);
		wristMotor.config_kF(0, WristConstants.kF);
		// wristMotor.setIntegralAccumulator(10);
        wristMotor.config_IntegralZone(0, 2000);
		wristMotor.configClosedLoopPeakOutput(0, WristConstants.kMaxAutoPower);
		wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        wristMotor.configVoltageCompSaturation(12.0);
		wristMotor.setInverted(Motors.Wrist.kInverted);
		wristMotor.setNeutralMode(Motors.Wrist.idlemode);
		wristMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Motors.Wrist.currentLimit, Motors.Wrist.threshholdLimit, 0));
		wristMotor.configOpenloopRamp(Motors.Wrist.openLoopRampRate);
		
		collection = wristMotor.getSensorCollection();
		resetWrist();
		// collection.setIntegratedSensorPosition(0, 0);
		// collection.setIntegratedSensorPosition(WristConstants.startingPosition,0);
		// feedForward = new ArmFeedforward(0.5, .9	, 0);
	}

	/**
	 * Sets the wrist motor to the specified speed
	 * Only applies if isPositionControl == false
	 * @param pwr the speed to set the motor to [-1,1]
	 */
	public void power(double pwr) {
		// speed = MathUtil.clamp(pwr, -WristConstants.kMaxPower, WristConstants.kMaxPower);
		speed = pwr*WristConstants.kMaxPower;
	}

	/**
	 * Sets the wrist motor to the specified position
	 * Only applies if isPositionControl == true
	 * @param pos the position to set the motor to [kMinAngle, kMaxAngle]
	 */
	public void position(double pos){
		this.pos = MathUtil.clamp(pos, WristConstants.kMinAngle, WristConstants.kMaxAngle);
	}

	public double getAngle(){
		// return -collection.getIntegratedSensorPosition();
		return ((collection.getIntegratedSensorPosition()/2048.0)*WristConstants.GEAR_RATIO*360)+WristConstants.startingPosition;
	}

	public double getVelocity(){
		return collection.getIntegratedSensorVelocity();
	}

	/**
	 * Changes whether  the mode is position or speed control
	 * @param positionControl true for position control, false for speed control
	 */
	public void setPositionControl(boolean positionControl){
		isPositionControl = positionControl;
	}

	@Override
	public void periodic(){
		String angleString = rounder.format(getAngle());
		SmartDashboard.putBoolean("Wrist PosControl", isPositionControl);
		SmartDashboard.putString("Wrist Angle",angleString);
		// Supplier<String> angleStringSupp = () -> angleString;
		// RobotContainer.auto.addString("Wrist Angle", angleStringSupp).withPosition(3, 0);
		// System.out.println(getAngle());
		if(true){
			// wristMotor.setReference(pos, ControlType.kPosition, 0, wristFF.calculate(getAngle(), getVelocity()));
			// if(getAngle()<WristConstants.kMinAngle){
				// 	// System.out.println("too low");
				// 	return;
				// }
				// if(getAngle()>WristConstants.kMaxAngle){
					// 	// System.out.println("too high");
					// 	return;
					// }
			SmartDashboard.putNumber("Target Wrist Angle", pos);
			double pos = (this.pos-WristConstants.startingPosition)/360.0/WristConstants.GEAR_RATIO*2048*-1;
					// System.out.println(pos);
			wristMotor.set(TalonFXControlMode.Position, pos);
		}else{
			SmartDashboard.putNumber("Target Wrist Angle", pos);
			if(speed>0 && getAngle()<=WristConstants.kMinAngle){
				// System.out.println("Too Low");
				speed = 0;
			}
			if(speed<0 && getAngle()>=WristConstants.kMaxAngle){
				// System.out.println("Too High");
				speed = 0;
			}
			
			double feedforward = Math.cos(Units.degreesToRadians(getAngle()))*-.075;
			if(getAngle()>210){
				feedforward *= 1.3;
			}
			// wristMotor.set(TalonFXControlMode.PercentOutput, speed-(feedForward.calculate(Units.degreesToRadians(getAngle()), 0)/12.0));
			wristMotor.set(TalonFXControlMode.PercentOutput, speed+feedforward);
		}
	}

	public void toggleCoast() {
		wristMotor.setNeutralMode(NeutralMode.Brake);
	}
}
