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
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
	private static PivotSubsystem instance;
	private WPI_TalonFX pivotMotor;
	private double speed = 0;
	private double pos = 0;
	private boolean isPositionControl = false;
	private TalonFXSensorCollection collection;
	private DecimalFormat rounder = new DecimalFormat("#.0");
	// private ArmFeedforward feedForward;

	public static PivotSubsystem getInstance(){
		if(instance==null){
			instance = new PivotSubsystem();
		}
		return instance;
	}

	public PivotSubsystem(){
		pivotMotor = new WPI_TalonFX(PivotConstants.pivotMotorId);
		configurePivotMotor();
		new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);
	}
	public void resetPivot(){
		collection.setIntegratedSensorPosition(0, 0);
	}

	private void configurePivotMotor() {
		// pivotMotor.restoreFactoryDefaults();
		
        pivotMotor.config_kP(0, PivotConstants.kP);
        pivotMotor.config_kI(0, PivotConstants.kI);
        pivotMotor.config_kD(0, PivotConstants.kD);
		pivotMotor.config_kF(0, PivotConstants.kF);
		// pivotMotor.setIntegralAccumulator(10);
        pivotMotor.config_IntegralZone(0, 2000);
		pivotMotor.configClosedLoopPeakOutput(0, PivotConstants.kMaxAutoPower);
		pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        pivotMotor.configVoltageCompSaturation(12.0);
		pivotMotor.setInverted(Motors.Pivot.kInverted);
		pivotMotor.setNeutralMode(Motors.Pivot.idlemode);
		pivotMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Motors.Pivot.currentLimit, Motors.Pivot.threshholdLimit, 0));
		pivotMotor.configOpenloopRamp(Motors.Pivot.openLoopRampRate);
		
		collection = pivotMotor.getSensorCollection();
		resetPivot();
		// collection.setIntegratedSensorPosition(0, 0);
		// collection.setIntegratedSensorPosition(PivotConstants.startingPosition,0);
		// feedForward = new ArmFeedforward(0.5, .9	, 0);
	}

	/**
	 * Sets the pivot motor to the specified speed
	 * Only applies if isPositionControl == false
	 * @param pwr the speed to set the motor to [-1,1]
	 */
	public void power(double pwr) {
		// speed = MathUtil.clamp(pwr, -PivotConstants.kMaxPower, PivotConstants.kMaxPower);
		speed = pwr*PivotConstants.kMaxPower;
	}

	/**
	 * Sets the pivot motor to the specified position
	 * Only applies if isPositionControl == true
	 * @param pos the position to set the motor to [kMinAngle, kMaxAngle]
	 */
	public void position(double pos){
		this.pos = MathUtil.clamp(pos, PivotConstants.kMinAngle, PivotConstants.kMaxAngle);
	}

	public double getAngle(){
		// return -collection.getIntegratedSensorPosition();
		return -1.0*((collection.getIntegratedSensorPosition()/2048.0)*PivotConstants.GEAR_RATIO*360)+PivotConstants.startingPosition;
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
		
		SmartDashboard.putString("Pivot Angle",angleString);
		// Supplier<String> angleStringSupp = () -> angleString;
		// RobotContainer.auto.addString("Pivot Angle", angleStringSupp).withPosition(3, 0);
		//SmartDashboard.putNumber("Pivot Angle", getAngle());
		// System.out.println(getAngle());
		if(isPositionControl){
			// pivotMotor.setReference(pos, ControlType.kPosition, 0, pivotFF.calculate(getAngle(), getVelocity()));
			// if(getAngle()<PivotConstants.kMinAngle){
			// 	// System.out.println("too low");
			// 	return;
			// }
			// if(getAngle()>PivotConstants.kMaxAngle){
			// 	// System.out.println("too high");
			// 	return;
			// }
			pos = (pos-PivotConstants.startingPosition)/360.0/PivotConstants.GEAR_RATIO*2048*-1;
			// System.out.println(pos);
			pivotMotor.set(TalonFXControlMode.Position, pos);
		}else{
			if(speed>0 && getAngle()<=PivotConstants.kMinAngle){
				// System.out.println("Too Low");
				speed = 0;
			}
			if(speed<0 && getAngle()>=PivotConstants.kMaxAngle){
				// System.out.println("Too High");
				speed = 0;
			}
			
			double feedforward = Math.cos(Units.degreesToRadians(getAngle()))*-.075;
			if(getAngle()>210){
				feedforward *= 1.3;
			}
			// pivotMotor.set(TalonFXControlMode.PercentOutput, speed-(feedForward.calculate(Units.degreesToRadians(getAngle()), 0)/12.0));
			pivotMotor.set(TalonFXControlMode.PercentOutput, speed+feedforward);
		}
	}

	public void toggleCoast() {
		pivotMotor.setNeutralMode(NeutralMode.Brake);
	}
}
