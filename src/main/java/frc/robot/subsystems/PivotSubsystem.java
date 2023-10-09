package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;

public class PivotSubsystem extends SubsystemBase {
	private static PivotSubsystem instance;
	private WPI_TalonFX pivotMotor;
	private double pos = PivotConstants.startingPosition;
	private TalonFXSensorCollection collection;
	private DecimalFormat rounder = new DecimalFormat("#.0");
	private boolean isEnabled = true;
	PositionsPivot currentPos = PositionsPivot.STARTING;
	private boolean isPIDControl;

	public static PivotSubsystem getInstance() {
		if (instance == null) {
			instance = new PivotSubsystem();
		}
		return instance;
	}

	public PivotSubsystem() {
		pivotMotor = new WPI_TalonFX(PivotConstants.pivotMotorId);
		configurePivotMotor();
		new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);
	}

	public void resetPivot() {
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
		pivotMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		pivotMotor.configVoltageCompSaturation(12.0);
		pivotMotor.setInverted(Motors.Pivot.kInverted);
		pivotMotor.setNeutralMode(Motors.Pivot.idlemode);
		pivotMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, Motors.Pivot.currentLimit, Motors.Pivot.threshholdLimit, 0));
		pivotMotor.configOpenloopRamp(Motors.Pivot.openLoopRampRate);
		pivotMotor.configMotionSCurveStrength(8);
		collection = pivotMotor.getSensorCollection();
		resetPivot();
		// collection.setIntegratedSensorPosition(0, 0);
		// collection.setIntegratedSensorPosition(PivotConstants.startingPosition,0);
		// feedForward = new ArmFeedforward(0.5, .9 , 0);
	}

	/**
	 * Sets the pivot motor to the specified position
	 * Only applies if isPositionControl == true
	 * 
	 * @param pos the position to set the motor to [kMinAngle, kMaxAngle]
	 */
	public void position(double pos) {
		// this.pos = MathUtil.clamp(pos, PivotConstants.kMinAngle, PivotConstants.kMaxAngle);
		this.pos = pos;
	}

	public double getAngle() {
		// return -collection.getIntegratedSensorPosition();
		return -1.0 * ((collection.getIntegratedSensorPosition() / 2048.0) * PivotConstants.GEAR_RATIO * 360)
				+ PivotConstants.startingPosition;
	}

	public double getVelocity() {
		return collection.getIntegratedSensorVelocity();
	}

	public void setEnabled(boolean enabled) {
		isEnabled = enabled;
	}

	public void adjustUp() {
		pos += 2;
	}

	public void adjustDown() {
		pos -= 2;
	}

	public void setCurrentPosition(PositionsPivot pos) {
		currentPos = pos;
	}

	public void savePositions() {
		switch (currentPos) {
			case BASE:
				PivotConstants.basePosition = pos;
				break;
			case DOUBLE_CONE:
				PivotConstants.doubleSubstationConeAngle = pos;
				break;
			case DOUBLE_CUBE:
				PivotConstants.doubleSubstationCubeAngle = pos;
				break;
			case FLOOR_CONE:
				PivotConstants.floorPickupConeAngle = pos;
				break;
			case FLOOR_CUBE:
				PivotConstants.floorPickupCubeAngle = pos;
				break;
			case HIGH_CONE:
				PivotConstants.highConeAngle = pos;
				break;
			case HIGH_CUBE_BACK:
				PivotConstants.highCubeBackAngle = pos;
				break;
			case HIGH_CUBE_FRONT:
				PivotConstants.highCubeFrontAngle = pos;
				break;
			case LOW_BACK:
				PivotConstants.lowBackAngle = pos;
				break;
			case LOW_FRONT:
				PivotConstants.lowFrontAngle = pos;
				break;
			case MID_CONE_BACK:
				PivotConstants.midConeBackAngle = pos;
				break;
			case MID_CONE_FRONT:
				PivotConstants.midConeFrontAngle = pos;
				break;
			case MID_CUBE_BACK:
				PivotConstants.midCubeBackAngle = pos;
				break;
			case MID_CUBE_FRONT:
				PivotConstants.midCubeFrontAngle = pos;
				break;
			case SINGLE_CONE:
				PivotConstants.singleSubstationConeAngle = pos;
				break;
			case SINGLE_CUBE:
				PivotConstants.singleSubstationCubeAngle = pos;
				break;
			case STARTING:
				break;
			default:
				break;

		}
	}

	public void setPIDControl(boolean isPIDControl){
		this.isPIDControl = isPIDControl;
	}

	@Override
	public void periodic() {
		String angleString = rounder.format(getAngle());
		SmartDashboard.putBoolean("Pivot Enabled", isEnabled);
		SmartDashboard.putString("Pivot Angle", angleString);
		// Supplier<String> angleStringSupp = () -> angleString;
		// RobotContainer.auto.addString("Pivot Angle", angleStringSupp).withPosition(3,
		// 0);
		// SmartDashboard.putNumber("Pivot Angle", getAngle());
		// System.out.println(getAngle());
		// if(isPositionControl){
		if (isEnabled) {
			// pivotMotor.setReference(pos, ControlType.kPosition, 0,
			// pivotFF.calculate(getAngle(), getVelocity()));
			// if(getAngle()<PivotConstants.kMinAngle){
			// // System.out.println("too low");
			// return;
			// }
			// if(getAngle()>PivotConstants.kMaxAngle){
			// // System.out.println("too high");
			// return;
			// }
			SmartDashboard.putNumber("Target Pivot Angle", this.pos);
			SmartDashboard.putNumber("pivot difference", Math.abs(getAngle()-this.pos));
			SmartDashboard.putBoolean("pivot PID", isPIDControl);
			double pos = (this.pos - PivotConstants.startingPosition) / 360.0 / PivotConstants.GEAR_RATIO * 2048 * -1;

			if (getAngle()>-45 || this.pos>-50) {
				// System.out.println(pos);
				pivotMotor.set(TalonFXControlMode.Position, pos);
				// if(Math.abs(getAngle()-this.pos)<.5){
				// 	isPIDControl = true;
				// }
			} else {
				// TODO:figure out if any of this is right
				// double massArm = 3; //kg 
				// double centerOfMassArm = Units.inchesToMeters(20); //m
				// double lengthArm = Units.inchesToMeters(30); //m
				
				// double massWrist = 5; //kg
				// double centerOfMassWrist = Units.inchesToMeters(10); //m

				// double theta = Math.toRadians(getAngle()); // rad

				// double phi = Math.toRadians((180 - WristSubsystem.getInstance().getAngle())) - theta; // rad
				// double feedforward =  massArm * centerOfMassArm * Math.sin(theta) + (lengthArm * Math.sin(theta) + centerOfMassWrist * Math.cos(phi)) * massWrist;
				// SmartDashboard.putNumber("pivot Feedforward", feedforward*.11);
				// if(getAngle()>-40)
				// 	pivotMotor.set(TalonFXControlMode.PercentOutput, feedforward * .11);
				// if(Math.abs(getAngle()-this.pos)>.5){
				// 	isPIDControl = true;
				// }
				pivotMotor.set(0);
			}
		} else {
			pivotMotor.set(0);
		}
	}

	public void brake() {
		pivotMotor.setNeutralMode(NeutralMode.Brake);
	}
}
