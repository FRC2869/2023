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
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class WristSubsystem extends SubsystemBase {
	private static WristSubsystem instance;
	private WPI_TalonFX wristMotor;
	private double pos = WristConstants.startingPosition;
	private TalonFXSensorCollection collection;
	private DecimalFormat rounder = new DecimalFormat("#.0");
	// private ArmFeedforward feedForward;
	private boolean isEnabled = true;
	PositionsWrist currentPos = PositionsWrist.STARTING;
	private boolean isPIDControl;

	public static WristSubsystem getInstance() {
		if (instance == null) {
			instance = new WristSubsystem();
		}
		return instance;
	}

	public WristSubsystem() {
		wristMotor = new WPI_TalonFX(WristConstants.wristMotorId);
		configureWristMotor();
		new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
	}

	public void resetWrist() {
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
		wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		wristMotor.configVoltageCompSaturation(12.0);
		wristMotor.setInverted(Motors.Wrist.kInverted);
		wristMotor.setNeutralMode(Motors.Wrist.idlemode);
		wristMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, Motors.Wrist.currentLimit, Motors.Wrist.threshholdLimit, 0));
		wristMotor.configOpenloopRamp(Motors.Wrist.openLoopRampRate);
		wristMotor.configMotionSCurveStrength(8);

		collection = wristMotor.getSensorCollection();
		resetWrist();

		// collection.setIntegratedSensorPosition(0, 0);
		// collection.setIntegratedSensorPosition(WristConstants.startingPosition,0);
		// feedForward = new ArmFeedforward(0.5, .9 , 0);
	}

	/**
	 * Sets the wrist motor to the specified position
	 * 
	 * @param pos the position to set the motor to [kMinAngle, kMaxAngle]
	 */
	public void position(double pos) {
		// this.pos = MathUtil.clamp(pos, WristConstants.kMinAngle, WristConstants.kMaxAngle);
		this.pos = pos;
	}

	public double getAngle() {
		// return -collection.getIntegratedSensorPosition();
		return ((collection.getIntegratedSensorPosition() / 2048.0) * WristConstants.GEAR_RATIO * 360)
				+ WristConstants.startingPosition;
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

	public void setCurrentPosition(PositionsWrist pos) {
		currentPos = pos;
		
	}

	public void savePositions() {
		switch (currentPos) {
			case BASE:
				WristConstants.basePosition = pos;
				break;
			case DOUBLE_CONE:
				WristConstants.doubleSubstationConeAngle = pos;
				break;
			case DOUBLE_CUBE:
				WristConstants.doubleSubstationCubeAngle = pos;
				break;
			case FLOOR_CONE:
				WristConstants.floorPickupConeAngle = pos;
				break;
			case FLOOR_CUBE:
				WristConstants.floorPickupCubeAngle = pos;
				break;
			case HIGH_CONE:
				WristConstants.highConeAngle = pos;
				break;
			case HIGH_CUBE_BACK:
				WristConstants.highCubeBackAngle = pos;
				break;
			case HIGH_CUBE_FRONT:
				WristConstants.highCubeFrontAngle = pos;
				break;
			case LOW_BACK:
				WristConstants.lowBackAngle = pos;
				break;
			case LOW_FRONT:
				WristConstants.lowFrontAngle = pos;
				break;
			case MID_CONE_BACK:
				WristConstants.midConeBackAngle = pos;
				break;
			case MID_CONE_FRONT:
				WristConstants.midConeFrontAngle = pos;
				break;
			case MID_CUBE_BACK:
				WristConstants.midCubeBackAngle = pos;
				break;
			case MID_CUBE_FRONT:
				WristConstants.midCubeFrontAngle = pos;
				break;
			case SINGLE_CONE:
				WristConstants.singleSubstationConeAngle = pos;
				break;
			case SINGLE_CUBE:
				WristConstants.singleSubstationCubeAngle = pos;
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
		SmartDashboard.putBoolean("Wrist Enabled", isEnabled);
		SmartDashboard.putString("Wrist Angle", angleString);
		// Supplier<String> angleStringSupp = () -> angleString;
		// RobotContainer.auto.addString("Wrist Angle", angleStringSupp).withPosition(3,
		// 0);
		// System.out.println(getAngle());
		if (isEnabled) {
			// wristMotor.setReference(pos, ControlType.kPosition, 0,
			// wristFF.calculate(getAngle(), getVelocity()));
			// if(getAngle()<WristConstants.kMinAngle){
			// // System.out.println("too low");
			// return;
			// }
			// if(getAngle()>WristConstants.kMaxAngle){
			// // System.out.println("too high");
			// return;
			// }
			SmartDashboard.putNumber("Target Wrist Angle", this.pos);
			SmartDashboard.putNumber("wrist difference", Math.abs(getAngle()-this.pos));
			SmartDashboard.putNumber("just the angle", Math.abs(getAngle()));
			SmartDashboard.putBoolean("wrist base?", getAngle()>140);
			double pos = (this.pos - WristConstants.startingPosition) / 360.0 / WristConstants.GEAR_RATIO * 2048
					* -1;
			if (this.pos<150) {
				// System.out.println(pos);
				wristMotor.set(TalonFXControlMode.Position, pos);
				// if(Math.abs(getAngle()-this.pos)<.5){
				// 	isPIDControl = true;
				// }
			} else {
				if(getAngle()<150){
					if(getAngle()<130){
						double basepos = (135 - WristConstants.startingPosition) / 360.0 / WristConstants.GEAR_RATIO * 2048
					* -1;
						// System.out.println("fast");
						wristMotor.set(TalonFXControlMode.Position, basepos);
					}
					else {
						// System.out.println("slow");
						wristMotor.set(TalonFXControlMode.PercentOutput, -.1);
					}
				}
				else {
					// System.out.println("stop");
					wristMotor.set(0);
				}
				// TODO:figure out if any of this is right
				// double theta = Math.toRadians(PivotSubsystem.getInstance().getAngle()); // rad
				
				// double phi = Math.toRadians((180 - getAngle())) - theta; // rad
				// double feedforward = Math.sin(phi);
				// wristMotor.set(TalonFXControlMode.PercentOutput, feedforward * 0.0001);
				// if(Math.abs(getAngle()-this.pos)>.5){
				// 	isPIDControl = true;
				// }
			}
		} else {
			wristMotor.set(0);
		}
	}

	public void brake() {
		wristMotor.setNeutralMode(NeutralMode.Brake);
	}
}
