package frc.robot.subsystems.swerve.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Motors;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Drive;
import frc.robot.Constants.SwerveConstants.Encoder;

public class SDSMK4i_SwerveModule extends SwerveModule {
    // module data
    private String id;
    private Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private CANSparkMax turnMotor;
    private CANCoderWrapper absoluteEncoder;

    // drive
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    // controller
    private SparkMaxPIDController drivePID;
    private SimpleMotorFeedforward driveFF;
    private SparkMaxPIDController turnPID;
    
    private final SlewRateLimiter turnRateLimit;

    private double prevVelocity;
    
    public SDSMK4i_SwerveModule(String id, Translation2d location, int turnMotorId, Rotation2d angleOffset, int driveMotorId, int encoderId){
        
        //module data
        this.id = id;
        this.location = location;

        //turn
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        absoluteEncoder = new CANCoderWrapper(encoderId);
        configureTurnMotor(angleOffset);

        turnRateLimit = new SlewRateLimiter(SwerveConstants.kMaxAngularSpeed);
        
        // drive
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        configureDriveMotor();
        
        driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

        targetState = new SwerveModuleState();

    }


    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        drivePID.setP(Drive.kP);
        drivePID.setI(Drive.kI);
        drivePID.setD(Drive.kD);
        drivePID.setOutputRange(-1, 1);

        driveMotor.enableVoltageCompensation(12.0);
        
        driveMotor.setInverted(Motors.Swerve.Drive.kInverted);
        driveMotor.setIdleMode(Motors.Swerve.Drive.idlemode);
        driveMotor.setSmartCurrentLimit(Motors.Swerve.Drive.currentLimit);
        driveMotor.setOpenLoopRampRate(Motors.Swerve.Drive.openLoopRampRate);
        driveMotor.burnFlash();
        
        driveEncoder.setPosition(0);
    }


    private void configureTurnMotor(Rotation2d angleOffset) {
        turnMotor.restoreFactoryDefaults();

        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(absoluteEncoder);

        turnPID.setP(SwerveConstants.Turn.kP);
        turnPID.setI(SwerveConstants.Turn.kI);
        turnPID.setD(SwerveConstants.Turn.kD);
        turnPID.setOutputRange(-1, 1);

        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Encoder.Turn.MIN_PID_INPUT);
        turnPID.setPositionPIDWrappingMaxInput(Encoder.Turn.MAX_PID_INPUT);

        turnMotor.enableVoltageCompensation(12.0);

        turnMotor.setInverted(Motors.Swerve.Turn.kInverted);
        turnMotor.setIdleMode(Motors.Swerve.Turn.idlemode);
        turnMotor.setSmartCurrentLimit(Motors.Swerve.Turn.currentLimit);
        turnMotor.setOpenLoopRampRate(Motors.Swerve.Turn.openLoopRampRate);
        turnMotor.burnFlash();
    }


    @Override
    public String getID() {
        return id;
    }

    @Override
    public Translation2d getOffset() {
        return location;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition());
    }


    private double getVelocity() {
        return driveEncoder.getVelocity();
    }


    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    public void setTargetState(SwerveModuleState swerveModuleState) {
        targetState = SwerveModuleState.optimize(swerveModuleState, getAngle());
    }
    
    @Override
    public void periodic() {
        // turn
        turnPID.setReference(
            turnRateLimit.calculate(targetState.angle.getRadians()),
            ControlType.kPosition);

        // drive
        double vel = getVelocity();
        double ffVoltage = driveFF.calculate(prevVelocity, vel, Constants.DT);
        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity, 0, ffVoltage, ArbFFUnits.kVoltage);
        
        prevVelocity = vel;

        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Velocity", vel);
    }
}
