package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BackLeft;
import frc.robot.Constants.SwerveConstants.BackRight;
import frc.robot.Constants.SwerveConstants.FrontLeft;
import frc.robot.Constants.SwerveConstants.FrontRight;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.swerve.modules.SDSMK4i_SwerveModule;
import frc.robot.subsystems.swerve.modules.SwerveModule;
public class SwerveSubsystem extends SubsystemBase {
    
    private static SwerveSubsystem instance;

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem(
                new SDSMK4i_SwerveModule(FrontLeft.name,FrontLeft.location,FrontLeft.turnMotorId,FrontLeft.offset,FrontLeft.driveMotorId,FrontLeft.encoderId),
                new SDSMK4i_SwerveModule(FrontRight.name,FrontRight.location,FrontRight.turnMotorId,FrontRight.offset,FrontRight.driveMotorId,FrontRight.encoderId),
                new SDSMK4i_SwerveModule(BackLeft.name,BackLeft.location,BackLeft.turnMotorId,BackLeft.offset,BackLeft.driveMotorId,BackLeft.encoderId),
                new SDSMK4i_SwerveModule(BackRight.name,BackRight.location,BackRight.turnMotorId,BackRight.offset,BackRight.driveMotorId,BackRight.encoderId)
            );
        }
        return instance;
    }
    

    private final SwerveModule[] modules;

    public final AHRS gyro;

    private final SwerveDriveKinematics kinematics;

    private final FieldObject2d[] module2ds;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(getModuleOffsets());

        module2ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getID()+"-2d");
        }
    }

    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];    
        
        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getOffset();
        }
        
        return locations;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // private SwerveModule getModule(String id) {
    //     for (SwerveModule module : modules) 
    //         if (module.getID().equals(id)) {
    //             return module;
    //     }
    //     throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    // }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    /** MODULE STATES API **/
    public void drive(double y, double x, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x, -y,
                -omega,
                getGyroAngle());

		ChassisSpeeds speeds2 = new ChassisSpeeds(-x, y, -omega);
		var fieldRelative = false;

		System.out.println(speeds);
		// var swerveModuleStates = kinematics.toSwerveModuleStates(
		// 		fieldRelative
		// 				? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, gyro.getRotation2d())
		// 				: new ChassisSpeeds(x, y, omega));
		// SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeed);

        // Pose2d robotVel = new Pose2d(
        //     Constants.DT * speeds.vxMetersPerSecond,
        //     Constants.DT * speeds.vyMetersPerSecond,
        //     Rotation2d.fromRadians(Constants.DT * speeds.omegaRadiansPerSecond));
        // Twist2d twistVel = new Pose2d().log(robotVel);

        // setChassisSpeeds(new ChassisSpeeds(
        //     twistVel.dx / Constants.DT,
        //     twistVel.dy / Constants.DT,
        //     twistVel.dtheta / Constants.DT
        // ));
		setChassisSpeeds(speeds2);
		// setModuleStates(swerveModuleStates);
    }
    
    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }
    
    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeed);
        
        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }
    
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    /** KINEMATICS **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = odometry.getRotation();
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getOffset().rotateBy(angle)),
                modules[i].getState().angle.plus(angle)
            ));
        }

        // SmartDashboard.putNumber("Swerve/Gyro Angle", getGyroAngle().getDegrees());
        // SmartDashboard.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
        // SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());
    }

}
