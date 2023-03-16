// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BackLeft;
import frc.robot.Constants.SwerveConstants.BackRight;
import frc.robot.Constants.SwerveConstants.FrontLeft;
import frc.robot.Constants.SwerveConstants.FrontRight;

public class DrivetrainSubsystem extends SubsystemBase {
        private static DrivetrainSubsystem instance;

        public static DrivetrainSubsystem getInstance() {
                if (instance == null) {
                        instance = new DrivetrainSubsystem();
                }
                return instance;
        }

        public static void deleteInstance() {
                instance = null;
        }

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         * About 4.3 m/s
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 * 6.75 * .1013 * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        // TODO: Maybe lower constant
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(SwerveConstants.trackLength_meter / 2.0, SwerveConstants.trackWidth_meter / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(SwerveConstants.trackWidth_meter / 2.0,
                                        SwerveConstants.trackLength_meter / 2.0),
                        // Front right
                        new Translation2d(SwerveConstants.trackWidth_meter / 2.0,
                                        -SwerveConstants.trackLength_meter / 2.0),
                        // Back left
                        new Translation2d(-SwerveConstants.trackWidth_meter / 2.0,
                                        SwerveConstants.trackLength_meter / 2.0),
                        // Back right
                        new Translation2d(-SwerveConstants.trackWidth_meter / 2.0,
                                        -SwerveConstants.trackLength_meter / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        private Object m_pose;
        private SwerveDriveOdometry m_odometry;

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // Mk3SwerveModuleHelper.createFalcon500(...)
                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.
                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                // This is the ID of the drive motor
                                FrontLeft.driveMotorId,
                                // This is the ID of the steer motor
                                FrontLeft.turnMotorId,
                                // This is the ID of the steer encoder
                                FrontLeft.encoderId,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FrontLeft.offset.getRadians());

                // We will do the same for the other modules
                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                FrontRight.driveMotorId,
                                FrontRight.turnMotorId,
                                FrontRight.encoderId,
                                FrontRight.offset.getRadians());

                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                BackLeft.driveMotorId,
                                BackLeft.turnMotorId,
                                BackLeft.encoderId,
                                BackLeft.offset.getRadians());

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                BackRight.driveMotorId,
                                BackRight.turnMotorId,
                                BackRight.encoderId,
                                BackRight.offset.getRadians());
                m_odometry = new SwerveDriveOdometry(
                                m_kinematics, m_navx.getRotation2d(),
                                getModulePositions(), new Pose2d(5.0, 13.5, new Rotation2d()));

        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                if (m_navx.isMagnetometerCalibrated()) {
                        // We will only get valid fused headings if the magnetometer is calibrated
                        return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                }
                //
                // // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public Rotation2d getGyroPitch() {
                return Rotation2d.fromDegrees(m_navx.getPitch());
        }

        public void drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
                var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                x.getAsDouble(),
                                y.getAsDouble(),
                                omega.getAsDouble(),
                                getGyroscopeRotation());
                m_chassisSpeeds = chassisSpeeds;
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                                new SwerveModulePosition(m_frontLeftModule.getDriveVelocity(),
                                                Rotation2d.fromDegrees(m_frontLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_frontRightModule.getDriveVelocity(),
                                                Rotation2d.fromDegrees(m_frontRightModule.getSteerAngle())),
                                new SwerveModulePosition(m_backLeftModule.getDriveVelocity(),
                                                Rotation2d.fromDegrees(m_backLeftModule.getSteerAngle())),
                                new SwerveModulePosition(m_backRightModule.getDriveVelocity(),
                                                Rotation2d.fromDegrees(m_backRightModule.getSteerAngle()))
                };
        }

        /**
         * 
         * @return pose in meters
         */
        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void resetOdometery(Pose2d pose) {
                m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
        }

        public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        // Reset odometry for the first path you run during auto
                                        if (isFirstPath) {
                                                resetOdometery(traj.getInitialHolonomicPose());
                                        }
                                }),
                                new PPSwerveControllerCommand(
                                                traj,
                                                () -> getPose(), // Pose supplier
                                                this.m_kinematics, // SwerveDriveKinematics
                                                new PIDController(0, 0, 0), // X controller. Tune these values for your
                                                                            // robot. Leaving them 0 will only use
                                                                            // feedforwards.
                                                new PIDController(0, 0, 0), // Y controller (usually the same values as
                                                                            // X controller)
                                                new PIDController(0, 0, 0), // Rotation controller. Tune these values
                                                                            // for your robot. Leaving them 0 will only
                                                                            // use feedforwards.
                                                this::setModuleStates, // Module states consumer
                                                true, // Should the path be automatically mirrored depending on alliance
                                                      // color. Optional, defaults to true
                                                this // Requires this drive subsystem
                                ));
        }

        public void setModuleStates(SwerveModuleState[] states) {
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                setModuleStates(states);
                m_pose = m_odometry.update(getGyroscopeRotation(), getModulePositions());

        }

}
