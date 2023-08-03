// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Timer autoTimer = new Timer();
  public static boolean isAuto = false;
  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 2;
	public static final int kOperatorController2Port = 2;
    public static final int kDriver1ControllerPort = 0;
    public static final int kDriver2ControllerPort = 1;
  }
  public static class SwerveConstants {
    public static final double kMaxSpeed = 3; // 2 meters per second
    public static final double kMaxAngularSpeed = Math.PI * 2; // 2 rotations per second
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final double trackWidth_meter = Units.inchesToMeters(15.8);
    public static final double trackLength_meter = Units.inchesToMeters(20.8);
    public static final double kMaxAutoSpeed = 1;
    public static final double kMaxAutoAcceration = 0;
	public static final int kSecondSpeed = -2;
	public static final double kFirstSpeed = -15;
    public static final boolean canCoderInvert = false;

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackLength_meter / 2.0, trackWidth_meter / 2.0),
            new Translation2d(trackLength_meter / 2.0, -trackWidth_meter / 2.0),
            new Translation2d(-trackLength_meter / 2.0, trackWidth_meter / 2.0),
            new Translation2d(-trackLength_meter / 2.0, -trackWidth_meter / 2.0));
    public interface Drive {
      double kP = 0.1;
	  double kI = 0.0;
	  double kD = 0.0;

      double kS = 0;
	  double kV = 12.0 / 4.4196;
	  double kA = 0.0;
    double kFF = 0.0;
    }
    
    public interface Turn {
      double kP = .7;
	  double kI = 0.0;
	  double kD = 0.5;
	  double kFF = 0.0;
    }

    public interface Encoder {
      public interface Turn {

        double POSITION_CONVERSION = (12.8 / 1.0);
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

        double MIN_PID_INPUT = -1;
        double MAX_PID_INPUT = 1;

      }
      public interface Drive {

        double WHEEL_DIAMETER = Units.inchesToMeters(4);
        double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        double GEAR_RATIO = 1.0 / 6.75;
        
        double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

      }
    }

    public interface Mod0 {
      //Front Left
	  //MAKE SURE WHEELS ARE SPINNING RIGHT
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-105.73242118206458));
      public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, trackLength_meter / 2.0);
      public static final String name = "FL";
      public static final int driveMotorId = 7;
      public static final int turnMotorId = 8;
      public static final int 	encoderId = 12;
	}
    public interface Mod1 {
      //Front Right
	  //MAKE SURE WHEELS ARE SPINNING RIGHT
      public static final Rotation2d offset = new  Rotation2d(Units.degreesToRadians(-146.1621003889326));
      public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, -trackLength_meter / 2.0);
      public static final String name = "FR";
      public static final int driveMotorId = 6;
      public static final int turnMotorId = 5;
      public static final int encoderId = 11;
    }
    public interface Mod2 {
      //Back Left
	  //MAKE SURE WHEELS ARE SPINNING RIGHT
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-129.99023417033672));
      public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, trackLength_meter / 2.0);
      public static final String name = "BL";
      public static final int driveMotorId = 2;
      public static final int turnMotorId = 9;
      public static final int encoderId = 13;
    }
    public interface Mod3 {
      //Back Right
	  //MAKE SURE WHEELS ARE SPINNING RIGHT
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-59.41406023132423));
    
      public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, -trackLength_meter / 2.0);
      public static final String name = "BR";
      public static final int driveMotorId = 3;
      public static final int turnMotorId = 4;
      public static final int encoderId = 10;
    }


  }

  public static class Motors{
    public interface Swerve{
      public interface Drive{
        boolean kInverted = false;
        IdleMode idlemode = IdleMode.kBrake;
        int currentLimit = 40;
        double openLoopRampRate = 0;
      }
      public interface Turn{
        boolean kInverted = true;
        IdleMode idlemode = IdleMode.kBrake;
        int currentLimit = 20;
        double openLoopRampRate = 0;
      }
    }
    public interface Arm{
      public interface Extension{
        boolean kInverted = true;
        IdleMode idlemode = IdleMode.kBrake;
        int currentLimit = 40;
        double openLoopRampRate = 0;
      }
    }
    public interface Pivot{
      boolean kInverted = false;
      NeutralMode idlemode = NeutralMode.Coast;
      int currentLimit = 40;
      double openLoopRampRate = 0;
    double threshholdLimit = 0;
    }
    public interface Wrist{
      boolean kInverted = true;
      NeutralMode idlemode = NeutralMode.Coast;
      int currentLimit = 20;
      double openLoopRampRate = 1;
    double threshholdLimit = 0;
    }
	public interface Grabber1{
		boolean kInverted = true;
      NeutralMode idlemode = NeutralMode.Brake;
      int currentLimit = 20;
      double openLoopRampRate = 0;
	}
	public interface Grabber2{
		boolean kInverted = false;
      IdleMode idlemode = IdleMode.kBrake;
      int currentLimit = 20;
      double openLoopRampRate = 0;
	}
  }
public static final int pidTimer = 50; 

  public static class PivotConstants{
	
    public static final double basePower = 0.4;
    public static final double doubleSubstationBackAngle = 0; 
    public static final double doubleSubstationFrontAngle = 0;
    public static final double floorPickupBackAngle = 0;
    public static final double highConeAngle = 0;
    public static final double highCubeBackAngle = 0;
    public static final double lowBackAngle = 0;
    public static final double lowCubeAngle = 0;
    public static final double lowFrontAngle = -40;
    public static final double midConeBackAngle = 0;
    public static final double midCubeBackAngle = 0;
    public static final double tolerance = 1;
    public static final double basePosition = -40;
    public static final double floorPickupFrontAngle = -40;
    public static final double GEAR_RATIO = (1/25.0)*(12.0/54.0);
    public static final double highCubeFrontAngle = 0;
    public static final double kP = .05;
    public static final double kI = 0.0001;
    public static final double kD = 8;
    public static final double kF = 0;
    public static final double kS = 1.0;
    public static final double kV = 0;
    public static final double kG = 0.9;
    public static final double kMaxAngle = 240;
    public static final double kMaxAutoPower = .7;
    public static final double kMaxPower = .35;
    public static final double kMinAngle = -45;
    public static final double midConeFrontAngle = 24;
    public static final double midCubeFrontAngle = -18;
    public static final double singleSubstationFrontAngle = 0;
    public static final double startingPosition = -52;
    public static final double VELOCITY_CONVERSION = 1/60.0;
    public static final int pivotMotorId = 14;
  }
  public static class PneumaticsConstants{
	public static final int solenoidPortForwardsTwo = 1;
	public static final int solenoidPortBackwardsTwo = 0;
	public static int solenoidPortForwards = 3;
	public static int solenoidPortBackwards = 2;
  }

  public static class WristConstants {
    public static final int wristMotorId = 17;
    public static final double kP = .3;
    public static final double kI = 0.0;
    public static final double kD = 20;
    public static final double kF = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kG = 0.5;
    public static final double GEAR_RATIO = 1.0/25.0;
    public static final double tolerance = 1;
    public static final double kMaxAutoPower = .30;

    public static final double startingPosition = 156;

    public static final double basePosition = 150;

    public static final double doubleSubstationBackAngle = 0;
    public static final double doubleSubstationFrontAngle = 0;

    public static final double floorPickupBackAngle = 0;
    public static final double floorPickupFrontAngle = 40;

    public static final double highConeAngle = 0;

    public static final double highCubeBackAngle = 0;
    public static final double highCubeFrontAngle = 0;

    public static final double lowBackAngle = 0;
    public static final double lowFrontAngle = 70;

    public static final double midConeBackAngle = 0;
    public static final double midConeFrontAngle = -20;

    public static final double midCubeBackAngle = 0;
    public static final double midCubeFrontAngle = 70;

    public static final double singleSubstationFrontAngle = 0;
	public static final double VELOCITY_CONVERSION = 1/60.0;
	public static final double kMinAngle = -140;
	public static final double kMaxAngle = 145;
	public static final double kMaxPower = 0.30;

  }
public static final double DT = 0.02;
public static final int pidCounter = 100;
public static boolean locked = false; 
}

