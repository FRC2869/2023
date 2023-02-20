// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants {
    public static final double kMaxSpeed = 2; // 2 meters per second
    public static final double kMaxAngularSpeed = Math.PI * 2; // 2 rotations per second

    public static final double trackWidth_meter = Units.inchesToMeters(15.8);
    public static final double trackLength_meter = Units.inchesToMeters(20.8);

    public interface Drive {
      double kP = 1.0;
      double kI = 0.0;
      double kD = 0.0;

      double kS = 0.0;
      double kV = 0.0;
      double kA = 0.0;
    }
    
    public interface Turn {
      double kP = 1.0;
      double kI = 0.0;
      double kD = 0.0;
    }

    public interface Encoder {
      public interface Turn {

        double MIN_PID_INPUT = 0;
        double MAX_PID_INPUT = 0;

      }
      public interface Drive {

        double WHEEL_DIAMETER = Units.inchesToMeters(4);
        double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        double GEAR_RATIO = 1.0 / 6.75;
        
        double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

      }
    }

    public interface FrontLeft {
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-22));
      public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, trackLength_meter / 2.0);
      public static final String name = "FL";
      public static final int driveMotorId = 0;
      public static final int turnMotorId = 0;
      public static final int encoderId = 13;
    }
    public interface FrontRight {
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-170));
      public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, -trackLength_meter / 2.0);
      public static final String name = "FR";
      public static final int driveMotorId = 0;
      public static final int turnMotorId = 0;
      public static final int encoderId = 10;
    }
    public interface BackLeft {
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(167));
      public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, trackLength_meter / 2.0);
      public static final String name = "BL";
      public static final int driveMotorId = 0;
      public static final int turnMotorId = 0;
      public static final int encoderId = 11;
    }
    public interface BackRight {
      public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(163));
      public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, -trackLength_meter / 2.0);
      public static final String name = "BR";
      public static final int driveMotorId = 0;
      public static final int turnMotorId = 0;
      public static final int encoderId = 12;
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
        boolean kInverted = false;
        IdleMode idlemode = IdleMode.kBrake;
        int currentLimit = 20;
        double openLoopRampRate = 0;
      }
    }
  }

public static final double DT = 0.02; 
}
