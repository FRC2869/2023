// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2869;

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

	/**
	 * only stores pid constants for drive motors in swerve
	 */
    public enum Drive {
      kP(1.0),
      kI(0.0),
      kD(0.0),
      kS(0.0),
      kV(0.0),
      kA(0.0);

	  private double pidValue;
	  Drive(double v){
		pidValue = v;
	  }
	  public double value(){
		return pidValue;
	  }
    };
    
	//pid constants for turning
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

	public enum ModuleSwerve {
		FrontLeft("FL"),FrontRight("FR"),BackLeft("BL"),BackRight("BR");

		private String name;
		ModuleSwerve(String n){
			name = n;
		}
		public String getValue(){
			return name;
		}
	}

	public static final int FrontLeftDriveMotorID = 0;
	public static final int FrontRightDriveMotorID = 0;
	public static final int BackLeftDriveMotorID = 0;
	public static final int BackRightDriveMotorID = 0;
	public static final int FrontLeftTurnMotorID = 0;
	public static final int FrontRightTurnMotorID = 0;
	public static final int BackLeftTurnMotorID = 0;
	public static final int BackRightTurnMotorID = 0;
	public static final int FrontLeftEncoderID = 13;
	public static final int FrontRightEncoderID = 10;
	public static final int BackLeftEncoderID = 11;
	public static final int BackRightEncoderID = 12;


	/**
	 * 
	 * @param w the module (FrontLeft, FrontRight, etc)
	 * @return the rotation offset discovered experimentally by aligning the wheels.
	 * This value funtions as the "zero" for the wheel - pointing straight ahead.
	 */
	public static Rotation2d getWheelRotation(ModuleSwerve w) {
		switch(w){
			case FrontLeft:
				return new Rotation2d(Units.degreesToRadians(-22));
			case FrontRight:
				return new Rotation2d(Units.degreesToRadians(-170));
			case BackLeft:
				return new Rotation2d(Units.degreesToRadians(167));
			case BackRight:
				return new Rotation2d(Units.degreesToRadians(163));
		}
		return null;
	}
	/**
	 * 
	 * @param w the module (FrontLeft, FrontRight, etc)
	 * @return the location of the module
	 */
	public static Translation2d getModuleLocation(ModuleSwerve w) {
		switch(w){
			case FrontLeft:
				return new Translation2d(trackWidth_meter / 2.0, trackLength_meter / 2.0);
			case FrontRight:
				return new Translation2d(trackWidth_meter / 2.0, -trackLength_meter / 2.0);
			case BackLeft:
				return new Translation2d(-trackWidth_meter / 2.0, trackLength_meter / 2.0);
			case BackRight:
				return new Translation2d(-trackWidth_meter / 2.0, -trackLength_meter / 2.0);
		}
		return null;
	}


    // public interface FrontLeft {
    //   public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-22));
    //   public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, trackLength_meter / 2.0);
    //   public static final String name = "FL";
    //   public static final int driveMotorId = 0;
    //   public static final int turnMotorId = 0;
    //   public static final int encoderId = 13;
    // }
    // public interface FrontRight {
    //   public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(-170));
    //   public static final Translation2d location =  new Translation2d(trackWidth_meter / 2.0, -trackLength_meter / 2.0);
    //   public static final String name = "FR";
    //   public static final int driveMotorId = 0;
    //   public static final int turnMotorId = 0;
    //   public static final int encoderId = 10;
    // }
    // public interface BackLeft {
    //   public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(167));
    //   public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, trackLength_meter / 2.0);
    //   public static final String name = "BL";
    //   public static final int driveMotorId = 0;
    //   public static final int turnMotorId = 0;
    //   public static final int encoderId = 11;
    // }
    // public interface BackRight {
    //   public static final Rotation2d offset = new Rotation2d(Units.degreesToRadians(163));
    //   public static final Translation2d location =  new Translation2d(-trackWidth_meter / 2.0, -trackLength_meter / 2.0);
    //   public static final String name = "BR";
    //   public static final int driveMotorId = 0;
    //   public static final int turnMotorId = 0;
    //   public static final int encoderId = 12;
    // }


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

public static void main(String[] args) {
	double kA = SwerveConstants.Drive.kA.value();
	
}
}

