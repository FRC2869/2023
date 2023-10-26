// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;
import frc.robot.commands.ArmBasePos;
import frc.robot.commands.ArmConeMidBack;
import frc.robot.commands.ArmConeMidFront;
import frc.robot.commands.ArmCubeMidFront;
import frc.robot.commands.ArmFloorPickupCube;
import frc.robot.commands.ArmMove;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PivotAdjustDown;
import frc.robot.commands.PivotAdjustUp;
import frc.robot.commands.WaitUntilAtPos;
import frc.robot.commands.WristAdjustDown;
import frc.robot.commands.WristAdjustUp;
import frc.robot.commands.grabber.CloseGrabber;
import frc.robot.commands.grabber.CloseGrabberFast;
import frc.robot.commands.grabber.OffGrabber;
import frc.robot.commands.grabber.OpenGrabber;
import frc.robot.commands.pivot.PivotCancel;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
	// private DrivetrainSubsystem swerve = DrivetrainSubsystem.getInstance();
	// private final ArmSubsystem arm = ArmSubsystem.getInstance();
	// private final PivotSubsystem pivot = PivotSubsystem.getInstance();
	// private final GrabberSubsystem grabber = GrabberSubsystem.getInstance();
	public static ShuffleboardTab auto = Shuffleboard.getTab("Auto");
	
	ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
	ShuffleboardTab test = Shuffleboard.getTab("test");

	private enum Autos {
		Nothing, Forward, ScoreMidCone, 
		REDClearCone, REDClearCube, 
		BLUEClearCone, BLUEClearCube, 
		REDBumpCone, REDBumpCube, 
		BLUEBumpCone, BLUEBumpCube, 
		REDClearMove, BLUEClearMove, 
		REDBumpMove, BLUEBumpMove
	}
	Autos currentAuto = Autos.Nothing;
	private SendableChooser<Autos> newautopick; 

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		
		newautopick = new SendableChooser<>();
		newautopick.addOption("Nothing", Autos.Nothing);
		newautopick.addOption("Forward", Autos.Forward);
		newautopick.setDefaultOption("ScoreMidCone", Autos.ScoreMidCone);
		newautopick.addOption("RedClearCone", Autos.REDClearCone);
		newautopick.addOption("RedClearCube", Autos.REDClearCube);
		newautopick.addOption("BlueClearCone", Autos.BLUEClearCone);
		newautopick.addOption("BlueClearCube", Autos.BLUEClearCube);
		newautopick.addOption("RedBumpCone", Autos.REDBumpCone);
		newautopick.addOption("RedBumpCube", Autos.REDBumpCube);
		newautopick.addOption("BlueBumpCone", Autos.BLUEBumpCone);
		newautopick.addOption("BlueBumpCube", Autos.BLUEBumpCube);
		newautopick.addOption("RedClearMove", Autos.REDClearMove);
		newautopick.addOption("RedBumpMove", Autos.REDBumpMove);
		newautopick.addOption("BlueClearMove", Autos.BLUEClearMove);
		newautopick.addOption("BlueBumpMove", Autos.BLUEBumpMove);
		
		auto.add("auto", newautopick).withPosition(0, 0).withSize(3, 1);
		SmartDashboard.putNumber("Balance kP", 0.25);
		SmartDashboard.putNumber("Balance kD", 0.1);

		CommandScheduler.getInstance().cancelAll();
		configureDefaultCommands();
		configureBindings();
		resetSwerve();
		// grabber.compressorOn();
	}

	/****************/
	/*** DEFAULTS ***/
	/****************/

	private void configureDefaultCommands() {
		// m_SwerveSubsystem.setDefaultCommand(new SwerveDriveDrive());
		swerve.setDefaultCommand(new DefaultDriveCommand(
            () -> Inputs.getTranslationY(),
            () -> Inputs.getTranslationX(),
            () -> Inputs.getRotation(),
			() -> (true), false, false
    ));
		// pivot.setDefaultCommand(new PivotDefault());
		// arm.setDefaultCommand(new ArmDefault());
		// grabber.setDefaultCommand(new OffGrabber());
	}

	public void resetSwerve(){
		swerve.zeroGyro();

	}

	/***************/
	/*** BUTTONS ***/
	/***************/
	private void configureBindings() {
		configureDriverBindings();
		configureOperatorBindings();
	}

	/**
	 * 
	 */
	private void configureDriverBindings() {
		
		Inputs.getBalanceButton().onTrue(getAutonomousCommand());
		// Inputs.getSlowMode().whileTrue(new DefaultDriveCommand(()->Inputs.getTranslationY(), ()->Inputs.getTranslationX(), ()->Inputs.getRotation(), ()-> true, true, false));
	}
	private void configureOperatorBindings(){
		Inputs.getArmBase().onTrue(new ArmMove(PositionsPivot.BASE, PositionsWrist.BASE, 0,0));

		Inputs.getArmLowFront().onTrue(new ArmMove(PositionsPivot.LOW_FRONT, PositionsWrist.LOW_FRONT, 0,0));
		Inputs.getArmCubeMidFront().onTrue(new ArmMove(PositionsPivot.MID_CUBE_FRONT, PositionsWrist.MID_CUBE_FRONT, 0,1));
		Inputs.getArmConeMidFront().onTrue(new ArmMove(PositionsPivot.MID_CONE_FRONT, PositionsWrist.MID_CONE_FRONT, 0,1));
		Inputs.getArmCubeHighFront().onTrue(new ArmMove(PositionsPivot.HIGH_CUBE_FRONT, PositionsWrist.HIGH_CUBE_FRONT, 0,1));
		
		Inputs.getArmLowBack().onTrue(new ArmMove(PositionsPivot.LOW_BACK, PositionsWrist.LOW_BACK, 0,1));
		Inputs.getArmCubeMidBack().onTrue(new ArmMove(PositionsPivot.MID_CUBE_BACK, PositionsWrist.MID_CUBE_BACK, 0,1));
		Inputs.getArmConeMidBack().onTrue(new ArmMove(PositionsPivot.MID_CONE_BACK, PositionsWrist.MID_CONE_BACK, 0,1));
		Inputs.getArmConeHighBack().onTrue(new ArmMove(PositionsPivot.HIGH_CONE, PositionsWrist.HIGH_CONE, 0,0));
		Inputs.getArmCubeHighBack().onTrue(new ArmMove(PositionsPivot.HIGH_CUBE_BACK, PositionsWrist.HIGH_CUBE_BACK, 0,1));

		Inputs.getArmFloorPickupCube().onTrue(new ArmMove(PositionsPivot.FLOOR_CUBE, PositionsWrist.FLOOR_CUBE, 0,0));
		Inputs.getArmSingleSubStationCone().onTrue(new ArmMove(PositionsPivot.SINGLE_CONE, PositionsWrist.SINGLE_CONE, 0,0));
		Inputs.getArmSingleSubStationCube().onTrue(new ArmMove(PositionsPivot.SINGLE_CUBE, PositionsWrist.SINGLE_CUBE, 0,1));
		Inputs.getArmDoubleSubStationCone().onTrue(new ArmMove(PositionsPivot.DOUBLE_CONE, PositionsWrist.DOUBLE_CONE, 0,1));
		Inputs.getArmDoubleSubStationCube().onTrue(new ArmMove(PositionsPivot.DOUBLE_CUBE, PositionsWrist.DOUBLE_CUBE, 0,1));
		
		Inputs.getPivotCancelButton().onTrue(new PivotCancel());
		
		Inputs.getIntakeSlow().whileTrue(new CloseGrabber());
		Inputs.getIntakeFast().whileTrue(new CloseGrabberFast());
		Inputs.getOuttake().whileTrue(new OpenGrabber());
		Inputs.getOffGrabber().onTrue(new OffGrabber());
		
		Inputs.getWristAdjustUp().onTrue(new WristAdjustUp());
		Inputs.getWristAdjustDown().onTrue(new WristAdjustDown());
		Inputs.getPivotAdjustUp().onTrue(new PivotAdjustUp());
		Inputs.getPivotAdjustDown().onTrue(new PivotAdjustDown());
		// Inputs.getSaveAdjustment().onTrue(new ConstantsSave());
		// Inputs.getAutoTrigger().onTrue(new AutoGoToCube());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		switch(newautopick.getSelected()){
			case REDClearCube:
				HashMap<String, Command> eventMap1 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredtop1", new PathConstraints(1, 3), eventMap1, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredtop2", new PathConstraints(1, 3), eventMap1, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case REDClearCone:
				HashMap<String, Command> eventMap2 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredtop1", new PathConstraints(1, 3), eventMap2, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), true),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos()
					// SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredtop2", new PathConstraints(1, 3), eventMap2, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), true),
					// new ArmCubeMidFront(),
					// new WaitUntilAtPos(),
					// new CloseGrabberFast()
				);
			case BLUEClearCube:
				HashMap<String, Command> eventMap3 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluetop1", new PathConstraints(2, 1), eventMap3, new PIDConstants(0, 0.0, 0.0), new PIDConstants(0, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluetop2", new PathConstraints(2, 1), eventMap3, new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case BLUEClearCone:
				HashMap<String, Command> eventMap4 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidBack(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluetop1", new PathConstraints(1, 3), eventMap4, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), true),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos()
					// SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluetop2", new PathConstraints(1, 3), eventMap4, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), true),
					// new ArmCubeMidFront(),
					// new WaitUntilAtPos(),
					// new CloseGrabberFast()
				);
			case BLUEBumpCube:
				HashMap<String, Command> eventMap5 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluebottom1", new PathConstraints(1, 3), eventMap5, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluebottom2", new PathConstraints(1, 3), eventMap5, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case BLUEBumpCone:
				HashMap<String, Command> eventMap6 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluebottom1", new PathConstraints(1, 3), eventMap6, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathbluebottom2", new PathConstraints(1, 3), eventMap6, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case REDBumpCube:
				HashMap<String, Command> eventMap7 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredbottom1", new PathConstraints(1, 3), eventMap7, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredbottom2", new PathConstraints(1, 3), eventMap7, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case REDBumpCone:
				HashMap<String, Command> eventMap8 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new OpenGrabber(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredbottom1", new PathConstraints(1, 3), eventMap8, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false),
					new ArmFloorPickupCube(),
					new CloseGrabber(),
					new WaitCommand(1),
					new OffGrabber(),
					new ArmBasePos(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("Pathredbottom2", new PathConstraints(1, 3), eventMap8, new PIDConstants(5, 0, 0), new PIDConstants(.5, 0, 0), false),
					new ArmCubeMidFront(),
					new WaitUntilAtPos(),
					new CloseGrabberFast()
				);
			case REDBumpMove:
				HashMap<String, Command> eventMap9 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("bottomRedMove", new PathConstraints(1, 3), eventMap9, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false)
				);
			case REDClearMove:
				HashMap<String, Command> eventMap10 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("topRedMove", new PathConstraints(1, 3), eventMap10, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false)
				);
			case BLUEBumpMove:
				HashMap<String, Command> eventMap11 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("bottomBlueMove", new PathConstraints(1, 3), eventMap11, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false)
				);
			case BLUEClearMove:
				HashMap<String, Command> eventMap12 = new HashMap<>();
				return new SequentialCommandGroup(
					new ArmConeMidFront(),
					new WaitUntilAtPos(),
					new WaitCommand(1),
					new ArmBasePos(),
					// new WaitUntilAtPos(),
					new OffGrabber(),
					SwerveSubsystem.getInstance().createPathPlannerCommand("topBlueMove", new PathConstraints(1, 3), eventMap12, new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0), false)
				);
			default:
				return new WaitCommand(500000);
				
					// return new WaitCommand(5000);			
		}
		// return null;
		// An example command will be run in autonomous
		// return swerve.followTrajectoryCommand(floorPickup, true);
	   	// return swerve.followTrajectoryCommand(forward1m, true);
		// return new SequentialCommandGroup(new AutoForwards(), new AutoBack(), new AutoRight(), new AutoChargeStationOn(), new SwerveDriveAutoBalance());
		// return new SequentialCommandGroup(new AutoChargeStationOn(), new SwerveDriveAutoBalance());
		// return new SequentialCommandGroup(new ArmConeMid(), new ParallelRaceGroup(new OpenGrabber(), new WaitCommand(1)), new ParallelRaceGroup(new OffGrabber(), new WaitCommand(1)),  new ArmBasePos(), new AutoChargeStationOn(), new SwerveDriveAutoBalance(), new WaitCommand(500));
		// return new SequentialCommandGroup(new ArmConeMid(), new ParallelRaceGroup(new OpenGrabber(), new WaitCommand(1)), new ParallelRaceGroup(new OffGrabber(), new WaitCommand(1)),  new ArmBasePos(), swerve.followTrajectoryCommand(floorPickup, true), new WaitCommand(500));
		// return new AutoForwardsDist(1);
	}
	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
		  if (value > 0.0) {
			return (value - deadband) / (1.0 - deadband);
		  } else {
			return (value + deadband) / (1.0 - deadband);
		  }
		} else {
		  return 0.0;
		}
	  }
	  public static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.05);
	
		// Square the axis
		value = Math.copySign(value * value, value);
	
		return value;
	  }
}
