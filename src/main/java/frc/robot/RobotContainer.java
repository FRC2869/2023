// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;
import frc.robot.commands.ArmMove;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PivotAdjustDown;
import frc.robot.commands.PivotAdjustUp;
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
		Nothing, Forward, ScoreMidCone, ScoreMidConeAndMove, TurnToCube, ScoreMidConeAndScoreCube, ScoreMidConeAndChargeStation, ChargeStation, ScoreMidConeAndScoreCubeAndChargeStation, ScoreMidConeAndCrossChargeStation, HumanPlayerSideScoreAndMove, EdgeSideScoreAndMove, REDHumanPlayerSideScoreAndMove, BLUEHumanPlayerSideScoreAndMove, BLUEBumpSideScoreAndMove, REDBumpSideScoreAndMove
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
		newautopick.addOption("ScoreMidConeAndMove", Autos.ScoreMidConeAndMove);
		// newautopick.addOption("ScoreMidConeAndPickupCube", Autos.ScoreMidConeAndPickupCube);
		// newautopick.addOption("ScoreMidConeAndScoreCube", Autos.ScoreMidConeAndScoreCube);
		newautopick.addOption("ScoreMidConeAndChargeStation", Autos.ScoreMidConeAndChargeStation);
		newautopick.addOption("ScoreMidConeAndCrossChargeStation", Autos.ScoreMidConeAndCrossChargeStation);
		newautopick.addOption("ChargeStation", Autos.ChargeStation);
			newautopick.addOption("REDHumanPlayerSideScoreAndMove", Autos.REDHumanPlayerSideScoreAndMove);
			newautopick.addOption("BLUEHumanPlayerSideScoreAndMove", Autos.BLUEHumanPlayerSideScoreAndMove);
			newautopick.addOption("REDBumpSideScoreAndMove", Autos.REDBumpSideScoreAndMove);
			newautopick.addOption("BLUEBumpSideScoreAndMove", Autos.BLUEBumpSideScoreAndMove);
			newautopick.addOption("EdgeSideScoreAndMove", Autos.EdgeSideScoreAndMove);
			newautopick.addOption("TurnToCube", Autos.TurnToCube);
		// newautopick.addOption("ScoreMidConeAndScoreCubeAndChargeStation", Autos.ScoreMidConeAndScoreCubeAndChargeStation);

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
			() -> (true), true, true
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
			// case ChargeStation:
			// return new SequentialCommandGroup(
			// 	new AutoCrossChargeStation(),
			// 	new WaitCommand(.5),
			// 	new AutoChargeStationOnBack(),

			// 	// new AutoChargeStationOn(), 
			// 	// between 2.9 and 2.65
			// 	new AutoForwardsAprilTags(2.7),
			// 	// new AutoForwardsDist(SmartDashboard.getNumber("Charge Station Dist", 2.7), .6),
			// 	// new ParallelRaceGroup(new SwerveDriveAutoBalance(), new WaitCommand(3)),
			// 	new NaiveBalance(),
			// 	// new AutoForwards(SmartDashboard.getNumber("Charge Station Dist", 2.2)),
			// 	new WaitCommand(500));
			// case Forward:
			// // return new Print();
			// return new SequentialCommandGroup(
			// 	new AutoForwardsDist(5.2),
			// 	new WaitCommand(500));
			// 	// return new AutoForwards();
			// case REDHumanPlayerSideScoreAndMove:
			// return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 		// new WaitCommand(.5), 
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  

			// 		new ParallelRaceGroup(new AutoForwardsDist(4.8), new SequentialCommandGroup(new WaitCommand(1), new ArmFloorPickupBack())),
			// 		new AutoDriveLeftAndTurn(180, .8),
			// 		new ParallelRaceGroup(new CloseGrabberFast(), new ArmFloorPickupBack(), new WaitCommand(1)),
			// 		// new AutoDriveAndTurn(180, .75),
			// 		new ParallelDeadlineGroup(new AutoGoToCube(), new ArmFloorPickupBack(), new WaitCommand(5)),
			// 		// new ParallelRaceGroup(new CloseGrabber(), new ArmBasePos()),
			// 		// new ParallelRaceGroup(new AutoBack(5), new CloseGrabber(), new ArmCubeMid()),
			// 		// new ArmCubeMid(),
			// 		// new OpenGrabber(),
			// 		new WaitCommand(500));
			// case BLUEBumpSideScoreAndMove:
			// return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 		// new WaitCommand(.5), 
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  

			// 		new ParallelRaceGroup(new AutoForwardsDist(5), new SequentialCommandGroup(new WaitCommand(1), new ArmFloorPickupBack())),
			// 		new AutoDriveLeftAndTurn(180, .8),
			// 		new ParallelRaceGroup(new CloseGrabberFast(), new ArmFloorPickupBack(), new WaitCommand(1)),
			// 		// new AutoDriveAndTurn(180, .75),
			// 		new ParallelDeadlineGroup(new AutoGoToCube(), new ArmFloorPickupBack(), new WaitCommand(5)),
			// 		// new ParallelRaceGroup(new CloseGrabber(), new ArmBasePos()),
			// 		// new ParallelRaceGroup(new AutoBack(5), new CloseGrabber(), new ArmCubeMid()),
			// 		// new ArmCubeMid(),
			// 		// new OpenGrabber(),
			// 		new WaitCommand(500));
			// case BLUEHumanPlayerSideScoreAndMove:
			// return new SequentialCommandGroup(
			// 	new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 	// new WaitCommand(.5), 
			// 	new OpenGrabber1sec(),
			// 	new OffGrabber(),  

			// 	new ParallelRaceGroup(new AutoForwardsDist(4.8), new SequentialCommandGroup(new WaitCommand(1), new ArmFloorPickupBack())),
			// 	new AutoDriveRightAndTurn(180, .8),
			// 	new ParallelRaceGroup(new CloseGrabberFast(), new ArmFloorPickupBack(), new WaitCommand(1)),
			// 	// new AutoDriveAndTurn(180, .75),
			// 	new ParallelDeadlineGroup(new AutoGoToCube(), new ArmFloorPickupBack(), new WaitCommand(5)),
			// 	// new ParallelRaceGroup(new CloseGrabber(), new ArmBasePos()),
			// 	// new ParallelRaceGroup(new AutoBack(5), new CloseGrabber(), new ArmCubeMid()),
			// 	// new ArmCubeMid(),
			// 	// new OpenGrabber(),
			// 	new WaitCommand(500));
			// case REDBumpSideScoreAndMove:
			// return new SequentialCommandGroup(
			// 	new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 	// new WaitCommand(.5), 
			// 	new OpenGrabber1sec(),
			// 	new OffGrabber(),  

			// 	new ParallelRaceGroup(new AutoForwardsDist(5), new SequentialCommandGroup(new WaitCommand(1), new ArmFloorPickupBack())),
			// 	new AutoDriveRightAndTurn(180, .8),
			// 	new ParallelRaceGroup(new CloseGrabberFast(), new ArmFloorPickupBack(), new WaitCommand(1)),
			// 	// new AutoDriveAndTurn(180, .75),
			// 	new ParallelDeadlineGroup(new AutoGoToCube(), new ArmFloorPickupBack(), new WaitCommand(5)),
			// 	// new ParallelRaceGroup(new CloseGrabber(), new ArmBasePos()),
			// 	// new ParallelRaceGroup(new AutoBack(5), new CloseGrabber(), new ArmCubeMid()),
			// 	// new ArmCubeMid(),
			// 	// new OpenGrabber(),
			// 	new WaitCommand(500));
			// case Nothing:
			// 	return new WaitCommand(5000);
			// case ScoreMidCone:
			// 	return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(5)),
			// 		new WaitCommand(.5), 
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  
			// 		new ArmBasePos(), new WaitCommand(500));
			// case ScoreMidConeAndCrossChargeStation:
			// 	return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  
			// 		new ArmBasePos(), 
			// 		new AutoCrossChargeStation(),
			// 		new AutoChargeStationOnBack(), 
			// 		new ParallelRaceGroup(new SwerveDriveAutoBalance(), new WaitCommand(3)),
			// 		new WaitCommand(500));
			// case ScoreMidConeAndChargeStation:
			// 	return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(3)),
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  
			// 		new ArmBasePos(), 
			// 		new AutoChargeStationOn(), 
			// 		new ParallelRaceGroup(new NaiveBalance(), new WaitCommand(8)),
			// 		new WaitCommand(500));
			// case ScoreMidConeAndMove:
			// 	System.out.println("AUTO");
			// 	return new SequentialCommandGroup(
			// 		new ParallelRaceGroup(new ArmConeMidBack(), new WaitCommand(4)),
			// 		new WaitCommand(.25), 
			// 		new OpenGrabber1sec(),
			// 		new OffGrabber(),  
			// 		new ParallelRaceGroup(new ArmBasePos(), new WaitCommand(5)), 
			// 		new AutoForwards(), 
			// 		// new SwerveStop(),
			// 		new WaitCommand(500));
			// case TurnToCube:
			// 	return new AutoGoToCube();
			// case ScoreMidConeAndScoreCube:
			// 	break;
			// case ScoreMidConeAndScoreCubeAndChargeStation:
			// 	break;
			default:
				return new WaitCommand(5000);			
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
