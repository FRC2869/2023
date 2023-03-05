// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmBasePos;
import frc.robot.commands.ArmConeLow;
import frc.robot.commands.ArmConeMid;
import frc.robot.commands.ArmCubeHigh;
import frc.robot.commands.ArmCubeLow;
import frc.robot.commands.ArmCubeMid;
import frc.robot.commands.ArmDoubleSubStation;
import frc.robot.commands.arm.ArmDefault;
import frc.robot.commands.autonomous.AutoBack;
import frc.robot.commands.autonomous.AutoChargeStationOn;
import frc.robot.commands.autonomous.AutoForwards;
import frc.robot.commands.autonomous.AutoRight;
import frc.robot.commands.grabber.CloseGrabber;
import frc.robot.commands.grabber.OffGrabber;
import frc.robot.commands.grabber.OpenGrabber;
import frc.robot.commands.pivot.PivotDefault;
import frc.robot.commands.pivot.PivotPosPwrSwitch;
import frc.robot.commands.swerve.SwerveDriveAutoBalance;
import frc.robot.commands.swerve.SwerveDriveDrive;
import frc.robot.commands.swerve.SwerveDriveResetGyro;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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

	private final ArmSubsystem arm = ArmSubsystem.getInstance();
	private final PivotSubsystem pivot = PivotSubsystem.getInstance();
	private final GrabberSubsystem grabber = GrabberSubsystem.getInstance();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		CommandScheduler.getInstance().cancelAll();
		configureDefaultCommands();
		configureBindings();
		// grabber.compressorOn();
	}

	/****************/
	/*** DEFAULTS ***/
	/****************/

	private void configureDefaultCommands() {
		// m_SwerveSubsystem.setDefaultCommand(new SwerveDriveDrive());
		swerve.setDefaultCommand(new SwerveDriveDrive());
		pivot.setDefaultCommand(new PivotDefault());
		arm.setDefaultCommand(new ArmDefault());
		// grabber.setDefaultCommand(new OffGrabber());
	}

	/***************/
	/*** BUTTONS ***/
	/***************/
	private void configureBindings() {
		configureDriverBindings();
	}

	/**
	 * 
	 */
	private void configureDriverBindings() {
		// Inputs.getBalanceButton().onTrue(new SwerveDriveAutoBalance());
		// Inputs.getPivotPos().onTrue(new PivotPosPwrSwitch(true));
		// Inputs.getPivotPwr().onTrue(new PivotPosPwrSwitch(false));
		// Inputs.getArmConeLow().onTrue(new ArmConeLow());
		// Inputs.getArmConeMid().onTrue(new ArmConeMid());
		// Inputs.getArmConeHigh().onTrue(new ArmConeHigh());
		// Inputs.getArmCubeLow().onTrue(new ArmCubeLow());
		// Inputs.getArmCubeMid().onTrue(new ArmCubeMid());
		// Inputs.getArmCubeHigh().onTrue(new ArmCubeHigh());
		Inputs.getCloseGrabber().whileTrue(new CloseGrabber());
		Inputs.getOpenGrabber().whileTrue(new OpenGrabber());
		Inputs.getOffGrabber().onTrue(new OffGrabber());
		Inputs.getResetGyroButton().onTrue(new SwerveDriveResetGyro());
		Inputs.getArmDoubleSubStation().onTrue(new ArmDoubleSubStation());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return new SequentialCommandGroup(new AutoForwards(), new AutoBack(), new AutoRight(), new AutoChargeStationOn(), new SwerveDriveAutoBalance());
		// return new SequentialCommandGroup(new AutoChargeStationOn(), new SwerveDriveAutoBalance());
		return new SequentialCommandGroup(new ArmConeMid(), new OpenGrabber(), new ArmBasePos());
	}
}
