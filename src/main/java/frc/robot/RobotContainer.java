// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos;
import frc.robot.commands.arm.ArmDefault;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.swerve.SwerveSubsystem;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
//   private final SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();
private final ArmSubsystem arm = ArmSubsystem.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
//   private final CommandXboxController m_driverController = new CommandXboxController(
//       OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    // m_SwerveSubsystem.setDefaultCommand(new SwerveDriveDrive());
	arm.setDefaultCommand(new ArmDefault());
  }

  /***************/
  /*** BUTTONS ***/
  /***************/
  private void configureBindings() {
    configureDriverBindings();
  }

  private void configureDriverBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
