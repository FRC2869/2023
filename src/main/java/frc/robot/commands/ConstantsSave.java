// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConstantsSave extends InstantCommand {
  private WristSubsystem wrist;
  private PivotSubsystem pivot;

  public ConstantsSave() {
    wrist = WristSubsystem.getInstance();
    pivot = PivotSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    wrist.savePositions();
    pivot.savePositions();
    try {
      WristConstants.writeConstants();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    try {
      PivotConstants.writeConstants();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }
}
