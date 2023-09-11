package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmConeHigh extends SequentialCommandGroup {
	public ArmConeHigh(){
		addCommands(new ArmMove(PositionsPivot.HIGH_CONE, PositionsWrist.HIGH_CONE, 0,1));
	}
}