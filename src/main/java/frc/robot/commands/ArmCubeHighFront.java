package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmCubeHighFront extends SequentialCommandGroup {
	public ArmCubeHighFront(){
		addCommands(new ArmMove(PositionsPivot.HIGH_CUBE_FRONT, PositionsWrist.HIGH_CUBE_FRONT, 0,1));
	}
}