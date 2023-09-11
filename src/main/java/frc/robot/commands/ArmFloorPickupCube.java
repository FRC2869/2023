package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmFloorPickupCube extends SequentialCommandGroup {
	public ArmFloorPickupCube(){
		addCommands(new ArmMove(PositionsPivot.FLOOR_CUBE, PositionsWrist.FLOOR_CUBE, 0,1));
	}
}