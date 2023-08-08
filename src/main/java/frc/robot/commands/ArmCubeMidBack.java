package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmCubeMidBack extends CommandBase {
	private PivotSubsystem pivot;
	private int pivotCounter;
	private boolean hasRun = false;
	private WristSubsystem wrist;
	private int wristCounter;
	private boolean pivotDone;
	private boolean wristDone;
	private double startTime;
	private final double targetPivotPos = PivotConstants.midCubeBackAngle;
	private final double targetWristPos = WristConstants.midCubeBackAngle;

	public ArmCubeMidBack() {
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute() {
		if (!hasRun) {
			startTime = Constants.autoTimer.get();
			System.out.println(startTime + ": Arm Cone Mid Start");
			hasRun = true;
		}
		if(pivotCounter < Constants.pidTimer){
			pivot.position(targetPivotPos);
			pivot.setCurrentPosition(PositionsPivot.MID_CUBE_BACK);
		}
		if((Constants.autoTimer.get()-startTime)>1){
			wrist.position(targetWristPos);
			wrist.setCurrentPosition(PositionsWrist.MID_CUBE_BACK);
		}
	}

	@Override
	public boolean isFinished() {
		pivotDone = Math.abs(pivot.getAngle() - targetPivotPos) < PivotConstants.tolerance;
		wristDone = Math.abs(wrist.getAngle() - targetWristPos) < WristConstants.tolerance;

		if (wristDone) {
			wristCounter++;
		} else {
			wristCounter = 0;
		}
		if (pivotDone) {
			pivotCounter++;
		} else {
			pivotCounter = 0;
		}
		if (pivotCounter > Constants.pidTimer && wristCounter > Constants.pidTimer) {
			System.out.println(Constants.autoTimer.get() + ": Arm Cone Mid Done");
			return true;
		}
		return false;
	}

	@Override
	public void end(boolean isInterrupted) {
	}
}
