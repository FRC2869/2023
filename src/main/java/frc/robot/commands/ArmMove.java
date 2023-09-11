package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmMove extends CommandBase {
	private boolean hasRun = false;

	private double startTime;
	
	private final PivotSubsystem pivot;
	private final WristSubsystem wrist;

	private final double targetPivotPos;
	private final double targetWristPos;
	
	private final PositionsPivot pivotEnum;
	private final PositionsWrist wristEnum;
	
	private final double pivotDelay;
	private final double wristDelay;
	
	private int pivotCounter;
	private int wristCounter;
	
	private boolean pivotDone;
	private boolean wristDone;
	
	private boolean hasSetPivot = false;
	private boolean hasSetWrist = false;

	public ArmMove(PositionsPivot pivotEnum, PositionsWrist wristEnum, double pivotDelay, double wristDelay) {
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		
		this.pivotEnum = pivotEnum;
		this.wristEnum = wristEnum;
		
		this.targetPivotPos = PivotConstants.getTargetPos(pivotEnum);
		this.targetWristPos = WristConstants.getTargetPos(wristEnum);

		this.pivotDelay = pivotDelay;
		this.wristDelay = wristDelay;
		
		addRequirements(pivot);
		addRequirements(wrist);

	}

	@Override
	public void execute() {
		if (!hasRun) {
			startTime = Constants.autoTimer.get();
			System.out.println(startTime + ": Arm Movement:" + pivotEnum.name());
			hasRun = true;
		}

		if((Constants.autoTimer.get()-startTime)>pivotDelay && !hasSetPivot){
			pivot.position(targetPivotPos);
			pivot.setCurrentPosition(pivotEnum);
			hasSetPivot = true;
		}

		if((Constants.autoTimer.get()-startTime)>wristDelay && !hasSetWrist){
			wrist.position(targetWristPos);
			wrist.setCurrentPosition(wristEnum);
			hasSetWrist = true;
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
			System.out.println(Constants.autoTimer.get() + ": Arm Movement Done:" + pivotEnum.name());
			return true;
		}
		return false;
	}

	@Override
	public void end(boolean isInterrupted) {
	}
}
