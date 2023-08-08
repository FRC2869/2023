package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmFloorPickupCube extends CommandBase {
	private PivotSubsystem pivot;
	private int pivotCounter;
	private boolean hasRun = false;
	private WristSubsystem wrist;
	private int wristCounter;
	private boolean pivotDone;
	private boolean wristDone;
	private double startTime;
	private final double targetPivotPos = PivotConstants.floorPickupCubeAngle;
	private final double targetWristPos = WristConstants.floorPickupCubeAngle;

	public ArmFloorPickupCube() {
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
		}
		if((Constants.autoTimer.get()-startTime)>1){
		wrist.position(targetWristPos);
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
