package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmConeMidBack extends CommandBase {
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private boolean hasRun = false;
	private WristSubsystem wrist;
	private int wristCounter;

	public ArmConeMidBack() {
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		// System.out.println("Cone Mid");
		// addRequirements(arm);
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute() {
		if (!hasRun) {
			System.out.println(Constants.autoTimer.get() + ": Arm Cone Mid Start");
			hasRun = true;
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.midConeDistance);
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midConeBackAngle);
		wrist.setPositionControl(true);
		wrist.position(WristConstants.midConeBackAngle);
	}

	@Override
	public boolean isFinished() {
		// boolean armDone =
		// Math.abs(arm.getPosition()-ArmConstants.Extension.midConeDistance) <
		// ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle() - PivotConstants.midConeBackAngle) < PivotConstants.tolerance;
		boolean wristDone = Math.abs(wrist.getAngle() - WristConstants.midConeBackAngle) < WristConstants.tolerance;

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
		// System.out.println(pivotCounter);
		if (pivotCounter > Constants.pidTimer && wristCounter > Constants.pidTimer) {
			// System.out.println("DONE");
			System.out.println(Constants.autoTimer.get() + ": Arm Cone Mid Done");
			pivot.setPositionControl(false);
			wrist.setPositionControl(false);
			return true;
		} else {
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) &&
		// (pivot.getAngle() == PivotConstants.lowConeAngle);
	}

	@Override
	public void end(boolean isInterrupted) {
		pivot.setPositionControl(false);
		wrist.setPositionControl(false);
	}
}
