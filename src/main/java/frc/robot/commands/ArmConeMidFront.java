package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmConeMidFront extends CommandBase {
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private boolean hasRun = false;
	private WristSubsystem wrist;
	private int wristCounter;
	private boolean pivotDone;
	private boolean wristDone;
	private double startTime;

	public ArmConeMidFront() {
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
			startTime = Constants.autoTimer.get();
			System.out.println(startTime + ": Arm Cone Mid Start");
			hasRun = true;
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.midConeDistance);
		if(pivotCounter < Constants.pidTimer){
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midConeFrontAngle);
		}
		if((Constants.autoTimer.get()-startTime)>1){
		wrist.setPositionControl(true);
		wrist.position(WristConstants.midConeFrontAngle);
		}
	}

	@Override
	public boolean isFinished() {
		// boolean armDone =
		// Math.abs(arm.getPosition()-ArmConstants.Extension.midConeDistance) <
		// ArmConstants.Extension.tolerance;
		pivotDone = Math.abs(pivot.getAngle() - PivotConstants.midConeFrontAngle) < PivotConstants.tolerance;
		wristDone = Math.abs(wrist.getAngle() - WristConstants.midConeFrontAngle) < WristConstants.tolerance;

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
			return true;
		}
		return false;
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) &&
		// (pivot.getAngle() == PivotConstants.lowConeAngle);
	}

	@Override
	public void end(boolean isInterrupted) {
		pivot.setPositionControl(false);
		wrist.setPositionControl(false);
	}
}
