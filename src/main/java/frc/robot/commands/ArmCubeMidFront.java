package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmCubeMidFront extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private WristSubsystem wrist;
	private int wristCounter;
	private boolean pivotDone;
	private boolean wristDone;
	private boolean hasRun;

	public ArmCubeMidFront(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		System.out.println("Cube Mid");
		// addRequirements(arm);
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute() {
		if (!hasRun) {
			System.out.println(Constants.autoTimer.get() + ": Arm Cube Mid Start");
			hasRun = true;
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.midConeDistance);
		if(pivotCounter < Constants.pidTimer){
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midCubeFrontAngle);
		}else if(wristCounter < Constants.pidTimer){
		wrist.setPositionControl(true);
		wrist.position(WristConstants.midCubeFrontAngle);
		}
	}

	@Override
	public boolean isFinished() {
		// boolean armDone =
		// Math.abs(arm.getPosition()-ArmConstants.Extension.midConeDistance) <
		// ArmConstants.Extension.tolerance;
		pivotDone = Math.abs(pivot.getAngle() - PivotConstants.midCubeFrontAngle) < PivotConstants.tolerance;
		wristDone = Math.abs(wrist.getAngle() - WristConstants.midCubeFrontAngle) < WristConstants.tolerance;

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
		} else if(pivotCounter > Constants.pidTimer){
			pivot.setPositionControl(false);
		} else if(wristCounter> Constants.pidTimer){
			wrist.setPositionControl(false);
		}
		return false;
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}
