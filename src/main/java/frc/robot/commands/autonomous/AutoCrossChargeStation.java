package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoCrossChargeStation extends CommandBase{
	private DrivetrainSubsystem swerve;
    boolean hasCrossed = false;  //on top of the charge station.
    private boolean hasFullyCrossed;

	public AutoCrossChargeStation(){
		swerve = DrivetrainSubsystem.getInstance();
		addRequirements(swerve);
	}

	@Override
	public void initialize(){
		DrivetrainSubsystem.enable();
	}

	@Override
	public void execute(){
        if(swerve.getGyroscopeRotation().getDegrees() > 5) {
            //rotate a bit.
            swerve.drive(0,0,.1*SwerveConstants.kMaxAngularSpeed);
        }
        if (swerve.getGyroscopeRotation().getDegrees() < -5) {
            //rotate a bit.
            swerve.drive(0,0,-.1*SwerveConstants.kMaxAngularSpeed);
        }else{
		swerve.drive(RobotContainer.modifyAxis(.7)* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,0,0);
        }
    }

    /**
     * Determines if the bot has driven up and over the charging station.
     */
	@Override
	public boolean isFinished() {
		if(swerve.getAdjustedGyroPitch().getDegrees()>10) {
            System.out.println(Constants.autoTimer.get()+": Uphill");

			hasCrossed = true;
		}
        //Do we want to rotate while we drive up or just do that when we are on
        //the level charging station.

        
        //LL: this was previously < 10. Might want -10 instead.
        //LL: chenaged to else if instead of if.
        if(hasCrossed && swerve.getAdjustedGyroPitch().getDegrees()<-10) {
            System.out.println(Constants.autoTimer.get()+": Downhill");

            hasFullyCrossed = true;
            return false;
        }
        if (hasCrossed && hasFullyCrossed && swerve.getAdjustedGyroPitch().getDegrees()<1){
            swerve.drive(0, 0, 0);
            System.out.println(Constants.autoTimer.get()+": Crossed Charging Station");
            return true;
        }
        return false;
	}
}
