package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * From Sachem 263
 * 
 * It will decrease the amount of time it moves forward and backwards until it 
 * balances on the charging station.
 */
public class BalanceRobotTimeIntervalCommand extends CommandBase {

    private final DrivetrainSubsystem mDrive;

    private double mTiltBackTime;
    private double mTiltForwardTime;

    private State mState;

    private final SlidingWindow mWindow;

    enum State {
        SCALE_CHARGE_STATION,
        TILT_BACK,
        TILT_FORWARD,
        FIND_MIDDLE,
        FINISH
    }

    public BalanceRobotTimeIntervalCommand() {
        mDrive = DrivetrainSubsystem.getInstance();
        mTiltBackTime = 0;
        mTiltForwardTime = 0;
        mState = State.SCALE_CHARGE_STATION;
        mWindow = new SlidingWindow(20, -5.5);
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.driveDirect(SwerveConstants.kFirstSpeed, 0.0, 0.0);
    }

    @Override
    public void execute() {
        mWindow.add(-mDrive.getAdjustedGyroPitch().getDegrees());
        if (!mWindow.windowReady()) return;

        System.out.println("State --> " + mState);
        final double tipAngle = 10.0;
        switch (mState) {
            case SCALE_CHARGE_STATION:
                if (mWindow.getDelta().doubleValue() < -tipAngle) {
                    mState = State.TILT_BACK;
                    mDrive.driveDirect(SwerveConstants.kSecondSpeed + 5, 0, 0);
                }
                break;
            case TILT_BACK:
                if (mWindow.getDelta().doubleValue() > tipAngle) {
                    mState = State.TILT_FORWARD;
                    mTiltBackTime = Timer.getFPGATimestamp();
                    mDrive.driveDirect(-(SwerveConstants.kSecondSpeed + 5), 0, 0);
                }
                break;
            case TILT_FORWARD:
                if (mWindow.getDelta().doubleValue() < -tipAngle - 0.5) {
                    mTiltForwardTime = Timer.getFPGATimestamp();
                    mDrive.driveDirect(SwerveConstants.kSecondSpeed + 5, 0, 0);
                    mState = State.FIND_MIDDLE;
                }
                break;
            case FIND_MIDDLE:
                if (Timer.getFPGATimestamp() >= ((mTiltForwardTime - mTiltBackTime) * 0.5) + mTiltForwardTime) {
                    mState = State.FINISH;
                }
			case FINISH:
				break;
			default:
				break;
        };
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(0,0,0);
        Constants.locked = true;
    }

    @Override
    public boolean isFinished() {
        return mState == State.FINISH;
    }
}
