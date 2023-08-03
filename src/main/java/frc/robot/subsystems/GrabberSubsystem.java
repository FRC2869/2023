package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.RobotContainer;

public class GrabberSubsystem extends SubsystemBase{
	private static GrabberSubsystem instance;
	// private Compressor compressor; 
	// private DoubleSolenoid solenoid1;
	// private DoubleSolenoid solenoid2;

	private WPI_TalonFX grabber1;

	public static GrabberSubsystem getInstance(){
		if(instance == null){
			instance = new GrabberSubsystem();
		}
		return instance;
	}

	public GrabberSubsystem(){
		// compressor = new Compressor(16, PneumaticsModuleType.CTREPCM);
		// solenoid1 = new DoubleSolenoid(16, PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidPortForwards, PneumaticsConstants.solenoidPortBackwards);
		// solenoid2 = new DoubleSolenoid(16, PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidPortForwardsTwo, PneumaticsConstants.solenoidPortBackwardsTwo);
		grabber1 = new WPI_TalonFX(18);
		RobotContainer.auto.addBoolean("Intaked?", ()->isIntaked()).withPosition(6, 1).withSize(1, 5);

		configureGrabberMotors();

	}

	private void configureGrabberMotors() {
		grabber1.configFactoryDefault();

        grabber1.configVoltageCompSaturation(12.0);
		grabber1.setInverted(Motors.Grabber1.kInverted);
		grabber1.setNeutralMode(Motors.Grabber1.idlemode);
		grabber1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Motors.Grabber1.currentLimit, Motors.Grabber1.currentLimit, 1));
		grabber1.configOpenloopRamp(Motors.Grabber1.openLoopRampRate);
	}

	public void closeGrabber(){		
		grabber1.set(-.1);
	}
	public void closeGrabberFast(){
		grabber1.set(-.2);
	}
	public void openGrabber(){
		grabber1.set(.2);
	}

	public void offGrabber(){
		grabber1.set(0);
	}
	@Override
	public void periodic(){
		// SmartDashboard.putBoolean("Intaked?", isIntaked());
		
		// System.out.print(grabber1.getOutputCurrent());
		// System.out.println(grabber2.getOutputCurrent());
	}

	public boolean isIntaked() {
		var current1 = grabber1.getSupplyCurrent();
		return current1 > 15;
	}

}
