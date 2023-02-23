// package frc.robot.subsystems.odometry;

// import org.team2869.subsystems.swerve.SwerveSubsystem;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class OdometryImpl extends Odometry {
//     private final SwerveDrivePoseEstimator poseEstimator;
//     private final SwerveDriveOdometry odometry;
//     private final Field2d field;

//     private final FieldObject2d odometryPose2d;
//     public OdometryImpl() {   
//         var swerve = SwerveSubsystem.getInstance();
//         var startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//         poseEstimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), startingPose);
//         odometry = new SwerveDriveOdometry(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), startingPose);
//         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1000000, 100000, Units.degreesToRadians(10000)));
//         field = new Field2d();

//         odometryPose2d = field.getObject("Odometry Pose");

//         swerve.initFieldObjects(field);
//         SmartDashboard.putData("Field", field);
//     }

//     @Override
//     public Field2d getField() {
//         return field;
//     }

//     @Override
//     public void reset(Pose2d pose) {
//         SwerveSubsystem drive = SwerveSubsystem.getInstance();
//         poseEstimator.resetPosition(
//                     drive.getGyroAngle(), 
//                     drive.getModulePositions(), 
//                     pose
//         );
//         odometry.resetPosition(drive.getGyroAngle(), 
//         drive.getModulePositions(), 
//         pose);
//     }

//     @Override
//     public Pose2d getPose() {
//         // TODO Auto-generated method stub
//         return poseEstimator.getEstimatedPosition();
//     }
    
//     private void processResults(SwerveSubsystem drive){ 
//         poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
//         odometry.update(drive.getGyroAngle(), drive.getModulePositions());
//     }

//     @Override
//     public void periodic() {

//         SwerveSubsystem drive = SwerveSubsystem.getInstance();
//         processResults(drive);        

//         field.setRobotPose(getPose());

//         Pose2d pose = odometry.getPoseMeters();
//         odometryPose2d.setPose(pose);
//         SmartDashboard.putNumber("Odometry/Odometry Pose X", pose.getX());
//         SmartDashboard.putNumber("Odometry/Odometry Pose Y", pose.getY());
//         SmartDashboard.putNumber("Odometry/Odometry Rotation", pose.getRotation().getDegrees());

        
//         SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
//         SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
//         SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        
//     }
// }
