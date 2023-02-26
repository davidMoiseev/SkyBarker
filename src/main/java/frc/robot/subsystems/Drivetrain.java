package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.MAX_VOLTAGE;
import static frc.robot.Constants.realBot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.TeleopCommander;
import frc.robot.sensors.Pigeon;

public class Drivetrain{
    private static SwerveModule frontLeftModule;
    private static SwerveModule frontRightModule;
    private static SwerveModule backLeftModule;
    private static SwerveModule backRightModule;

    private static TalonFX frontLeftDrive;
    private static TalonFX frontRightDrive;
    private static TalonFX backLeftDrive;
    private static TalonFX backRightDrive;

    static SwerveModulePosition[] positions;

    public static SwerveDrivePoseEstimator poseEstimator;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveModuleState[] states;

    Timer timer = new Timer();

    public static SwerveDriveKinematics kinematics;

    private HolonomicDriveController holonomicController;

    static double frontLeftPos;
    static double frontRightPos;
    static double backLeftPos;
    static double backRightPos;

    public Drivetrain(){
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        kinematics = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(2, 0),
                                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(4, 0),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                        .withSize(2, 4)
                                        .withPosition(6, 0),
                        Mk4iSwerveModuleHelper.GearRatio.L2,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET);

        frontLeftDrive = frontLeftModule.getDriveMotor();
        frontRightDrive = frontRightModule.getDriveMotor();
        backLeftDrive = backLeftModule.getDriveMotor();
        backRightDrive = backRightModule.getDriveMotor();

        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();

        positions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

        ProfiledPIDController thetaController = new ProfiledPIDController(1, .3, 0.06,
                                                new TrapezoidProfile.Constraints(6.28, 3.14));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(
            new PIDController(3.5, .6, .025),  //x Long side of field
            new PIDController(20, .6, .025), //y Short side of field
            thetaController); // (2Pk,PI) constrains to 1 2pi/sec

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Pigeon.getRotation2d(),
            positions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

        poseEstimator.resetPosition(Rotation2d.fromDegrees(-90), positions, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
    }

    public void zero(double angle){
        frontLeftModule.zeroMotorPos();
        frontRightModule.zeroMotorPos();
        backLeftModule.zeroMotorPos();
        backRightModule.zeroMotorPos();

        poseEstimator.resetPosition(Rotation2d.fromDegrees(angle), positions, new Pose2d(0,0, Rotation2d.fromDegrees(angle)));
    }

    public static void zeroPositions(){
        frontLeftDrive.setSelectedSensorPosition(0);
        frontRightDrive.setSelectedSensorPosition(0);
        backLeftDrive.setSelectedSensorPosition(0);
        backRightDrive.setSelectedSensorPosition(0);

        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;
    }

    public void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());
    }

    public void setModulePositions(double speed, double angle){
        frontLeftModule.set(speed, Math.toRadians(angle));
        frontRightModule.set(speed, Math.toRadians(angle));
        backLeftModule.set(speed, Math.toRadians(angle));
        backRightModule.set(speed, Math.toRadians(angle));
    }

    public void teleAction(TeleopCommander commander){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            commander.getForwardCommand(),
            commander.getStrafeCommand(),
            commander.getTurnCommand(),
            Pigeon.getRotation2d());
    
        setSwerveModuleStates(chassisSpeeds);
    }
    
    public void autonAction(AutonCommader autonCommader){
        // if(autonCommader.isDriving()){
        driveToPos(autonCommader.getDesiredState(), autonCommader.getTargetTheta());
        // } else {
        //     setSwerveModuleStates(new ChassisSpeeds(0,0,0));
        // }

        SmartDashboard.putNumber("_Time", timer.get());
    }

    public void driveToPos(State state, Rotation2d theta){
        ChassisSpeeds speeds = holonomicController.calculate(poseEstimator.getEstimatedPosition(), 
        state,
        theta);
        
        setSwerveModuleStates(speeds);

        SmartDashboard.putNumber("Commanded Forward", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Commanded Sideways", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Commanded Turn", speeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("Commanded Theta",theta.getDegrees());
        SmartDashboard.putNumber("Commanded X", state.poseMeters.getX());
        SmartDashboard.putNumber("Commanded Y", state.poseMeters.getY());
    }

    public void updatePose(){
        frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Pigeon.getRotation2d(), positions);

        SmartDashboard.putNumber("Estimated Theta", Rotation2d.fromDegrees(Pigeon.getAngle()).getDegrees());
        SmartDashboard.putNumber("Estimated X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimated Y", poseEstimator.getEstimatedPosition().getY());
    }
}
