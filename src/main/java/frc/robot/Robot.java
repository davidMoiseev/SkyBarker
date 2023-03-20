package frc.robot;

import java.util.List;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.BlueAutoLeft;
import frc.robot.Autons.BlueAutoLeft1Bal;
import frc.robot.Autons.BlueAutoLeft2half;
import frc.robot.Autons.BlueAutoLeft3;
import frc.robot.Autons.BlueAutoMid1Bal;
import frc.robot.Autons.BlueAutoRight;
import frc.robot.Autons.OhCrap;
import frc.robot.Autons.RedAutoLeft;
import frc.robot.Autons.RedAutoLeft1Bal;
import frc.robot.Autons.RedAutoMid1Bal;
import frc.robot.Autons.RedAutoRight;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hopper;
// import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Hopper hopper;
    private Arm arm;
    private LED leds;
    // private Intake intake;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private BlueAutoRight blueAutoRight;
    private Gripper gripper;
    private BlueAutoLeft blueAutoLeft;
    private OhCrap ohCrap;
    private RedAutoRight redAutoRight;
    private RedAutoLeft redAutoLeft;
    private RedAutoLeft1Bal redAutoLeft1Bal;
    private BlueAutoLeft1Bal blueAutoLeft1Bal;
    // private RedAutoMid1Bal redAutoMid1Bal;
    private BlueAutoMid1Bal blueAutoMid1Bal;
    private BlueAutoLeft3 blueAutoLeft3;
    private BlueAutoLeft2half blueAutoLeft2half;

    private String autonSelection = "Red Mid 1";

    private Alliance alliance;

    private String autoSelected = "Blue Left";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY",
        "Shoulder Absolute Pos", "Shoulder Motor Pos", "Extension Pos", "Elbow Absolute Pos", "Elbow Motor Pos",
        "Shoulder Desired Pos", "Extension Desired Pos", "Elbow Desired Pos",
        "HopSensor Bottom", "HopSensor Left", "HopSensor Right", "HopSensor Top", "Hopper Override","determineArmZoneHandOff","DesiredIntakeAngle");

        m_chooser.setDefaultOption("Blue Left", "Blue Left");
        m_chooser.addOption("Red Right", "Red Right");
        m_chooser.addOption("Blue Mid 1", "Blue Mid 1");
        m_chooser.addOption("Red Mid 1 (Intake Towards Right)", "Red Mid 1");
        m_chooser.addOption("Blue Left 3", "Blue Left 3");
        m_chooser.addOption("Blue Left 2 Half", "Blue Left 2 Half");
        // m_chooser.addOption("Red Mid 1 (Intake Towards Right)", "Red Mid 1");

        Shuffleboard.getTab("Competition")
        .add("Auto Selector", m_chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 2);

        //intake = new Intake();
        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        gripper = new Gripper(Constants.GRIPPER);
        hopper = new Hopper();
        leds = new LED();
        blueAutoRight = new BlueAutoRight();
        blueAutoLeft = new BlueAutoLeft();
        ohCrap = new OhCrap();
        redAutoRight = new RedAutoRight();
        redAutoLeft = new RedAutoLeft();
        redAutoLeft1Bal = new RedAutoLeft1Bal();
        blueAutoLeft1Bal = new BlueAutoLeft1Bal();
        blueAutoMid1Bal = new BlueAutoMid1Bal();
        // redAutoMid1Bal = new RedAutoMid1Bal();
        blueAutoLeft3 = new BlueAutoLeft3();
        blueAutoLeft2half = new BlueAutoLeft2half();

        camera.disabled();

    }

    @Override
    public void robotPeriodic() {
        arm.updatePose();
        camera.logData();
        pigeon.logData();
        hopper.logData();
        arm.logdata();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("FPGA Time", Timer.getFPGATimestamp());
        drivetrain.updatePose();
        SmartDashboard.putString("Auton Selected", autoSelected);
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot Mode", "Disabled");

        camera.disabled();
        arm.coastMode();
        leds.disabledAction();
    }

    @Override
    public void disabledPeriodic() {
        arm.armPercentOutZero();
        arm.coastMode();
    }


    @Override
    public void autonomousInit() {
        autonSelection = m_chooser.getSelected();

        // Shuffleboard.getTab("Competition")
        // .add("Selected Auto", autonSelection);

        SmartDashboard.putString("Robot Mode", "Autonomous");

        SmartDashboard.getString("Auton Selection", autonSelection);

        if(autonSelection == "Blue Left"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoLeft);
        } else if(autonSelection == "Blue Right"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoRight);
        } else if(autonSelection == "Red Left"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(redAutoLeft);
        } else if(autonSelection == "Red Right"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(redAutoRight);
        }else if(autonSelection == "Red Left 1"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(redAutoLeft1Bal);
        }else if(autonSelection == "Red Right 1"){
            // autonCommader.initAuton(redAutoRight1Bal);
        }else if(autonSelection == "Blue Left 1"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoLeft1Bal);
        }else if(autonSelection == "Blue Right 1"){
            // autonCommader.initAuton(redAutoLeft1Bal);
        }else if(autonSelection == "Blue Mid 1"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoMid1Bal);
        }else if(autonSelection == "Red Mid 1"){
            alliance = Alliance.Blue;
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoMid1Bal);
        } else if(autonSelection == "Blue Left 3"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoLeft3);
        } else if(autonSelection == "Blue Left 2 Half"){
            alliance = DriverStation.getAlliance();
            autonCommader.allaince = alliance;
            autonCommader.initAuton(blueAutoLeft2half);
        }else {
            autonCommader.initAuton(ohCrap);
        }

        if(alliance == Alliance.Blue){
            drivetrain.zero(-90);
            autonCommader.auton.reset();
            Pigeon.zeroSensor(-90);
        } else {
            drivetrain.zero(90);
            autonCommader.auton.reset();
            Pigeon.zeroSensor(90);
        }

        // camera.enabled();

        arm.initilizeOffsets();
        leds.autonInit();
    }

    @Override
    public void autonomousPeriodic() {
        autonCommader.runAuto();
        pigeon.enabledAction(teleopCommander);
        drivetrain.autonAction(autonCommader);
        arm.action(autonCommader);
        // hopper.HopperPeriodic(autonCommader);
        gripper.action(autonCommader);
        leds.autonAction();
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Robot Mode", "Teleop");

        teleopCommander.allaince = alliance;

        drivetrain.zero();

        // camera.enabled();

        arm.brakeMode();
        arm.initilizeOffsets();
    }

    @Override
    
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(teleopCommander);
        arm.action(teleopCommander);
        arm.brakeMode();
        gripper.action(teleopCommander);
        // hopper.HopperPeriodic(teleopCommander);
        leds.teleopAction();
    }
}
