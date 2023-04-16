package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AHHHH;
import frc.robot.Autons.BlueAutoLeft;
import frc.robot.Autons.BlueAutoLeft3;
import frc.robot.Autons.BlueAutoLeft3Weave;
import frc.robot.Autons.BlueAutoLeft3WeaveBal;
import frc.robot.Autons.BlueAutoLeft3copy;
import frc.robot.Autons.BlueAutoLeftBalance;
import frc.robot.Autons.BlueAutoMid1Bal;
import frc.robot.Autons.BlueAutoRight2;
import frc.robot.Autons.BlueCord;
import frc.robot.Autons.CableAuto;
import frc.robot.Autons.MidCube;
import frc.robot.Autons.NewArmTest;
import frc.robot.Autons.NewMidBlue;
import frc.robot.Autons.OhCrap;
import frc.robot.Autons.RedAutoRight;
import frc.robot.Autons.RedAutoRight3;
import frc.robot.Autons.RedAutoRight3Weave;
import frc.robot.Autons.RedAutoRight3WeaveBal;
import frc.robot.Autons.RedAutoRightBalance;
import frc.robot.Autons.RedCord;
import frc.robot.Autons.RedLeftAuto3;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Hopper hopper;
    private Arm arm;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private Gripper gripper;
    private BlueAutoLeft blueAutoLeft;
    private OhCrap ohCrap;
    private RedAutoRight redAutoRight;
    private BlueAutoMid1Bal blueAutoMid1Bal;
    private BlueAutoLeft3 blueAutoLeft3;
    private RedAutoRightBalance redAutoRightBalance;
    private RedAutoRight3 redAutoRight3;
    private BlueAutoLeftBalance blueAutoLeftBalance;
    private CableAuto cableAuto;
    private BlueAutoRight2 blueAutoRight2;
    private RedAutoRight3Weave weave;
    private NewArmTest newArmTest;
    private RedAutoRight3WeaveBal redRight3Bal;
    private BlueAutoLeft3Weave blueAutoLeft3Weave;
    private BlueAutoLeft3WeaveBal blueAutoLeft3WeaveBal;
    private RedLeftAuto3 redLeftAuto3;
    private AHHHH ahhhh;
    private NewMidBlue newMidBlue;
    private BlueAutoLeft3copy copy;
    private BlueCord blueCord;
    private RedCord redCord;
    private MidCube midCube;

    LED leds;

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
        "TargetX", "TargetY", "TargetTheta", "Pose Theta", "poseX", "poseY",
        "Shoulder Absolute Pos", "Shoulder Motor Pos", "Extension Pos", "Elbow Absolute Pos", "Elbow Motor Pos",
        "Shoulder Desired Pos", "Extension Desired Pos", "Elbow Desired Pos","determineArmZoneHandOff","DesiredIntakeAngle",
        "Commanded Extension Position", "AutoState", "Driving");

        m_chooser.setDefaultOption("Blue Left", "Blue Left");
        m_chooser.addOption("Red Right", "Red Right");
        m_chooser.addOption("Blue Mid 1", "Blue Mid 1");
        m_chooser.addOption("Red Mid 1 (Intake Towards Right)", "Red Mid 1");
        m_chooser.addOption("Blue Left 3", "Blue Left 3");
        m_chooser.addOption("Red Right 3", "Red Right 3");
        m_chooser.addOption("Red Right Balance", "Red Right Balance");
        m_chooser.addOption("Blue Left Balance", "Blue Left Balance");
        m_chooser.addOption("Cable", "Cable");
        m_chooser.addOption("Blue Right 2", "Blue Right 2");
        m_chooser.addOption("Red Left 3", "Red Left 3");
        m_chooser.addOption("Red Weave", "Red Weave");
        m_chooser.addOption("New Arm Test", "New Arm Test");
        m_chooser.addOption("Red Right 3 Bal", "Red Right 3 Bal");
        m_chooser.addOption("Blue Left 3 Bal", "Blue Left 3 Bal");
        m_chooser.addOption("Blue Left 3 Weave", "Blue Left 3 Weave");
        m_chooser.addOption("AHHHH", "AHHHH");
        m_chooser.addOption("New Mid Blue", "New Mid Blue");
        m_chooser.addOption("Blue Cord", "Blue Cord");
        m_chooser.addOption("Red Cord", "Red Cord");
        m_chooser.addOption("Mid Cube", "Mid Cube");

        Shuffleboard.getTab("Competition")
        .add("Auto Selector", m_chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 2);

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        gripper = new Gripper(Constants.GRIPPER);
        hopper = new Hopper();
        blueAutoLeft = new BlueAutoLeft();
        ohCrap = new OhCrap();
        redAutoRight = new RedAutoRight();
        blueAutoMid1Bal = new BlueAutoMid1Bal();
        blueAutoLeft3 = new BlueAutoLeft3();
        leds = new LED();
        redAutoRightBalance = new RedAutoRightBalance();
        redAutoRight3 = new RedAutoRight3();
        blueAutoLeftBalance = new BlueAutoLeftBalance();
        cableAuto = new CableAuto();
        blueAutoRight2 = new BlueAutoRight2();
        weave = new RedAutoRight3Weave();
        newArmTest = new NewArmTest();
        redRight3Bal = new RedAutoRight3WeaveBal();
        blueAutoLeft3Weave = new BlueAutoLeft3Weave();
        blueAutoLeft3WeaveBal = new BlueAutoLeft3WeaveBal();
        redLeftAuto3 = new RedLeftAuto3();
        ahhhh = new AHHHH();
        newMidBlue = new NewMidBlue();
        copy = new BlueAutoLeft3copy();
        blueCord = new BlueCord();
        redCord = new RedCord();
        midCube = new MidCube();

        camera.disabled();

        // PortForwarder.add(5800, "limelight.local", 5800);
        // PortForwarder.add(5801, "limelight.local", 5801);
    }

    @Override
    public void robotPeriodic() {
        arm.updatePose();
        // camera.logData();
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
        arm.brakeMode();
        // arm.coastMode();

        leds.disabledAction();
    }

    @Override
    public void disabledPeriodic() {
        arm.armPercentOutZero();
        // arm.coastMode();
        arm.brakeMode();
        leds.fancyDisable();
    }

    @Override
    public void autonomousInit() {
        autonSelection = m_chooser.getSelected();

        SmartDashboard.putString("Robot Mode", "Autonomous");

        SmartDashboard.getString("Auton Selection", autonSelection);

        alliance = DriverStation.getAlliance();

        if(DriverStation.getAlliance() == Alliance.Red){
            drivetrain.zero(-90, new Pose2d(14.714, 3.905, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
        } else {
            drivetrain.zero(-90, new Pose2d(1.75, 3.905, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
        }

        if(autonSelection == "Blue Left"){
            autonCommader.initAuton(blueAutoLeft);
        } else if(autonSelection == "Red Right"){
            autonCommader.initAuton(redAutoRight);
        }else if(autonSelection == "Blue Mid 1"){
            alliance = Alliance.Blue;
            drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
            autonCommader.initAuton(blueAutoMid1Bal);
        }else if(autonSelection == "Red Mid 1"){
            alliance = Alliance.Blue;
            drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
            autonCommader.initAuton(blueAutoMid1Bal);
        } else if(autonSelection == "Blue Left 3"){
            // alliance = Alliance.Blue;
            // drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            // Pigeon.zeroSensor(-90);
            // autonCommader.initAuton(blueAutoLeft3);
            autonCommader.initAuton(blueAutoLeft3);
        } else if (autonSelection == "Red Right Balance"){
            autonCommader.initAuton(redAutoRightBalance);
        } else if(autonSelection == "Red Right 3"){
            // alliance = Alliance.Blue;
            // drivetrain.zero(90, new Pose2d(0,0, Rotation2d.fromDegrees(90)));
            // Pigeon.zeroSensor(90);
            autonCommader.initAuton(redAutoRight3);
        } else if(autonSelection == "Blue Left Balance"){
            autonCommader.initAuton(blueAutoLeftBalance);
        } else if(autonSelection == "Blue Right 2"){
            alliance = Alliance.Blue;
            drivetrain.zero(90, new Pose2d(0,0, Rotation2d.fromDegrees(90)));
            Pigeon.zeroSensor(90);
            autonCommader.initAuton(blueAutoRight2);
        } else if(autonSelection == "Red Left 3"){
            alliance = Alliance.Blue;
            drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
            autonCommader.initAuton(redLeftAuto3);
        } else if(autonSelection == "AHHHH"){
            alliance = Alliance.Blue;
            drivetrain.zero(90, new Pose2d(0,0, Rotation2d.fromDegrees(90)));
            Pigeon.zeroSensor(90);
            autonCommader.initAuton(ahhhh);
        } else if(autonSelection == "Red Weave"){
            autonCommader.initAuton(weave);
        } else if(autonSelection == "Cable"){
            alliance = Alliance.Blue;
            autonCommader.initAuton(cableAuto);
        } else if(autonSelection == "New Arm Test"){
            autonCommader.initAuton(newArmTest);
        } else if(autonSelection == "Red Right 3 Bal"){
            autonCommader.initAuton(redRight3Bal);
        } else if(autonSelection == "Blue Left 3 Weave"){
            autonCommader.initAuton(blueAutoLeft3Weave);
        } else if(autonSelection == "Blue Left 3 Bal"){
            autonCommader.initAuton(blueAutoLeft3WeaveBal);
        } else if(autonSelection == "Blue Cord"){
            drivetrain.zero(180, new Pose2d(1.68, 1.65, Rotation2d.fromDegrees(180)));
            Pigeon.zeroSensor(180);
            autonCommader.initAuton(blueCord);
        } else if(autonSelection == "Red Cord"){
            drivetrain.zero(0, new Pose2d(15, 1.65, Rotation2d.fromDegrees(0)));
            Pigeon.zeroSensor(0);
            autonCommader.initAuton(redCord);
        } else if(autonSelection == "New Mid Blue"){
            alliance = Alliance.Blue;
            drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
            autonCommader.initAuton(newMidBlue);
        } else if(autonSelection == "Mid Cube"){
            alliance = Alliance.Blue;
            drivetrain.zero(-90, new Pose2d(0,0, Rotation2d.fromDegrees(-90)));
            Pigeon.zeroSensor(-90);
            autonCommader.initAuton(midCube);
        } else {
            autonCommader.initAuton(ohCrap);
        }

        autonCommader.allaince = alliance;

        // if(alliance == Alliance.Blue){
        //     drivetrain.zero(-90);
        //     autonCommader.auton.reset();
        //     Pigeon.zeroSensor(-90);
        // } else {
        //     drivetrain.zero(90);
        //     autonCommader.auton.reset();
        //     Pigeon.zeroSensor(90);
        // }


        arm.brakeMode();

        // drivetrain.zero(autonCommader.auton.initalAngle, autonCommader.auton.initalPose);
        autonCommader.auton.reset();
        // Pigeon.zeroSensor(autonCommader.auton.initalAngle);

        leds.autonInit();

        // camera.enabled();

        arm.initilizeOffsets();
    }

    @Override
    public void autonomousPeriodic() {
        autonCommader.runAuto();
        pigeon.enabledAction(teleopCommander);
        drivetrain.autonAction(autonCommader);
        arm.action(autonCommader);
        hopper.HopperPeriodic(autonCommader);
        gripper.action(autonCommader);

        leds.autonAction();
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Robot Mode", "Teleop");

        teleopCommander.allaince = alliance;

        if(alliance == Alliance.Red){
            Pigeon.zeroSensor(Pigeon.getAngle()+180);
        }

        if(Camera.getLeftDetecting()){

        }
    
        drivetrain.zero();

        // camera.enabled();

        arm.brakeMode();
        // arm.initilizeOffsets();
    }

    @Override
    
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(teleopCommander);
        arm.action(teleopCommander);
        arm.brakeMode();
        gripper.action(teleopCommander);
        hopper.HopperPeriodic(teleopCommander);

        leds.teleopAction(teleopCommander);
    }
}
