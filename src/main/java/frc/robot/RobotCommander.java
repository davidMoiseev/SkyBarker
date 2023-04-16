package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;

public abstract class RobotCommander {

    double[] intakeArray = {10000, 0};

    public Alliance allaince;

    public XboxController driver;
    public XboxController operator;

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getResetIMU();
    public abstract double armShoulder();
    public abstract double armExtension();
    public abstract Arm.ArmPos getArmPosition();
    public abstract Arm.IntakePos getIntakePosition();
    public abstract Arm.IntakeSpeed getIntakeSpeed();
    public abstract boolean getPickUpObject();
    public abstract boolean getArmReset();
    public abstract double armElbow();
    public abstract boolean hopperOverrideLeft();
    public abstract boolean hopperOverrideRight();
    public abstract boolean getAutoBalance();
    public abstract double getGripperCommand();
    public abstract boolean useNegativeSide();
    public abstract ArmBumpDirection getArmBumpDirection();
    public abstract boolean runHopper();
    public abstract boolean getCubeStopIntake();
    public abstract boolean getCubeMode();
    public abstract double getHopperSpeed();
    public abstract boolean intakeCone();
}
