package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPos;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;

import static frc.robot.Constants.*;


public class TeleopCommander extends RobotCommander{

    // public XboxController driver;
    // public  XboxController operator;
    private static boolean Bumpercheck = false;
    
    private boolean manualMode = false;

    private boolean cubeMode = false; // true ball, false cone

    private boolean slowSpeed = false;

    public TeleopCommander() {
        driver = new XboxController(0);
        operator = new XboxController(1);
    }

    @Override
    public double getForwardCommand() {
        if(!getDriverSlowSpeed()){
            return -(modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND);
        } else {
            return -(modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND) * SLOW_SPEED_MULTIPLIER;
        }
    }

    @Override
    public double getStrafeCommand() {
        if(!getDriverSlowSpeed()){
            return -(modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND);
        } else {
            return -(modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND) * SLOW_SPEED_MULTIPLIER;
        }
    }

//     if(driver.getRightTriggerAxis()){
//         gripperMotorCommand = GRIPPER_HOLD_POWER + driver.getRightTriggerAxis();

//     }
// } else {
//     if(driver.getRightTriggerAxis() > .1){
//         gripperMotorCommand  = driver.getRightTriggerAxis();
//     } else {
//         gripperMotorCommand =  operator.getLeftY();

//     }
// }

    @Override
    public double getTurnCommand() {
        double value = deadband(Math.abs(driver.getRightX()) * driver.getRightX(), 0.01, 0.75) * (MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        
        if(!getDriverSlowSpeed()){
            return -value;
        } else {
            return -value * .5 * SLOW_SPEED_MULTIPLIER;
        }
    }

    @Override
    public boolean getResetIMU() {
        return driver.getBackButton();
    }

    public boolean getDriveToScoring(){
        return driver.getBButton();
    }

    private double deadband(double value, double deadband, double maxRange){
        if(Math.abs(value) < deadband){
            return 0;
        } else if (value < 0) {
            return  ((value + deadband)/(1.0 - deadband)) * maxRange;
        } else {
            return  ((value - deadband)/(1.0 - deadband)) * maxRange;
        }
    }
  
    private double modifyAxis(double value) {
        boolean deadband = 0.13 > Math.sqrt(Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2));

        if (deadband) {
            return 0;
        } else {
            return Math.abs(value) * value;
        }
    }
    
    public boolean getDriverSlowSpeed(){
        if(driver.getLeftBumper()){
            slowSpeed = true;
        } else if(driver.getRightBumper()){
            slowSpeed = false;
        }

        return slowSpeed;
    }

    IntakePos intakePos = IntakePos.none;
    IntakeSpeed intakeSpeed = IntakeSpeed.none;

    public IntakeSpeed getIntakeSpeed(){
        if(operator.getRightTriggerAxis() > .3){
            if(intakeCone()){
                intakeSpeed = IntakeSpeed.onCone;
            } else {
                intakeSpeed = IntakeSpeed.onCube;
            }
        } else if (operator.getLeftTriggerAxis() > .3){
            intakeSpeed = IntakeSpeed.out;
        } else {
            intakeSpeed = IntakeSpeed.none;
        }

        return intakeSpeed;
    }

    public IntakePos getIntakePosition() {
        boolean Dpad_right = (operator.getPOV() > 70 && operator.getPOV() < 110);
        boolean Dpad_left = (operator.getPOV() > 250 && operator.getPOV() < 290);
        boolean Dpad_updown = false;
        boolean Trigger_right = (operator.getRightTriggerAxis() > .3);
        boolean Trigger_left = (operator.getLeftTriggerAxis() > .3);
            if (!this.getManualMode()) {
                if(getArmPosition() == ArmPos.humanPlayerPickup ||
                    getArmPosition() == ArmPos.humanPlayerReady){
                        intakePos = IntakePos.humanIntake;
                    }
                else if (getArmPosition() != ArmPos.Zero && 
                    getArmPosition() != ArmPos.manual && 
                    getArmPosition() != ArmPos.intake && 
                    Intake.angleEncoderAngle < 130) { 
                        intakeArray[0] = 102;
                        intakePos = IntakePos.armMoving;
                } else if(intakeCone()) {
                    if(Dpad_right){
                        intakePos = IntakePos.collectCone;
                    }
                } else {
                    if (Dpad_left) {
                        intakePos = IntakePos.pack;
                    } else if (Dpad_right) {
                        intakePos = IntakePos.collectCube;
                    } else if (Dpad_updown) {
                        intakePos = IntakePos.station;
                    } else if (operator.getPOV() == 180) {
                        intakePos = IntakePos.cubeHandoff;
                    } else {
                        intakePos = IntakePos.none;
                    }
                }
            }

        return intakePos;
    }

    public boolean getArmReset(){
        return operator.getBackButton();
    }

    public boolean getCubeStopIntake(){
        return false;
    }

    public boolean getManualMode(){
        if(!manualMode && operator.getStartButton()){
            manualMode = true;
        } else if (manualMode && operator.getBackButton()){
            manualMode = false;
        }
        return manualMode;
    } 
    
    public boolean resetModules(){
        return driver.getStartButton();
    }

    public ArmPos getArmPosition(){
        if (this.getManualMode()) {
            return ArmPos.manual;
        } else if (operator.getLeftBumper()) { //operator.getPOV() == 90
            return ArmPos.intake;
        } else if (operator.getPOV() == 0) {
            return ArmPos.lowerNode;
        } else if(operator.getPOV() == 180){
            return ArmPos.intake;
            
        }else if(operator.getYButton()){
            if(getCubeMode()){
                return ArmPos.topNodeCube;
            } else {
                return ArmPos.topNodeCone;
            }
        } else if(operator.getBButton()){
            if(getCubeMode()){
                return ArmPos.middleNodeCube;
            } else {
                return ArmPos.middleNodeCone;
            }
            // return ArmPos.yeetCube;
            // return ArmPos.groundGripperCone;
        } else if(operator.getAButton() || driver.getXButton()) {
            return ArmPos.packagePos;
        }else if(operator.getXButton()){
            if (operator.getRightBumper()){
                return ArmPos.humanPlayerPickup;
            } else {
                return ArmPos.humanPlayerReady;
            }
        } else {
            return ArmPos.Zero;
        }
    }

    public ArmBumpDirection getArmBumpDirection() {
        if (driver.getBButton()) {
            return ArmBumpDirection.bumpUp;
        } else if (driver.getAButton()) {
            return ArmBumpDirection.bumpDown;
        } else {
            return ArmBumpDirection.bumpZero;
        }
    }

    public boolean getCubeMode(){
        // if(operator.getLeftStickButton()){
        //     cubeMode = true;
        // } else if(operator.getRightStickButton()){
        //     cubeMode = false;
        // }

        if(operator.getLeftStickButton()){
            cubeMode = true;
        } else {
            cubeMode = false;
        }

        return cubeMode;
    }

    public double armShoulder(){     
        if(Math.abs(operator.getRightX()) > 0.1){
            return operator.getRightX() * 0.8;
        } else {
            return 0;
        }
    }

    public double armExtension(){
        if (!operator.getLeftBumper() && operator.getRightBumper()){
            return -.2;
        } else if (operator.getLeftBumper() && !operator.getRightBumper()) {
            return .2;
        } else {
            return 0;
        }
    }

    public double armElbow() {
        if(Math.abs(operator.getLeftX()) > 0.2){
            return operator.getLeftX() * 0.5;
        } else {
            return 0;
        }
    }

    @Override
    public boolean getPickUpObject() {
        // TODO Auto-generated method stub
        return driver.getAButton();
    }

    @Override
    public boolean hopperOverrideLeft() {
        return operator.getXButton();
    }

    @Override
    public boolean hopperOverrideRight() {
        return operator.getBButton();
    }

    @Override
    public boolean getAutoBalance() {
        // TODO Auto-generated method stub
        return false;
    }

    private double overrideGripper(){
        if (driver.getRightTriggerAxis() > .15) {
            return .8;
        } else if (driver.getYButton()) {
            return -8;
        } else {
            return operator.getLeftY();
        }
    }

    public double getGripperCommand() {
        double gripperMotorCommand = 0.0;
        if (this.getArmPosition() == ArmPos.lowerNode || this.getArmPosition() == ArmPos.topNodeCone || this.getArmPosition() == ArmPos.middleNodeCone ||
        this.getArmPosition() == ArmPos.topNodeCube || this.getArmPosition() == ArmPos.middleNodeCube) {
            gripperMotorCommand = GRIPPER_HOLD_POWER + overrideGripper();
        } else if(this.getArmPosition() == ArmPos.humanPlayerPickup ){
            gripperMotorCommand = -1;
        } else if (this.getArmPosition() == ArmPos.groundGripperConePick ){
            gripperMotorCommand = -1;
        } else{
            gripperMotorCommand = overrideGripper();
        }

        return gripperMotorCommand;
    }

    @Override
    public boolean useNegativeSide() {
        double angle =MathUtil.inputModulus(Pigeon.getAngle(),-180,180);
        if (angle > -180 && angle < 0) {
            return true;
        } else  {
            return false;
        }
    }
    
    public boolean xReleased(){
        if((operator.getXButtonReleased() == true) && (operator.getRightBumperReleased() == true)){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public boolean runHopper() {
        // TODO Auto-generated method stub
        return false;
    }

    public boolean getAutoLine() {
        return false;
    }

    @Override
    public double getHopperSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean intakeCone() {
        // TODO Auto-generated method stub
        return operator.getRightStickButton();
    }

    public boolean autoAim(){
        return driver.getLeftTriggerAxis() > .4;
    }
    
}