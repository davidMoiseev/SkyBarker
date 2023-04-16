package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Autons.AutonBase;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPos;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;

public class AutonCommader extends RobotCommander{

    AutonBase auton;

    public void initAuton(AutonBase passedAuton){
        auton = passedAuton;
    }

    public void runAuto(){
        auton.runAuto();
    }

    @Override
    public double getForwardCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getStrafeCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getTurnCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getResetIMU() {
        // TODO Auto-generated method stub
        // return auton.getResetIMU;
        return false;
    }

    public State getDesiredState(){
        return auton.desState;
    }

    public Rotation2d getTargetTheta(){
        return auton.targetTheta;
    }

    public Pose2d getInitalPose(){
        return auton.initalPose;
    }

    @Override
    public double armShoulder() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double armExtension() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public ArmPos getArmPosition() {
        // TODO Auto-generated method stub
        return auton.armPos;
    }


    @Override
    public boolean getPickUpObject() {
        // TODO Auto-generated method stub
        return auton.pickUpObject;
    }

    @Override
    public boolean getArmReset() {
        // TODO Auto-generated method stub
        return false;
    }
    
    public boolean isDriving(){
        return auton.driving;
    }

    @Override
    public double armElbow() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean hopperOverrideLeft() {
        // No hopper override in auton
        return false;
    }

    @Override
    public boolean hopperOverrideRight() {
        // No hopper override in auton
        return false;
    }

    @Override
    public boolean getAutoBalance() {
        // TODO Auto-generated method stub
        return auton.autoBalance;
    }
    
    public double getGripperCommand() {
        // TODO Auto-generated method stub
        return auton.gripperSpeed;
    }

    // @Override
    // public boolean useNegativeSide() {
    //     // TODO Auto-generated method stub
    //     return false;
    // }

    public ArmBumpDirection getArmBumpDirection() {
        return ArmBumpDirection.bumpZero;
    }

    @Override
    public boolean useNegativeSide() {
        double angle =MathUtil.inputModulus(Pigeon.getAngle(),-180,180);
        if(allaince == Alliance.Red){
            if (angle > -180 && angle < 0) {
                return false;
            } else  {
                return true;
            }
        } else {
            if (angle > -180 && angle < 0) {
                return true;
            } else  {
                return false;
            }
        }

    }

    public boolean overrideNegSide(){
        return auton.overrideNegSide;
    }

    public boolean getXMode(){
        return auton.xMode;
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

    @Override
    public IntakePos getIntakePosition() {
        // if(auton.overrideIntake){
        //     return auton.intakePos;
        // } else {
            if ((getArmPosition() != ArmPos.Zero && 
            getArmPosition() != ArmPos.manual && 
            getArmPosition() != ArmPos.intake && 
            Intake.angleEncoderAngle < 130)) { 
                return IntakePos.armMoving;
            } else {
                return auton.intakePos;
            }
        // }
    }

    @Override
    public IntakeSpeed getIntakeSpeed() {
        // TODO Auto-generated method stub
        return auton.intakeSpeed;
    }

    @Override
    public boolean getCubeStopIntake() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getCubeMode() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getHopperSpeed() {
        // TODO Auto-generated method stub
        return auton.hopperSpeed;
    }

    public boolean getNewAutoBal(){
        return auton.newAutoBal;
    }

    @Override
    public boolean intakeCone() {
        // TODO Auto-generated method stub
        return auton.coneIntake;
    }
}
