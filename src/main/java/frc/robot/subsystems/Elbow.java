package frc.robot.subsystems;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elbow {
    private VictorSPX elbow;
    private CANCoder elbowEncoder;
    private double elbowAngle;

    public double getElbowAngle() {
        return elbowAngle;
    }

    private double elbowOffset;
    private boolean achivedTarget = false;
    private static boolean achivedTargetAuto = false;

    public Elbow (int motorCANID, int encoderCANID) {
        
        elbow = new VictorSPX(motorCANID);
        elbow.configFactoryDefault();
        elbowEncoder = new CANCoder(encoderCANID);
        elbowEncoder.configFactoryDefault();
        elbowEncoder.setPositionToAbsolute();
        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        elbowEncoder.configMagnetOffset(Constants.ELBOW_OFFSET);

        //Configure sensor source for primary PID
        elbow.configRemoteFeedbackFilter(elbowEncoder, 0);
        
        elbow.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Constants.ELBOW_K_PID_LOOP_IDX,
        200);   

        
        elbow.setSensorPhase(true);
        elbow.setInverted(true);
        
        /* Set relevant frame periods to be at least as fast as periodic rate */
        // elbow.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ARM_TIMEOUT);
        // elbow.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);
        
        /* Set the peak and nominal outputs */
        elbow.configNominalOutputForward(0, Constants.ARM_TIMEOUT);
        elbow.configNominalOutputReverse(0, Constants.ARM_TIMEOUT);
        elbow.configPeakOutputForward(1, Constants.ARM_TIMEOUT);
        elbow.configPeakOutputReverse(-1, Constants.ARM_TIMEOUT);
        
        /* Set Motion Magic gains in slot0 - see documentation */
        elbow.selectProfileSlot(Constants.ELBOW_PID_SLOT, Constants.ELBOW_K_PID_LOOP_IDX);
        elbow.config_kF(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kF, Constants.ARM_TIMEOUT);
        elbow.config_kP(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kP, Constants.ARM_TIMEOUT);
        elbow.config_kI(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kI, Constants.ARM_TIMEOUT);
        elbow.config_kD(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kD, Constants.ARM_TIMEOUT);
        elbow.config_IntegralZone(Constants.ELBOW_PID_SLOT, this.convertToTicks(Constants.ELBOW_MOTOR_kI_ZONE));

        /* Set acceleration and vcruise velocity - see documentation */
        elbow.configMotionCruiseVelocity(Constants.ELBOW_CRUIESVELOCITY, Constants.ARM_TIMEOUT);
        elbow.configMotionAcceleration(Constants.ELBOW_ACCEL, Constants.ARM_TIMEOUT);

        this.intilizeOffset();
    }

    public void intilizeOffset() {
        elbowOffset = elbow.getSelectedSensorPosition() - elbowEncoder.getAbsolutePosition()*4096/360;
    }

    private double convertToTicks(double degrees) {
        return degrees / 360 * 4096;
    }
    
    private double convertToDegrees(double ticks) {
        return ticks * 360 / 4096;
    }

    public void updatePose() {
        elbowAngle = (elbow.getSelectedSensorPosition(Constants.ELBOW_K_PID_LOOP_IDX) - elbowOffset)*360/4096; 
        achivedTarget = Math.abs(this.convertToDegrees(elbow.getClosedLoopTarget() - elbowOffset) - elbowAngle) < 10;
        achivedTargetAuto = Math.abs(this.convertToDegrees(elbow.getClosedLoopTarget() - elbowOffset) - elbowAngle) < 30;
        SmartDashboard.putNumber("Elbow Angle", elbowAngle);
        SmartDashboard.putNumber("Elbow elbowOffset", elbowOffset);
        SmartDashboard.putNumber("Elbow Closed loop target", this.convertToDegrees(elbow.getClosedLoopTarget() - elbowOffset));
        SmartDashboard.putNumber("Elbow real angle", elbow.getSelectedSensorPosition(Constants.ELBOW_K_PID_LOOP_IDX));
        SmartDashboard.putNumber("Elbow Angle CANCODER", elbowEncoder.getAbsolutePosition());
        HotLogger.Log("Extension Pos",elbowAngle);
    }

    public void setMotorCommand(double motorCommand) { 
        if (Math.abs(motorCommand) > .15) {
            elbow.set(ControlMode.PercentOutput, motorCommand);
        } else {
            elbow.set(ControlMode.PercentOutput, 0.0);
        }
        SmartDashboard.putNumber("Elbow Total Command", elbow.getMotorOutputPercent());
    }

    public void goToPostion(double degrees) {
        if (degrees > -190 && degrees < 190) {
            elbow.set(ControlMode.MotionMagic, elbowOffset + this.convertToTicks(degrees), DemandType.ArbitraryFeedForward, .225*Math.sin(Math.toRadians(elbowAngle)));
        } else {
            elbow.set(ControlMode.PercentOutput, 0.0);
        }
        SmartDashboard.putNumber("Elbow Angle Command", elbowOffset + this.convertToTicks(degrees));
        SmartDashboard.putNumber("Elbow Command", degrees);
        SmartDashboard.putNumber("Elbow Command Actual", this.convertToDegrees(elbow.getActiveTrajectoryPosition()- elbowOffset)*360/4096);
        SmartDashboard.putNumber("Elbow FeedForward", elbow.getActiveTrajectoryArbFeedFwd()*100);
        SmartDashboard.putNumber("Elbow Proportional", elbow.getClosedLoopError()*Constants.ELBOW_MOTOR_kP/1023);
        SmartDashboard.putNumber("Elbow Derviative", elbow.getErrorDerivative()*Constants.ELBOW_MOTOR_kD/1023);
        SmartDashboard.putNumber("Elbow Integral", elbow.getIntegralAccumulator()*Constants.ELBOW_MOTOR_kI/1023);
        SmartDashboard.putNumber("Elbow Total Command", elbow.getMotorOutputPercent());
    }

    public void setCoastMode() {
        elbow.setNeutralMode(NeutralMode.Coast);
    }

    public void setBreakMode() {
        elbow.setNeutralMode(NeutralMode.Brake);
    }

    public boolean getAchivedTarget() {
        if (elbow.getControlMode() == ControlMode.MotionMagic) {
            return achivedTarget;
        } else {
            return false;
        }
    }

    public boolean getAutoAchived(){
        return achivedTargetAuto;
    }
}
