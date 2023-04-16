package frc.robot.subsystems;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;
import frc.robot.subsystems.Arm.ArmPos;

public class Shoulder {

    
    private TalonFX shoulder;
    private CANCoder shoulderEncoder;
    private double shoulderAngle;
    private boolean achivedTarget = false;
    private static boolean achivedTargetAuto = false;

    public double getShoulderAngle() {
        return shoulderAngle;
    }

    public Shoulder(int motorCANID, int canCODERCANID) {
        
        shoulder = new TalonFX(motorCANID);
        shoulder.configFactoryDefault();
        shoulderEncoder = new CANCoder(canCODERCANID);
        shoulderEncoder.configFactoryDefault();
        shoulderEncoder.setPositionToAbsolute();
        shoulderEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        shoulderEncoder.configMagnetOffset(Constants.SHOULDER_OFFSET);


        //Configure sensor source for primary PID
        shoulder.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.SHOULDER_K_PID_LOOP_IDX,
        Constants.ARM_TIMEOUT);   

        shoulder.setSensorPhase(true);
        shoulder.setInverted(false);
        shoulderEncoder.configSensorDirection(true);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.ARM_TIMEOUT);
        shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);

        /* Set the peak and nominal outputs */
        shoulder.configNominalOutputForward(0, Constants.ARM_TIMEOUT);
        shoulder.configNominalOutputReverse(0, Constants.ARM_TIMEOUT);
        shoulder.configPeakOutputForward(1, Constants.ARM_TIMEOUT);
        shoulder.configPeakOutputReverse(-1, Constants.ARM_TIMEOUT);

        /* Set Motion Magic gains in slot0 - see documentation */
        shoulder.selectProfileSlot(Constants.SHOULDER_PID_SLOT, Constants.SHOULDER_K_PID_LOOP_IDX);
        shoulder.config_kF(Constants.SHOULDER_PID_SLOT, Constants.SHOULDER_MOTOR_kF, Constants.ARM_TIMEOUT);
        shoulder.config_kP(Constants.SHOULDER_PID_SLOT, Constants.SHOULDER_MOTOR_kP, Constants.ARM_TIMEOUT);
        shoulder.config_kI(Constants.SHOULDER_PID_SLOT, Constants.SHOULDER_MOTOR_kI, Constants.ARM_TIMEOUT);
        shoulder.config_kD(Constants.SHOULDER_PID_SLOT, Constants.SHOULDER_MOTOR_kD, Constants.ARM_TIMEOUT);

        /* Set acceleration and vcruise velocity - see documentation */
        shoulder.configMotionCruiseVelocity(Constants.SHOULDER_CRUISEVELOCITY, Constants.ARM_TIMEOUT);
        shoulder.configMotionAcceleration(Constants.SHOULDER_ACCEL, Constants.ARM_TIMEOUT);

        /* Zero the sensor once on robot boot up */
        this.intilizeOffset();
    }

    public void intilizeOffset() {
        shoulder.setSelectedSensorPosition(this.convertToTicks(shoulderEncoder.getAbsolutePosition()));
    }

    private double convertToTicks(double degrees) {
        return degrees * Constants.FALCON500_TICKS_PER_REV * Constants.SHOULDER_RATIO;
    }
    
    private double convertToDegrees(double ticks) {
        return ticks / Constants.FALCON500_TICKS_PER_REV / Constants.SHOULDER_RATIO;
    }

    public void updatePose() {
        shoulderAngle = this.convertToDegrees(shoulder.getSelectedSensorPosition());
        achivedTarget = Math.abs(this.convertToDegrees(shoulder.getClosedLoopTarget()) - shoulderAngle) < 5;
        achivedTargetAuto = Math.abs(this.convertToDegrees(shoulder.getClosedLoopTarget()) - shoulderAngle) < 2.5;
        SmartDashboard.putNumber("Shoulder Angle", shoulderAngle);
        SmartDashboard.putNumber("Shoulder Closed loop target", this.convertToDegrees(shoulder.getClosedLoopTarget()));
        SmartDashboard.putNumber("Shoulder Angle CANCODER", shoulderEncoder.getAbsolutePosition());
        HotLogger.Log("Shoulder Pos",shoulderAngle);
    }

    public void setMotorCommand(double motorCommand) { 
        if (Math.abs(motorCommand) > .15) {
            shoulder.set(ControlMode.PercentOutput, motorCommand);
        } else {
            shoulder.set(ControlMode.PercentOutput, 0.0);
        }
        SmartDashboard.putNumber("Shoulder Total Command", shoulder.getMotorOutputPercent());
    }

    public void goToPostion(RobotCommander commander, double degrees) {
        if(commander.getArmPosition() == ArmPos.humanPlayerReady || 
        commander.getArmPosition() == ArmPos.humanPlayerPickup){
            shoulder.configMotionAcceleration(6000 * 16, Constants.ARM_TIMEOUT);
        } else {
            shoulder.configMotionAcceleration(Constants.SHOULDER_ACCEL, Constants.ARM_TIMEOUT);
        }

        if (Math.abs(degrees) < 115) {
            shoulder.set(ControlMode.MotionMagic, this.convertToTicks(degrees), DemandType.ArbitraryFeedForward, 0.0*Math.cos(Math.toRadians(shoulderAngle)));
        } else {
            shoulder.set(ControlMode.PercentOutput, 0.0);
        }
        SmartDashboard.putNumber("Should command ticks", this.convertToTicks(degrees));
        SmartDashboard.putNumber("Should ticks", shoulder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Shoulder Command", degrees);
        SmartDashboard.putNumber("Shoulder Command Actual", this.convertToDegrees(shoulder.getActiveTrajectoryPosition()));
        SmartDashboard.putNumber("Shoulder FeedForward", shoulder.getActiveTrajectoryArbFeedFwd()*100);
        SmartDashboard.putNumber("Shoulder Proportional", shoulder.getClosedLoopError()*Constants.SHOULDER_MOTOR_kP/1023);
        SmartDashboard.putNumber("Shoulder Derviative", shoulder.getErrorDerivative()*Constants.SHOULDER_MOTOR_kD/1023);
        SmartDashboard.putNumber("Shoulder Integral", shoulder.getIntegralAccumulator()*Constants.SHOULDER_MOTOR_kI/1023);
        SmartDashboard.putNumber("Shoulder Total Command", shoulder.getMotorOutputPercent());
    }

    public void setCoastMode() {
        shoulder.setNeutralMode(NeutralMode.Coast);
    }

    public void setBreakMode() {
        shoulder.setNeutralMode(NeutralMode.Brake);
    }

    public boolean getAchivedTarget() {
        if (shoulder.getControlMode() == ControlMode.MotionMagic) {
            return achivedTarget;
        } else {
            return false;
        }
    }

    public boolean getAutoAchived(){
        return achivedTargetAuto;
    }
}
