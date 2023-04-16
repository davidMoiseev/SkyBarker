package frc.robot.subsystems;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Extension {
    TalonFX extension;
    private double extensionPosition;
    private boolean achivedTarget = false;
    private static boolean achivedTargetAuto = false;

    public double getExtensionPosition() {
        return extensionPosition;
    }

    public Extension(int canID) {
        extension = new TalonFX(Constants.EXTENSION);
        extension.configFactoryDefault();

        //Configure sensor source for primary PID
        extension.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.EXTENSION_K_PID_LOOP_IDX,
        200);

        extension.setSensorPhase(true);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        extension.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.ARM_TIMEOUT);
        extension.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);

        /* Set the peak and nominal outputs */
        extension.configNominalOutputForward(0, Constants.ARM_TIMEOUT);
        extension.configNominalOutputReverse(0, Constants.ARM_TIMEOUT);
        extension.configPeakOutputForward(1, Constants.ARM_TIMEOUT);
        extension.configPeakOutputReverse(-1, Constants.ARM_TIMEOUT);

        /* Set Motion Magic gains in slot0 - see documentation */
        extension.selectProfileSlot(Constants.EXTENSION_PID_SLOT, Constants.EXTENSION_K_PID_LOOP_IDX);
        extension.config_kF(Constants.EXTENSION_PID_SLOT, Constants.EXTENSION_MOTOR_kF, Constants.ARM_TIMEOUT);
        extension.config_kP(Constants.EXTENSION_PID_SLOT, Constants.EXTENSION_MOTOR_kP, Constants.ARM_TIMEOUT);
        extension.config_kI(Constants.EXTENSION_PID_SLOT, Constants.EXTENSION_MOTOR_kI, Constants.ARM_TIMEOUT);
        extension.config_kD(Constants.EXTENSION_PID_SLOT, Constants.EXTENSION_MOTOR_kD, Constants.ARM_TIMEOUT);

        /* Set acceleration and vcruise velocity - see documentation */
        extension.configMotionCruiseVelocity(Constants.EXTENSION_CRUISEVELOCITY, Constants.ARM_TIMEOUT);
        extension.configMotionAcceleration(Constants.EXTENSION_ACCEL, Constants.ARM_TIMEOUT);

        /* Zero the sensor once on robot boot up */
        extension.setSelectedSensorPosition(0, Constants.EXTENSION_K_PID_LOOP_IDX, Constants.ARM_TIMEOUT);

    }

    private double convertToTicks(double inches) {
        return inches * Constants.FALCON500_TICKS_PER_REV * Constants.EXTENSION_RATIO;
    }
    
    private double convertToInches(double ticks) {
        return ticks / Constants.FALCON500_TICKS_PER_REV / Constants.EXTENSION_RATIO;
    }

    public void updatePose() {
        extensionPosition = this.convertToInches(extension.getSelectedSensorPosition()); 
        achivedTarget = Math.abs(this.convertToInches(extension.getClosedLoopTarget()) - extensionPosition) < 1;
        achivedTargetAuto = Math.abs(this.convertToInches(extension.getClosedLoopTarget()) - extensionPosition) < 3;
        SmartDashboard.putNumber("Extension Closed loop target", this.convertToInches(extension.getClosedLoopTarget()));
        SmartDashboard.putNumber("Extension", extensionPosition);
        HotLogger.Log("Extension Pos", extensionPosition);
    }

    public void setMotorCommand(double motorCommand) {
        SmartDashboard.putNumber("JoyStick Y", motorCommand);
        if (Constants.MAXIMUM_EXTENSION_INCHES > Math.abs(extensionPosition) && Math.abs(motorCommand) > .15) {
            extension.set(ControlMode.PercentOutput, -motorCommand);
        } else {
            extension.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void goToPostion(double inches) {
        if (Constants.MAXIMUM_EXTENSION_INCHES > Math.abs(extensionPosition)) {
            extension.set(ControlMode.MotionMagic, this.convertToTicks(inches));
        } else {
            extension.set(ControlMode.PercentOutput, 0.0);
        }
        HotLogger.Log("Commanded Extension Position", inches);
        SmartDashboard.putNumber("Elbow Angle Command", this.convertToTicks(inches));
        SmartDashboard.putNumber("Elbow Command", inches);
        SmartDashboard.putNumber("Elbow Command Actual", extension.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Elbow FeedForward", extension.getActiveTrajectoryArbFeedFwd()*100);
        SmartDashboard.putNumber("Elbow Proportional", extension.getClosedLoopError()*Constants.ELBOW_MOTOR_kP/1023);
        SmartDashboard.putNumber("Elbow Derviative", extension.getErrorDerivative()*Constants.ELBOW_MOTOR_kD/1023);
        SmartDashboard.putNumber("Elbow Integral", extension.getIntegralAccumulator()*Constants.ELBOW_MOTOR_kI/1023);
        SmartDashboard.putNumber("Elbow Total Command", extension.getMotorOutputPercent());
    }

    public void setCoastMode() {
        extension.setNeutralMode(NeutralMode.Coast);
    }

    public void setBreakMode() {
        extension.setNeutralMode(NeutralMode.Brake);
    }

    public boolean getAchivedTarget() {
        if (extension.getControlMode() == ControlMode.MotionMagic) {
            return achivedTarget;
        } else {
            return false;
        }
    }

    public boolean getAutoAchived(){
        return achivedTargetAuto;
    }
}
