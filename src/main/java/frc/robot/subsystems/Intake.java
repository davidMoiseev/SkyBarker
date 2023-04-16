// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.RobotCommander;
// import frc.robot.TeleopCommander;
// import frc.robot.subsystems.Arm.IntakePos;
// import frc.robot.subsystems.Arm.IntakeSpeed;

// import static frc.robot.Constants.*;

// public class Intake {
//     double speed1;
//     double speed2;
//     double angle;
//     TalonFX speedMotor1;
//     TalonFX speedMotor2;
//     CANSparkMax angleMotor;
//     CANCoder angleEncoder;
//     PIDController pidController;
//     private IntakePos determineIntakePosReading = IntakePos.none;

//     public static double angleEncoderAngle;

//     public static double startPosition;

//     public Intake(){


//         angle = 45;
//         speed1 = 0;
//         speed2 = 0;
//         speedMotor1 = new TalonFX(INTAKE_SPEED1_MOTOR_ID);
//         speedMotor2 = new TalonFX(INTAKE_SPEED2_MOTOR_ID);
//         angleMotor = new CANSparkMax(INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
//         angleEncoder = new CANCoder(23);
//         pidController = new PIDController(.0175, 0.0015, 0.000075); //.0175, 0.001, 0.000075
//         startPosition = angleEncoder.getAbsolutePosition();
//         angleEncoder.configSensorDirection(true);
//         angleEncoder.configMagnetOffset(INTAKE_OFFSET);

//         // speedMotor2.setInverted(TalonFXInvertType.OpposeMaster);
//         // speedMotor2.follow(speedMotor1);
//     }

//     public void setBrakeMode(){
//         // angleMotor.setIdleMode(IdleMode.kBrake);
//         angleMotor.setIdleMode(IdleMode.kCoast);
//     }

//     public void setCoastMode(){
//         angleMotor.setIdleMode(IdleMode.kCoast);
//     }

//     public void IntakePeriodic(IntakePos posIntake, IntakeSpeed intakeSpeed){
//         SmartDashboard.putNumber("Intake Desired Angle", posIntake.getPositionReading());
//         // SmartDashboard.putNumber("Intake Desired Speed", posIntake.getSpeedReading1());
//         speedPeriodic(intakeSpeed);
//         anglePeriodic(posIntake);
//     }
    
//     public void speedPeriodic(IntakeSpeed intakeSpeed){
//         speed1 = intakeSpeed.getSpeedReading1();
//         speed2 = intakeSpeed.getSpeedReading2();
//         // SmartDashboard.putNumber("Speed", speed);
//         speedMotor1.set(TalonFXControlMode.PercentOutput, speed1);
//         speedMotor2.set(TalonFXControlMode.PercentOutput, speed2);
//     }

//     public void anglePeriodic(IntakePos posIntake){
//         if(posIntake.getPositionReading() == 0){
//             angleMotor.set(0);
//         } else {
//             angle = posIntake.getPositionReading();
//             double ourAngle = angleEncoder.getAbsolutePosition();
//             double change = pidController.calculate(ourAngle, angle);
//             SmartDashboard.putNumber("Change", change);
//             if(Math.abs(ourAngle - angle) > .2){
//                 angleMotor.set(change);
//             } else {
//                 angleMotor.set(0);
//             }
//         }

//         SmartDashboard.putNumber("Desired Intake Angle", posIntake.getPositionReading());

//     } 

//     public void logData() {
//         angleEncoderAngle = angleEncoder.getAbsolutePosition();
//         SmartDashboard.putNumber("Intake Absolute Encoder", angleEncoderAngle);
//         // SmartDashboard.putNumber("magnet offset", angleEncoder.getAllConfigs(CANCoderConfiguration));
//     }
// }

package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_OFFSET;
import static frc.robot.Constants.id;
import static frc.robot.Constants.ierror;
import static frc.robot.Constants.iff;
import static frc.robot.Constants.ii;
import static frc.robot.Constants.iiz;
import static frc.robot.Constants.imax;
import static frc.robot.Constants.imaxacc;
import static frc.robot.Constants.imaxrpm;
import static frc.robot.Constants.imaxvel;
import static frc.robot.Constants.imin;
import static frc.robot.Constants.iminvel;
import static frc.robot.Constants.ip;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotCommander;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;

public class Intake {
    int tick = 0;
    double speed;
    double angle;
    TalonFX speedMotor1;
    TalonFX speedMotor2;
    CANSparkMax angleMotor;
    CANCoder angleEncoder;
    PIDController pidController;
    boolean anglecheck = false;
    double prevangle;
    SparkMaxPIDController smartMotion;
    double pidchange;
    double change;
    public static double angleEncoderAngle;
    boolean intakeMode;
    DigitalInput ballSensor;
    public static double startPosition;
    public IntakePos intakePos;
    public IntakeSpeed intakeSpeed;
    public RobotCommander robotCommander;

    public Intake(){
        intakePos = IntakePos.none;
        intakeSpeed = IntakeSpeed.none;

        angle = 0;
        speed = 0;
        ballSensor= new DigitalInput(INTAKE_CUBE_SENSOR);
        speedMotor1 = new TalonFX(INTAKE_SPEED1_MOTOR_ID);
        speedMotor2 = new TalonFX(INTAKE_SPEED2_MOTOR_ID);
        speedMotor2.setNeutralMode(NeutralMode.Brake);
        angleMotor = new CANSparkMax(INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleEncoder = new CANCoder(INTAKE_CANCODER);
        pidController = new PIDController(ip, ii, id); //.017, 0.0015, 0
        startPosition = angleEncoder.getAbsolutePosition();
        smartMotion = angleMotor.getPIDController();
        angleEncoder.configMagnetOffset(INTAKE_OFFSET);
        angleMotor.getEncoder().setPosition(0);
        change = 0;
        pidchange = 0;
    }

    public void setBrakeMode(){
        angleMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void IntakePeriodic(IntakePos pIntakePos, IntakeSpeed pIntakeSpeed, RobotCommander commander){
        // SmartDashboard.putNumber("Intake Desired Angle", commander.getIntakePosition()[0]);
        // SmartDashboard.putNumber("Intake Desired Speed", commander.getIntakePosition()[1]);
        intakePos = pIntakePos;
        intakeSpeed = pIntakeSpeed;
        robotCommander = commander;

        speedPeriodic();
        anglePeriodic();

        SmartDashboard.putBoolean("_intake Sensor", ballSensor.get());
    }
    
    public void speedPeriodic(){
        angle = angleEncoder.getAbsolutePosition();
        speed = intakeSpeed.getSpeedReading1();
        intakeMode = robotCommander.getCubeMode(); //defaults to cone mode
        SmartDashboard.putNumber("Speed", speed);

        // if (ballSensor.get() && tick < INTAKE_DELAY && robotCommander.getCubeStopIntake()) {
        //     tick += 1;
        // } else if (ballSensor.get() && (tick == INTAKE_DELAY) && (speed > 0) && robotCommander.getCubeStopIntake()) {
        //     speed = 0;
        // } else if (!ballSensor.get()){
        //     tick = 0;
        // }

        SmartDashboard.putBoolean("_Breaking Things", robotCommander.getCubeStopIntake());

        if(intakePos == IntakePos.cubeHandoff && Math.abs(angle - intakePos.getPositionReading()) < 10){
            // speedMotor1.set(TalonFXControlMode.PercentOutput, -1);
            speedMotor2.set(TalonFXControlMode.PercentOutput, 1);
        } else if(!ballSensor.get() && intakeSpeed != IntakeSpeed.out && intakeSpeed != IntakeSpeed.autoOut){
            if(tick < INTAKE_DELAY){
                speedMotor1.set(TalonFXControlMode.PercentOutput, speed);
                speedMotor2.set(TalonFXControlMode.PercentOutput, -speed);
                tick += 1;
            } else {
                speedMotor1.set(TalonFXControlMode.PercentOutput, 0);
                speedMotor2.set(TalonFXControlMode.PercentOutput, 0);
            }
        } else {
            if(robotCommander.intakeCone()){
                if(robotCommander.getIntakeSpeed() == IntakeSpeed.out){
                    speedMotor1.set(TalonFXControlMode.PercentOutput, speed);
                    speedMotor2.set(TalonFXControlMode.PercentOutput, speed);
                } else {
                    speedMotor1.set(TalonFXControlMode.PercentOutput, speed);
                    speedMotor2.set(TalonFXControlMode.PercentOutput, speed);
                }
                
            } else {
                if(robotCommander.getIntakeSpeed() == IntakeSpeed.out){
                    // speedMotor1.set(TalonFXControlMode.PercentOutput, speed);
                    speedMotor2.set(TalonFXControlMode.PercentOutput, -speed);
                } else {
                    speedMotor1.set(TalonFXControlMode.PercentOutput, speed);
                    speedMotor2.set(TalonFXControlMode.PercentOutput, -speed);
                }

            }
            
            tick = 0;
        }

    }

    public void anglePeriodic(){

        if(intakePos == IntakePos.none){
            angleMotor.set(0);
        } else {
            angle = intakePos.getPositionReading();
            double ourAngle = angleEncoder.getAbsolutePosition();
            
            if (prevangle != angle) {
                anglecheck = false;
                
            }
            if((Math.abs(ourAngle - angle) > INTAKE_DEADBAND) && !anglecheck){
                change = ourAngle - angle;
                        if (change > 0) {
                            angleMotor.set(-INTAKE_ANGLE_MAX_SPEED);
                        } else if (change < 0) {
                            angleMotor.set(INTAKE_ANGLE_MAX_SPEED);
                        }    
            } else if (((Math.abs(ourAngle - angle) > INTAKE_PID_DEADBAND) && (Math.abs(ourAngle - angle) < INTAKE_DEADBAND)) && !anglecheck) {

                pidchange = pidController.calculate(ourAngle, angle);
                if (pidchange > INTAKE_ANGLE_MAX_SPEED) {
                    pidchange = INTAKE_ANGLE_MAX_SPEED;
                } else if (pidchange < -INTAKE_ANGLE_MAX_SPEED) {
                    pidchange = -INTAKE_ANGLE_MAX_SPEED;
                }
                angleMotor.set(pidchange);

            }else {

                angleMotor.set(0);
                anglecheck = true;

            }

            prevangle = angle;
        }
        
    }


    public void logData() {
        angleEncoderAngle = angleEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Intake Change", change);
        SmartDashboard.putNumber("Intake PID Change", pidchange);
        SmartDashboard.putNumber("Intake Absolute Encoder", angleEncoderAngle);
        SmartDashboard.putNumber("Intake rotations", angleMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Angle Power", angleMotor.getAppliedOutput());
        SmartDashboard.putBoolean("intake Sensor", ballSensor.get());
        // SmartDashboard.putNumber("magnet offset", angleEncoder.getAllConfigs(CANCoderConfiguration));
    }
}