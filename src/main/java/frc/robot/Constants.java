// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// Swerve Bot Constants
public final class Constants {
    public static final boolean realBot = true;
    public static final boolean compBot = true;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;

    public static final double MAX_VOLTAGE = 12.0; 

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final int DRIVETRAIN_PIGEON_ID = 24;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 18;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = compBot ? -Math.toRadians(236.37 + 180) : -Math.toRadians(349.365234375-180); // 169.1015625

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 19;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = compBot ? -Math.toRadians(35.27 + 180) : -Math.toRadians(3.779296875+180); //183.251953125

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 20;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = compBot ? -Math.toRadians(243.57 - 180) : -Math.toRadians(308.935546875-180); // 129.814453125

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = compBot ? -Math.toRadians(88.41796875+90) : -Math.toRadians(55.72265625+180); //237.48046875 //340.87 - 180

    public static final double SLOW_SPEED_MULTIPLIER = .5;

    public static final int INTAKE_SPEED1_MOTOR_ID = 9;
    public static final int INTAKE_SPEED2_MOTOR_ID = 12;
    public static final int INTAKE_ANGLE_MOTOR_ID = 10;
    public static final double INTAKE_PACKAGE_POSITION = 90.5;
    public static final double INTAKE_STATION_POSITION = 121;
    public static final double INTAKE_COLLECT_POSITION = 168; //169
    public static final double INTAKE_SPEED_CUBE = .5;
    public static final double INTAKE_SPEED_CONE = 1;
    public static final int INTAKE_TIMEOUT = 200;

    public static final double INTAKE_OFFSET = compBot ? -8.3 : 52.5;
    
    // Hopper CAN ID
    public static final int HOPPER_MOTOR = 11;
    public static final int FINGER_SERVO = 2;
    public static final int LEFT_SENSOR = 0;
    public static final int TOP_SENSOR = 1;
    public static final int BOTTOM_SENSOR = 2;
    public static final int RIGHT_SENSOR = 3;
    public static final double HOPPER_OVERRIDE_SPEED = 1;
    public static final double HOPPER_SPEED = 0;

    //Arm CAN ID
    public static final int GRIPPER = 15;
    public static final int SHOULDER = 13;
    public static final int EXTENSION = 14;
    public static final int ELBOW = 17;
    public static final int SHOULDER_ENCODER = 22;
    public static final int ELBOW_ENCODER = 26;

    public static final double SHOULDER_RATIO = (5.0)*(5.0)*(72.0/18.0)*(60.0/12.0) / 360.0;  // Ratio
    public static final double EXTENSION_RATIO = (50.0/12.0)*(50.0/20.0)*(2.0)/(2.0*Math.PI);  // Revolutions to Inches
    public static final double WRIST_RATIO = (9.0)*(7.0)*(96.0/36.0);  // Ratio

    public static final double SHOULDER_OFFSET = compBot ? 111.709 : 160.049;
    public static final double ELBOW_OFFSET = compBot ? 50.889 : 167.783;

    //Arm pid motion magic gains
    public static final double SHOULDER_MOTOR_kF = 0;
    public static final double SHOULDER_MOTOR_kP = 0.4;
    public static final double SHOULDER_MOTOR_kI = 0;
    public static final double SHOULDER_MOTOR_kD = 0;

    public static final double EXTENSION_MOTOR_kF = 0;
    public static final double EXTENSION_MOTOR_kP = 0.4;
    public static final double EXTENSION_MOTOR_kI = 0;
    public static final double EXTENSION_MOTOR_kD = 0;

    public static final double ELBOW_MOTOR_kF = 0;
    public static final double ELBOW_MOTOR_kP = 3.5;
    public static final double ELBOW_MOTOR_kI = .012;
    public static final double ELBOW_MOTOR_kI_ZONE = 10;
    public static final double ELBOW_MOTOR_kD = .02;

    //Arm motion magic parameters
    public static final double SHOULDER_CRUISEVELOCITY = 16000 * 16;
    public static final double SHOULDER_ACCEL = 6000 * 6; // 16 // new 6

    public static final double EXTENSION_CRUISEVELOCITY = 16000 * 10;
    public static final double EXTENSION_ACCEL = 6000 * 10;

    public static final double ELBOW_CRUIESVELOCITY = 1600 * 6;
    public static final double ELBOW_ACCEL = 600 * 1;

    //Arm pid slots
    public static final int SHOULDER_PID_SLOT = 0;
    public static final int SHOULDER_K_PID_LOOP_IDX = 0;

    public static final int EXTENSION_PID_SLOT = 0;
    public static final int EXTENSION_K_PID_LOOP_IDX = 0;

    public static final int ELBOW_PID_SLOT = 0;
    public static final int ELBOW_K_PID_LOOP_IDX = 0;

    public static final double FALCON500_TICKS_PER_REV = 2048;
    // public static final double BAG_TICKS_PER_REV = 2048;
    // public static final double SHOULDER_DEGREES_TO_TICKS = -FALCON500_TICKS_PER_REV * 1.3786;
    // public static final double ELBOW_DEGREES_TO_TICKS = BAG_TICKS_PER_REV * 1;

    //Arm motion magic set positions
    public static final int ARM_BUMP_LATCH_TIME = 17;

    public static final int ARM_TIMEOUT = 200;


    // Extension Constraints
    
    public static final double MAXIMUM_EXTENSION_INCHES = 22.5; // actual maximum is 23


    public static final double GRIPPER_HOLD_POWER = -0.4;


    // LED constants

    public static final int[] LED_TELEOP_DEFAULT = {0, 0, 255};
    public static final int[] LED_AUTON_RED = {255, 0, 0};
    public static final int[] LED_AUTON_BLUE = {0, 0, 255};

    public static final int[] LED_DETECT_CORRECT = {0, 255, 0};
    public static final int[] LED_DETECT_BAD = {255, 0, 0};

    public static final double LED_LEFT_THRESH_LOW = 0;
    public static final double LED_LEFT_THRESH_HIGH = 16;

    public static final double LED_RIGHT_THRESH_LOW = -16;
    public static final double LED_RIGHT_THRESH_HIGH = 0;

}


// Swerve Bot Constants
// public final class Constants {
//     public static final boolean realBot = false;

//     public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445;

//     public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

//     public static final double MAX_VOLTAGE = 12.0;

//     public static final double MAX_VELOCITY_METERS_PER_SECOND =
//         6380.0 / 60.0 *
//         SdsModuleConfigurations.MK4_L2.getDriveReduction() *
//         SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

//     public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
//         Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

//     public static final int DRIVETRAIN_PIGEON_ID = 50;

//     public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
//     public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
//     public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
//     public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(21.97265625);

//     public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
//     public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
//     public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
//     public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(58.71093750000001);

//     public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
//     public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
//     public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;
//     public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(309.27886962890625);

//     public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
//     public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
//     public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
//     public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(212.2890625);

//     public static final double INTAKE_PACKAGE_POSITION = 90;
//     public static final double INTAKE_STATION_POSITION = 135;
//     public static final double INTAKE_COLLECT_POSITION = 172.5;
//     public static final double INTAKE_SPEED_CUBE = .5;
//     public static final double INTAKE_SPEED_CONE = 1;
//     public static final int INTAKE_TIMEOUT = 200;

//     // Hopper CAN ID
//     public static final int HOPPER_MOTOR = 11;
//     public static final int FINGER_SERVO = 2;
//     public static final int LEFT_SENSOR = 0;
//     public static final int TOP_SENSOR = 1;
//     public static final int BOTTOM_SENSOR = 2;
//     public static final int RIGHT_SENSOR = 3;
//     public static final double HOPPER_OVERRIDE_SPEED = 0.5;
//     public static final double HOPPER_SPEED = 0;

//     //Arm CAN ID
//     public static final int SHOULDER = 13;
//     public static final int EXTENSION = 14;
//     public static final int ELBOW = 17;
//     public static final int SHOULDER_ENCODER = 22;
//     public static final int ELBOW_ENCODER = 25;


//     //Arm pid motion magic gains
//     public static final double SHOULDER_MOTOR_kF = 0;
//     public static final double SHOULDER_MOTOR_kP = 0.4;
//     public static final double SHOULDER_MOTOR_kI = 0;
//     public static final double SHOULDER_MOTOR_kD = 0;

//     public static final double EXTENSION_MOTOR_kF = 0;
//     public static final double EXTENSION_MOTOR_kP = 0.4;
//     public static final double EXTENSION_MOTOR_kI = 0;
//     public static final double EXTENSION_MOTOR_kD = 0;

//     public static final double ELBOW_MOTOR_kF = 0;
//     public static final double ELBOW_MOTOR_kP = 0.2;
//     public static final double ELBOW_MOTOR_kI = 0;
//     public static final double ELBOW_MOTOR_kD = 0;

//     //Arm motion magic parameters
//     public static final double SHOULDER_CRUISEVELOCITY = 16000 * 6;
//     public static final double SHOULDER_ACCEL = 6000 * 6;

//     public static final double EXTENSION_CRUISEVELOCITY = 16000 * 6;
//     public static final double EXTENSION_ACCEL = 6000 * 6;

//     public static final double ELBOW_CRUIESVELOCITY = 1600 * 6;
//     public static final double ELBOW_ACCEL = 600 * 6;

//     //Arm pid slots
//     public static final int SHOULDER_PID_SLOT = 0;
//     public static final int SHOULDER_K_PID_LOOP_IDX = 0;

//     public static final int EXTENSION_PID_SLOT = 0;
//     public static final int EXTENSION_K_PID_LOOP_IDX = 0;

//     public static final int ELBOW_PID_SLOT = 0;
//     public static final int ELBOW_K_PID_LOOP_IDX = 0;

//     public static final double FALCON500_TICKS_PER_REV = 2048;
//     public static final double BAG_TICKS_PER_REV = 2048;
//     public static final double SHOULDER_DEGREES_TO_TICKS = -FALCON500_TICKS_PER_REV * 1.3786;
//     public static final double ELBOW_DEGREES_TO_TICKS = BAG_TICKS_PER_REV * 1;

//     //Arm motion magic set positions
//     public static final double SHOULDER_TARGET_POSITION_LOW = 90 * SHOULDER_DEGREES_TO_TICKS; // In degrees
//     public static final double EXTENSION_TARGET_POSITION_LOW = 0 * FALCON500_TICKS_PER_REV; // In Revolutions
//     public static final double ELBOW_TARGET_POSITION_LOW = 0 * ELBOW_DEGREES_TO_TICKS;

//     public static final double SHOULDER_TARGET_POSITION_MIDDLE = 80 * SHOULDER_DEGREES_TO_TICKS;
//     public static final double EXTENSION_TARGET_POSITION_MIDDLE = 0 * FALCON500_TICKS_PER_REV;
//     public static final double ELBOW_TARGET_POSITION_MIDDLE = 0 * ELBOW_DEGREES_TO_TICKS;

//     public static final double SHOULDER_TARGET_POSITION_HIGH = 45 * SHOULDER_DEGREES_TO_TICKS;
//     public static final double EXTENSION_TARGET_POSITION_HIGH = 50 * FALCON500_TICKS_PER_REV;
//     public static final double ELBOW_TARGET_POSITION_HIGH = 0 * ELBOW_DEGREES_TO_TICKS;

//     public static final double SHOULDER_TARGET_POSITION_PACKAGE = 0;
//     public static final double EXTENSION_TARGET_POSITION_PACKAGE = 0;
//     public static final double ELBOW_TARGET_POSITION_PACKAGE = 0;

//     public static final int ARM_TIMEOUT = 200;

// }