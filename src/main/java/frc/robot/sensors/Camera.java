package frc.robot.sensors;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    public static NetworkTable limelightLeft;
    public static NetworkTable limelightRight;  
    static NetworkTableEntry leftX;
    static NetworkTableEntry rightX;
    static NetworkTableEntry leftY;
    static NetworkTableEntry rightY;
    static NetworkTableEntry leftTagDetected;
    static NetworkTableEntry rightTagDetected;

    // static NetworkTableEntry leftTagDetected;
    // static NetworkTableEntry rightTagDetected;

    static Pose2d leftPose = new Pose2d(0,0, new Rotation2d());
    static Pose2d rightPose = new Pose2d(0,0, new Rotation2d());

    public Camera(){
        limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
        limelightRight = NetworkTableInstance.getDefault().getTable("limelight-right");
        leftX = limelightLeft.getEntry("tx");
        leftY = limelightLeft.getEntry("ty");
        rightX = limelightRight.getEntry("tx");
        rightY = limelightRight.getEntry("ty");
        leftTagDetected = limelightLeft.getEntry("tv");
        rightTagDetected = limelightRight.getEntry("tv");

        // Alliance color = Alliance.Blue;

        // if (color == Alliance.Blue) {
        //     leftBotPose = limelightLeft.getEntry("botpose_wpiblue");
        //     rightBotPose = limelightRight.getEntry("botpose_wpiblue");
        //     System.out.println("Here");
        // } else {
        //     leftBotPose = limelightLeft.getEntry("botpose_wpired");
        //     rightBotPose = limelightRight.getEntry("botpose_wpired");
        //     System.out.println("There");
        // }
    }


    public void enabled(){
        limelightLeft.getEntry("ledMode").setNumber(3);
        limelightRight.getEntry("ledMode").setNumber(3);
    }

    public void disabled(){
        limelightLeft.getEntry("ledMode").setNumber(1);
        limelightRight.getEntry("ledMode").setNumber(1);
    }

    public static double getLeftX(){
        return leftX.getDouble(0);
    }

    public static double getRightX(){
        return rightX.getDouble(0);
    }

    public static double getLeftY(){
        return leftY.getDouble(0);
    }

    public static double getRightY(){
        return rightY.getDouble(0);
    }

    public static boolean getLeftDetecting(){
        return leftTagDetected.getDouble(0) == 1;
    }

    public static boolean getRightDetecting(){
        return rightTagDetected.getDouble(0) == 1;
    }

    public void logData(){
        SmartDashboard.putNumber("LEft y", leftY.getDouble(0));
    }
}
