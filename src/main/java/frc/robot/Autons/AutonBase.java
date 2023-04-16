package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;

public abstract class AutonBase {
    public boolean coneIntake;
    public boolean newAutoBal;
    public boolean overrideIntake;
    public double hopperSpeed;
    public State desState;
    public Rotation2d targetTheta;
    public Timer timer;
    public Pose2d initalPose;
    public double initalAngle;
    public Pose2d endingPose;
    public Arm.ArmPos armPos = ArmPos.Zero;
    public boolean intakeOn;
    public boolean pickUpObject;
    public boolean driving;
    public boolean autoBalance;
    public double gripperSpeed;
    public IntakeSpeed intakeSpeed = IntakeSpeed.none;
    public IntakePos intakePos = IntakePos.none;
    public boolean overrideNegSide = false;
    public boolean xMode = false;

    public abstract void runAuto();
    public abstract void reset();

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose, Rotation2d heading1, Rotation2d heading2, double vel, double acc, boolean reversed){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startingPose.getTranslation(), heading1),
            List.of(),
            new Pose2d(endPose.getTranslation(), heading2),
            new TrajectoryConfig(vel, acc).setReversed(reversed));
    }

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose, double heading1, double heading2, double vel, double acc, boolean reversed){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(heading1)),
            List.of(),
            new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(heading2)),
            new TrajectoryConfig(vel, acc).setReversed(reversed));
    }

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose, Rotation2d heading1, Rotation2d heading2, double vel, double acc){
        return createTrajectory(startingPose, endPose, heading1, heading2, vel, acc, false);
    }

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose){
        double headingAngle = Math.toDegrees(Math.atan2(endPose.getY()-startingPose.getY(), 
                                             endPose.getX()-startingPose.getX()));

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            List.of(),
            new Pose2d(endPose.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
            new TrajectoryConfig(2, 1).setKinematics(Drivetrain.kinematics));
    }

    public Trajectory createTrajectory(Pose2d startingPose, Pose2d endPose, Rotation2d heading1, Rotation2d heading2){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startingPose.getTranslation(), heading1),
            List.of(),
            new Pose2d(endPose.getTranslation(), heading2),
            new TrajectoryConfig(4, 3));
    }

    public Trajectory importTraj(String loc){
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(loc);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + loc, ex.getStackTrace());
            return new Trajectory();
        }
    }
    
    public State getState(double time, Trajectory traj, Rotation2d heading){
        State curState = traj.sample(time);
            
        return new State(time,
            curState.velocityMetersPerSecond,
            curState.accelerationMetersPerSecondSq,
            new Pose2d(curState.poseMeters.getX(), curState.poseMeters.getY(), heading),
            1000);
    }
}
