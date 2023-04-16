package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class CableAutocopy extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToStation,
        mobility,
        balance,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();
    public Timer totalTime = new Timer();

    int point = 0;

    Pose2d startingPose = new Pose2d(0,0, Rotation2d.fromDegrees(-90));
    Pose2d endPose = new Pose2d(4,0, Rotation2d.fromDegrees(-90));

    Trajectory trajectory;

    public CableAutocopy(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

        point = 0;
        
        timer.reset();
        timer.start();

        totalTime.reset();
        totalTime.start();

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        driving = true;
        desState = new State();
    }
}