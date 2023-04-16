package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;
import frc.robot.subsystems.Arm.IntakePos;
import frc.robot.subsystems.Arm.IntakeSpeed;

import java.util.List;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class BlueCord extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        driveToObject2,
        score2,
        driveToObject3,
        driveToObject4,
        score3,
        end,
        pause1,
        pause2
    }

    public Trajectory firstCube;
    public Trajectory firstScore;
    public Trajectory secondCube;
    public Trajectory secondScore;

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    Trajectory trajectory;

    double armTime;

    public BlueCord(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(180);

        point = 0;
        
        firstCube = importTraj("pathweaver/output/bcord1.wpilib.json");
        firstScore = importTraj("pathweaver/output/bcord2.wpilib.json");
        secondCube = importTraj("pathweaver/output/bcord3.wpilib.json");
        secondScore = importTraj("pathweaver/output/bcord4.wpilib.json");

        timer.reset();
        timer.start();

        armTime = 0;

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                coneIntake = false;
                intakeSpeed = IntakeSpeed.out;
                armPos = ArmPos.packagePos;

                if(timer.get() > .2){
                    timer.reset();

                    autoState = AutoState.driveToObject1;
                }
            break;
            case driveToObject1:
                driving = true;
                coneIntake = false;

                armPos = ArmPos.Zero;

                if(timer.get() > 1.75){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                } else {
                    intakeSpeed = IntakeSpeed.none;
                    intakePos = IntakePos.pack;
                }

                if(Math.abs(Drivetrain.getPose().getX() - firstCube.getInitialPose().getX()) > 2.5){
                    desState = firstCube.sample(timer.get()*1.5);
                } else {
                    desState = firstCube.sample(timer.get()/1.5);
                }

                // if(Math.abs(Drivetrain.getPose().getX() - firstCube.getInitialPose().getX()) > 1){
                    targetTheta = Rotation2d.fromDegrees(5);
                // }

                if(Math.abs(Drivetrain.getPose().getX() - firstCube.getStates().get(firstCube.getStates().size()-1).poseMeters.getX()) < .1 &&
                Math.abs(Drivetrain.getPose().getY() - firstCube.getStates().get(firstCube.getStates().size()-1).poseMeters.getY()) < .1){
                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;

                
                intakePos = IntakePos.pack;
                if(timer.get() > .75){
                    intakeSpeed = IntakeSpeed.none;
                } else {
                    intakeSpeed = IntakeSpeed.onCube;
                }

                desState = firstScore.sample(timer.get()/1.5);
                targetTheta = Rotation2d.fromDegrees(-170);

                if(Math.abs(Drivetrain.getPose().getX() - firstScore.getStates().get(firstScore.getStates().size()-1).poseMeters.getX()) < .25 &&
                Math.abs(Drivetrain.getPose().getY() - firstScore.getStates().get(firstScore.getStates().size()-1).poseMeters.getY()) < .25){                
                    timer.reset();

                    intakeSpeed = IntakeSpeed.out;
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;  
            case score2:
                driving = false;
                
                intakeSpeed = IntakeSpeed.out;

                if(timer.get() > .25){
                    gripperSpeed = 0;
                    
                    timer.reset();

                    autoState = AutoState.driveToObject3;
                }
            break;
            case driveToObject3:
                driving = true;
                if(Math.abs(Drivetrain.getPose().getX() - firstCube.getInitialPose().getX()) > 4){
                    desState = secondCube.sample(timer.get()*1.5);
                } else {
                    desState = secondCube.sample(timer.get()/1.5);
                }
                // if(Math.abs(Drivetrain.getPose().getX() - firstCube.getInitialPose().getX()) > 3.5){
                targetTheta = Rotation2d.fromDegrees(65);

                if(timer.get() > 1.75){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }

                if(Math.abs(Drivetrain.getPose().getX() - secondCube.getStates().get(secondCube.getStates().size()-1).poseMeters.getX()) < .1 &&
                Math.abs(Drivetrain.getPose().getY() - secondCube.getStates().get(secondCube.getStates().size()-1).poseMeters.getY()) < .1){                
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.driveToObject4;
                }
            break;
            case driveToObject4:
                driving = true;

                desState = secondScore.sample(timer.get()/1.5);
                targetTheta = Rotation2d.fromDegrees(-165);
                
                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.none;

                gripperSpeed = -.75;

                if(Math.abs(Drivetrain.getPose().getX() - secondScore.getStates().get(secondScore.getStates().size()-1).poseMeters.getX()) < 1){                    
                    // timer.reset();
                    
                    // gripperSpeed = 0;

                    // autoState = AutoState.score3;

                    intakeSpeed = IntakeSpeed.out;
                }
            break;
            case score3:
                driving = false;

                intakeSpeed = IntakeSpeed.out;
            break;
            case end:
                driving = false;
                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.none;
            break;
        }

        HotLogger.Log("AutoState", autoState.toString());
        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}