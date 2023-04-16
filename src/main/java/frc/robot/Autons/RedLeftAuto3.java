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

public class RedLeftAuto3 extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        pause1,
        driveToObject2,
        score2,
        driveToObject3,
        pause2,
        driveToObject4,
        score3,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    // List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(90)),
    //                             new Pose2d(new Translation2d(5.3,-.5), Rotation2d.fromDegrees(10)), //4.82, .5
    //                             new Pose2d(new Translation2d(-.1,-.125), Rotation2d.fromDegrees(90)));

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.5,1), Rotation2d.fromDegrees(-10)), //4.82, .5
                                new Pose2d(new Translation2d(-.05,.25), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.1, .75), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(5.6, -1.5), Rotation2d.fromDegrees(-90)));

    Trajectory trajectory;

    double armTime;

    public RedLeftAuto3(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

        point = 0;
        
        timer.reset();
        timer.start();

        armTime = 0;

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                if(!Arm.getAchivedPostion()){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCone;
                    armTime = timer.get();
                } else {
                    if(Math.abs(armTime - timer.get()) < .25){
                        gripperSpeed = .75;
                    } else {
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;

                        trajectory = createTrajectory(path.get(point), path.get(point+1),
                        Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(-10),
                        2.5,1.5);
                
                        point++;

                        timer.reset();

                        autoState = AutoState.driveToObject1;
                    }
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.intake;
                if(timer.get() > trajectory.getTotalTimeSeconds()/2){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }

                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .1 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .1){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-12 + 180), Rotation2d.fromDegrees(12 + 180),
                    2.5,1.5);

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;

                    intakePos = IntakePos.cubeHandoff;
                    intakeSpeed = IntakeSpeed.cubeHandoff;
                }
            break;
            case driveToObject2:
                driving = true;
                
                intakePos = IntakePos.cubeHandoff;
                intakeSpeed = IntakeSpeed.cubeHandoff;

                hopperSpeed = -.25;

                if(timer.get() > 2.5){
                    armPos = ArmPos.topNodeCube;
                } else if(timer.get() > 1.5){
                    armPos = ArmPos.packagePos;
                }

                gripperSpeed = -.75;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .075 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .075){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                intakeSpeed = IntakeSpeed.none;

                driving = false;
                if(timer.get() < .75){
                    gripperSpeed = -.4;
                    armPos = ArmPos.topNodeCube;
                } else {
                    if(timer.get() < 1.25){
                        gripperSpeed = .5;
                    } else if(timer.get() < 1.75){
                        armPos = ArmPos.packagePos;
                        gripperSpeed = 0;
                    } else { 

                        trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(path.get(point).getTranslation(), Rotation2d.fromDegrees(20)), 
                        List.of(path.get(point+1).getTranslation()),
                        new Pose2d(path.get(point+2).getTranslation(), Rotation2d.fromDegrees(-90)), 
                        new TrajectoryConfig(2.5, 1.5));

                        point += 2;

                        timer.reset();

                        autoState = AutoState.driveToObject4;
                    }
                }
            break;
            case driveToObject4:
                driving = true;

                overrideIntake = true;

                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()/2){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }

                // gripperSpeed = -.75;

                if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .05){       
                    timer.reset();

                    gripperSpeed = 0;

                    autoState = AutoState.end;
                }
            break;
            case end:
                overrideIntake = false;
                driving = false;
                armPos = ArmPos.packagePos;
                intakePos = IntakePos.pack;
                intakeSpeed = IntakeSpeed.none;
            break;
        }

        HotLogger.Log("AutoState", autoState.toString());
        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}