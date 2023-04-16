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
import frc.robot.subsystems.Arm.ArmPos.ArmBumpDirection;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class NewMidBlue extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToStation,
        mobility,
        balance,
        pause1,
        pause2,
        finalMove,
        onStation,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();
    public Timer totalTime = new Timer();

    int point = 0;

    Pose2d startingPose = new Pose2d(0,0, Rotation2d.fromDegrees(-90));
    Pose2d over = new Pose2d(5.5,0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(0,0, Rotation2d.fromDegrees(170));
    Pose2d onStation = new Pose2d(1.8,0, Rotation2d.fromDegrees(170));

    Trajectory trajectory;

    double armTime;

    public NewMidBlue(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

        point = 0;
        
        timer.reset();
        timer.start();

        armTime = 0;

        totalTime.reset();
        totalTime.start();

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                if(timer.get() > .5){
                    if(!Arm.getAchivedPostion() || timer.get() < .75){
                        gripperSpeed = -.5;
                        armPos = ArmPos.topNodeCone;
                        armTime = timer.get();
                    } else {
                        if(Math.abs(armTime - timer.get()) < .45){
                            gripperSpeed = .75;
                        } else {
                            
                            armPos = ArmPos.packagePos;

                            trajectory = createTrajectory(startingPose, over,
                            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0),
                            1,1);

                            timer.reset();
    
                            autoState = AutoState.mobility;
                        }
                    }
                } else {
                    armPos = ArmPos.topNodeCone;
                }
            break;
            case mobility:
                driving = true;

                if(desState.poseMeters.getX() < 2.25){
                    targetTheta = startingPose.getRotation();
                } else {
                    targetTheta = over.getRotation();
                }

                if(desState.poseMeters.getX() > 2.9){
                    armPos = ArmPos.Zero;
                    intakeSpeed = IntakeSpeed.onCube;
                    intakePos = IntakePos.collectCube;
                }

                if(desState.poseMeters.getX() > 3.4){
                    desState = trajectory.sample(timer.get() * 1.5);
                } else {
                    desState = trajectory.sample(timer.get());
                }

                if(Math.abs(Drivetrain.getPose().getX() - over.getX()) < .075 && 
                Math.abs(Drivetrain.getPose().getY() - over.getY()) < .075){
                    trajectory = createTrajectory(over, endPose,
                    Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180),
                    1.1,1.1);

                    timer.reset();

                    autoState = AutoState.finalMove;
                }
            break;
            case finalMove:
                driving = true;

                intakeSpeed = IntakeSpeed.none;
                intakePos = IntakePos.pack;

                if(timer.get() < 2){
                    intakeSpeed = IntakeSpeed.onCube;
                } else {
                    intakeSpeed = IntakeSpeed.none;
                }

                if(desState.poseMeters.getX() < 1.5){
                    desState = trajectory.sample(timer.get() * 1.25);
                } else {
                    desState = trajectory.sample(timer.get());
                }
                targetTheta = endPose.getRotation();

                if(Math.abs(Drivetrain.getPose().getX() - endPose.getX()) < .075 && 
                Math.abs(Drivetrain.getPose().getY() - endPose.getY()) < .075){
                    intakeSpeed = IntakeSpeed.autoOut;



                    if(timer.get() > (trajectory.getTotalTimeSeconds() + .3)){
                        trajectory = createTrajectory(endPose, onStation,
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0),
                        2,2);

                        timer.reset();

                        autoState = AutoState.pause1;
                    }
                }
            break;
            case pause1:
                // if(){
                    desState = trajectory.sample(timer.get());
                    targetTheta = endPose.getRotation();    
                // }
 
                if(Math.abs(Pigeon.getPitch()) > 13) {
                    timer.reset();
                    autoState = AutoState.balance;
                }
            break;
            case balance:
                intakeSpeed = IntakeSpeed.none;
                driving = false;

                // if(totalTime.get() > 14.8){
                //     autoState = AutoState.end;
                // }
                if(totalTime.get() > 14.8){
                    xMode = true;
                    newAutoBal = false;
                } else {
                    if(Math.abs(Pigeon.getPitch()) < 13.75){
                        newAutoBal = false;
                        xMode = true;
                    } else {
                        newAutoBal = true;
                    }
                }
            break;
            case end:
                driving = false;
                newAutoBal = false;
                xMode = true;
            break;
        }
        // driving = false;
        // newAutoBal = true;
        // xMode = false;
    }
}