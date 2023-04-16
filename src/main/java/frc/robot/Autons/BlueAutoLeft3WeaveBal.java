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

public class BlueAutoLeft3WeaveBal extends AutonBase{
    enum AutoState {
        score1,
        driveToCone,
        chomp,
        driveToConeScore,
        score2,
        driveToCube,
        driveToScoreCube,
        score3,
        balance,
        end
    }

    public AutoState autoState;

    public Timer totTimer = new Timer();

    public Timer timer = new Timer();

    int point = 0;

    Trajectory driveToCone;
    Trajectory driveToScoreCone;
    Trajectory driveToCube;
    Trajectory driveToScoreCube;
    Trajectory driveToBalance;

    double armTime;
    
    public BlueAutoLeft3WeaveBal(){
        driveToCone = importTraj("pathweaver/output/bconechomp.wpilib.json");
        driveToScoreCone = importTraj("pathweaver/output/bconetoscore.wpilib.json");
        driveToCube = importTraj("pathweaver/output/bcubenotchomp.wpilib.json");
        driveToScoreCube = importTraj("pathweaver/output/bcubetoscore.wpilib.json");
        driveToBalance = importTraj("pathweaver/output/bbalance.wpilib.json");

        initalPose = driveToCone.getInitialPose();
        initalAngle = -90;

        reset();
    }

    public void reset(){
        desState = new State();
        
        totTimer.reset();
        totTimer.start();

        timer.reset();
        timer.start();

        targetTheta = Rotation2d.fromDegrees(initalAngle);

        armTime = 0;

        autoState = AutoState.score1;
    }

    public void runAuto(){
        switch(autoState){
            case score1:
                driving = false;
                if(timer.get() > .5){
                    if(!Arm.getAchivedPostion()){
                        gripperSpeed = -.5;
                        armPos = ArmPos.topNodeCone;
                        armTime = timer.get();
                    } else {
                        if(Math.abs(armTime - timer.get()) < .45){
                            gripperSpeed = .75;
                        } else {
                            timer.reset();
    
                            autoState = AutoState.driveToCone;
                        }
                    }
                } else {
                    armPos = ArmPos.topNodeCone;
                }
            break;
            case driveToCone:
                driving = true;

                armPos = ArmPos.groundGripperCone;

                desState = driveToCone.sample(timer.get());
                targetTheta = Rotation2d.fromDegrees(-90);

                if(timer.get() < 1){
                    gripperSpeed = .75;
                } else {
                    gripperSpeed = -.8;

                }
                
                if(Math.abs(Drivetrain.getPose().getX() - driveToCone.getStates().get(driveToCone.getStates().size()-1).poseMeters.getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - driveToCone.getStates().get(driveToCone.getStates().size()-1).poseMeters.getY()) < .025){
                    armPos = ArmPos.groundGripperConePick;
                    
                    timer.reset();
                    
                    autoState = AutoState.chomp;
                }
            break;
            case chomp:
                driving = false;

                armPos = ArmPos.groundGripperConePick;

                gripperSpeed = -.8;

                if(timer.get() > .35){
                    timer.reset();
                    
                    autoState = AutoState.driveToConeScore;
                }
            break;
            case driveToConeScore:
                driving = true;

                gripperSpeed = -.8;

                armPos = ArmPos.topNodeCone;

                desState = driveToScoreCone.sample(timer.get());
                targetTheta = Rotation2d.fromDegrees(-90);

                if(Math.abs(Drivetrain.getPose().getX() - driveToScoreCone.getStates().get(driveToScoreCone.getStates().size()-1).poseMeters.getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - driveToScoreCone.getStates().get(driveToScoreCone.getStates().size()-1).poseMeters.getY()) < .075){
                    timer.reset();
                    
                    autoState = AutoState.score2;

                    gripperSpeed = .75;
                }
            break;
            case score2:
                driving = false;

                if(timer.get() < .5){
                    gripperSpeed = -.5;
                } else {
                    gripperSpeed = .75;
                }

                if(timer.get() > 1.2){
                    timer.reset();
                    autoState = AutoState.driveToCube;
                }
            break;
            case driveToCube:
                driving = true;

                gripperSpeed = .5;

                if(timer.get() > driveToCube.getTotalTimeSeconds() /3){
                    intakePos = IntakePos.collectCube;
                    intakeSpeed = IntakeSpeed.onCube;
                }

                armPos = ArmPos.intake;

                hopperSpeed = -.2;

                desState = driveToCube.sample(timer.get());
                targetTheta = Rotation2d.fromDegrees(-45);

                if(Math.abs(Drivetrain.getPose().getX() - driveToCube.getStates().get(driveToCube.getStates().size()-1).poseMeters.getX()) < .05 &&
                Math.abs(Drivetrain.getPose().getY() - driveToCube.getStates().get(driveToCube.getStates().size()-1).poseMeters.getY()) < .05){
                    // intakePos = IntakePos.cubeHandoff;
                    // intakeSpeed = IntakeSpeed.cubeHandoff;

                    timer.reset();
                    
                    autoState = AutoState.balance;
                }
            break;
            case balance:
                driving = true;

                armPos = ArmPos.packagePos;
                intakePos = IntakePos.armMoving;
                intakeSpeed = IntakeSpeed.none;

                desState = driveToBalance.sample(timer.get() / 1.5);
                targetTheta = Rotation2d.fromDegrees(-90);

                if((Math.abs(Drivetrain.getPose().getX() - driveToBalance.getStates().get(driveToBalance.getStates().size()-1).poseMeters.getX()) < .075 &&
                Math.abs(Drivetrain.getPose().getY() - driveToBalance.getStates().get(driveToBalance.getStates().size()-1).poseMeters.getY()) < .075)
                || totTimer.get() > 14.75){
                    autoState = AutoState.end;
                }
            break;
            case end:
                driving = false;
                xMode = true;
                // armPos = ArmPos.packagePos;
            break;
        }

        HotLogger.Log("AutoState", autoState.toString());
        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}