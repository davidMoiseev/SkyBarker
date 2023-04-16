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

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class NewArmTest extends AutonBase{
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

    public Timer timer = new Timer();

    int point = 0;

    Trajectory driveToCone;
    Trajectory driveToScoreCone;
    Trajectory driveToCube;
    Trajectory driveToScoreCube;
    Trajectory driveToBalance;

    double armTime;
    
    public NewArmTest(){
        driveToCone = importTraj("pathweaver/output/conechomp.wpilib.json");
        driveToScoreCone = importTraj("pathweaver/output/conetoscore.wpilib.json");
        driveToCube = importTraj("pathweaver/output/cubenotchomp.wpilib.json");
        driveToScoreCube = importTraj("pathweaver/output/cubetoscore.wpilib.json");
        driveToBalance = importTraj("pathweaver/output/cubetobalance.wpilib.json");

        initalPose = driveToCone.getInitialPose();
        initalAngle = -90;

        reset();
    }

    public void reset(){
        desState = new State();
        
        timer.reset();
        timer.start();

        targetTheta = Rotation2d.fromDegrees(initalAngle);

        armTime = 0;

        autoState = AutoState.score1;
    }

    public void runAuto(){
        switch(autoState){
            case score1:
                armPos = ArmPos.topNodeCone;

                if(timer.get() > 3){
                    timer.reset();

                    autoState = AutoState.driveToCone;
                }
            break;
            case driveToCone:
                armPos = ArmPos.groundGripperConePick;

                if(timer.get() > 3){
                    timer.reset();

                    autoState = AutoState.driveToConeScore;
                }
            break;
            case driveToConeScore:
                armPos = ArmPos.topNodeCone;

                if(timer.get() > 3){
                    timer.reset();

                    autoState = AutoState.end;
                }
            break;
            case end:
                armPos = ArmPos.packagePos;
            break;
        }

        HotLogger.Log("AutoState", autoState.toString());
        SmartDashboard.putString("AutoState", autoState.toString());
        SmartDashboard.putBoolean("Arm Achieved Position", Arm.getAchivedPostion());
    }
}