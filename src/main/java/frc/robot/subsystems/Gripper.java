package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.subsystems.Arm.ArmPos;

public class Gripper {
    VictorSPX gripper;
    // private Arm arm = new Arm();
    private ArmPos armPositionPrev;
    int timer;
    


    public Gripper(int CANID) {
        gripper = new VictorSPX(CANID);

        gripper.setNeutralMode(NeutralMode.Brake);

        gripper.setInverted(true);
    }
    

    public void action(RobotCommander commander) {
        if ((commander.getArmPosition() != ArmPos.humanPlayerPickup && armPositionPrev == ArmPos.humanPlayerPickup)) {
            // start timer
            timer = 150;

        } else if ((commander.getArmPosition() != ArmPos.Zero || commander.getArmPosition() != ArmPos.manual)) {
            // Increment timer
            timer--;
        } 

        if ((commander.getArmPosition() != ArmPos.Zero || commander.getArmPosition() != ArmPos.manual) && 
            (timer < 150) && (timer > 0)){
            gripper.set(ControlMode.PercentOutput, -.8);
        }
        else{
        gripper.set(ControlMode.PercentOutput, commander.getGripperCommand());
        SmartDashboard.putNumber("Gripper Command", commander.getGripperCommand());
        }
        armPositionPrev = commander.getArmPosition();
        SmartDashboard.putNumber("timer", timer);
    }
}