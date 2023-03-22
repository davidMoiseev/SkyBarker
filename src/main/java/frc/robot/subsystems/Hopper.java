package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import static frc.robot.Constants.*;

import org.hotutilites.hotlogger.HotLogger;

public class Hopper {
    public static int PixieCam = 0;
    // public static Servo fingerServo = new Servo(FINGER_SERVO);
    // public static DigitalInput sensorTop= new DigitalInput(TOP_SENSOR);
    // public static DigitalInput sensorLeft= new DigitalInput(LEFT_SENSOR);
    // public static DigitalInput sensorRight= new DigitalInput(RIGHT_SENSOR);
    // public static DigitalInput sensorBottom= new DigitalInput(BOTTOM_SENSOR);
    public static CANSparkMax hopperMotor = new CANSparkMax(HOPPER_MOTOR, MotorType.kBrushless);
    public static HopperCam hopperCam = HopperCam.CUBE;
    public static boolean hopperOverrideActive = false;

    public Hopper(){
      hopperMotor.setIdleMode(IdleMode.kCoast);
    }

    enum HopperCam {
      NONE,
      CUBE,
      CONE
    }

    public void HopperPeriodic(RobotCommander commander){
        // if(commander.operator.getRightStickButton()){
        //   hopperMotor.set(-.3); // .25
        // } else {
        //   hopperMotor.set(0);

        // }
        if(commander.runHopper()){
          hopperMotor.set(-.2);
        } else {
          hopperMotor.set(0);
        }
    }

  public void logData(){
    HotLogger.Log("Hopper Override", hopperOverrideActive);
    }
}
  
