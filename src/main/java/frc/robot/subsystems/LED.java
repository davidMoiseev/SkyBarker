package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;
import java.util.Random;


import static frc.robot.Constants.*;

public class LED {
    public AddressableLED strip1 = new AddressableLED(LED_PWM);
    public AddressableLEDBuffer strip1Buffer = new AddressableLEDBuffer(LED_LENGTH);
    static Alliance alliance = Alliance.Red;
    public Random assigner = new Random(LED_LENGTH);
    public Random redRandom = new Random(LED_RED_DEVIATION);
    public Random yellowRandom = new Random(LED_YELLOW_DEVIATION);
    public int resetTimer = 0;

    public LED(){
        strip1.setLength(strip1Buffer.getLength());
        setLights(0, 0, 0);
        strip1.start();
        // initialize leds as off
    }

    public void fancyDisable() {
        for (var i = 0; i <= LED_FANCY_ITERATION; i++) {
            strip1Buffer.setRGB(assigner.nextInt(LED_LENGTH), LED_FANCY_BASE[0] - redRandom.nextInt(LED_RED_DEVIATION), LED_FANCY_BASE[1] + yellowRandom.nextInt(LED_YELLOW_DEVIATION), LED_FANCY_BASE[2]);
        }
         
        if (resetTimer == LED_RESET_TIMER) {
            resetTimer = 0;
            setLights(LED_FANCY_BASE);
        }
        strip1.setData(strip1Buffer);
	}

    public void autonInit() {
        alliance = DriverStation.getAlliance();

        if (alliance == Alliance.Blue) {
            setLights(LED_AUTON_BLUE);
            // if we are on the blue alliance set lights to blue
        } else if (alliance == Alliance.Red) {
            setLights(LED_AUTON_RED);
            // if we are on the red alliance set lights to red
        }
    }

    public void teleopAction(TeleopCommander commander){
        
        if (commander.getCubeMode()){
            setLights(LED_CUBE_PICKUP);
        } else {
            setLights(LED_CONE_PICKUP);
        }
        
    }

    public void disabledAction(){
        setLights(0, 0, 0);
        // turn lights off in disabled
    }

    public void autonAction(){

    }

    private void setLights(int red, int green, int blue){
        for (var i = 0; i < strip1Buffer.getLength(); i++) {
            strip1Buffer.setRGB(i, 0, 0, 0); 
        }
        strip1.setData(strip1Buffer);
        // function to set lights a color with 3 integer values
    }

    private void setLights(int[] rgb) {
        for (var i = 0; i < strip1Buffer.getLength(); i++) {
            strip1Buffer.setRGB(i, rgb[0], rgb[1], rgb[2]); 
        }
        strip1.setData(strip1Buffer);
        // function to set lights a color with an integer array with 3 values
    }
}