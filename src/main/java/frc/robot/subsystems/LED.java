package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;
import frc.robot.sensors.Camera;
import static frc.robot.Constants.*;

public class LED {
    public AddressableLED strip1 = new AddressableLED(LED_PWM);
    public AddressableLEDBuffer strip1Buffer = new AddressableLEDBuffer(LED_LENGTH);
    static int team = 2;

    public LED(){
        strip1.setLength(strip1Buffer.getLength());
        setLights(0, 0, 0);
        strip1.start();
        // initialize leds as off
    }

    public void autonInit() {
        Alliance alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Blue) {
            team = 0;
        } else if (alliance == Alliance.Red) {
            team = 1;
        }
        //get the alliance color and translate it to an int

        if (team == 0) {
            setLights(LED_AUTON_BLUE);
            // if we are on the blue alliance set lights to blue
        } else if (team == 1) {
            setLights(LED_AUTON_RED);
            // if we are on the red alliance set lights to red
        }
    }

    public void teleopAction(TeleopCommander commander){
        if (Camera.getLeftDetecting()){
            if (Camera.getLeftX() <= LED_LEFT_THRESH_HIGH && Camera.getLeftX() >= LED_LEFT_THRESH_LOW) {
                setLights(LED_DETECT_CORRECT);
                // if the camera is deteting and is within the thresholds, turn the lights green
            } else {
                setLights(LED_DETECT_BAD);
                // if the camera is detecting and is not within the thersholds, turn the lights red
            }
        } else if (Camera.getRightDetecting()){
            if (Camera.getRightX() <= LED_RIGHT_THRESH_HIGH && Camera.getRightX() >= LED_LEFT_THRESH_LOW) {
                setLights(LED_DETECT_CORRECT);
                // if the camera is deteting and is within the thresholds, turn the lights green
            } else {
                setLights(LED_DETECT_BAD);
                // if the camera is detecting and is not within the thersholds, turn the lights red
            }
        } else if (commander.getCubeMode()){
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

































































