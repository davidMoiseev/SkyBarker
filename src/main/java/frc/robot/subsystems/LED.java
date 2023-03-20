package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.sensors.Camera;

public class LED {
    public AddressableLED strip1 = new AddressableLED(2);
    public AddressableLEDBuffer strip1Buffer = new AddressableLEDBuffer(19);
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
    }

    public void teleopAction(){
        if (Camera.getLeftDetecting()){
            if (Camera.getLeftX() <= 16 && Camera.getLeftX() >= 0) {
                setLights(0, 255, 0);
                // if the camera is deteting and is within the thresholds, turn the lights green
            } else {
                setLights(255, 0, 0);
                // if the camera is detecting and is not within the thersholds, turn the lights red
            }
        } else if (Camera.getRightDetecting()){
            if (Camera.getRightX() >= -16 && Camera.getRightX() <= 0) {
                setLights(0, 255, 0);
                // if the camera is deteting and is within the thresholds, turn the lights green
            } else {
                setLights(255, 0, 0);
                // if the camera is detecting and is not within the thersholds, turn the lights red
            }
        } else {
            setLights(0, 0, 255);
            // if the camera isnt detecting, set the lights to blue
        }
    }

    public void disabledAction(){
        setLights(0, 0, 0);
        // turn lights off in disabled
    }

    public void autonAction(){
        if (team == 0) {
            setLights(0, 0, 255);
            // if we are on the blue alliance set lights to blue
        } else if (team == 1) {
            setLights(255, 0, 0);
            // if we are on the red alliance set lights to red
        }
    }

    private void setLights(int red, int green, int blue){
        for (var i = 0; i < strip1Buffer.getLength(); i++) {
            strip1Buffer.setRGB(i, 0, 0, 0); 
        }
        strip1.setData(strip1Buffer);
        // function to set lights a color to conserve space and time
    }
}










































































