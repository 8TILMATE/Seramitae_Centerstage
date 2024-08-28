package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    Servo Drona;
    public static double DroneOpen=0.65;
    public static double DroneClose=1;
    public void Init(HardwareMap hardwareMap){
        Drona=hardwareMap.get(Servo.class,"Drona");
    }
    public void UpdateDrone(Gamepad gamepad){
        if(gamepad.right_bumper){
            Drona.setPosition(DroneOpen);
        }
        if(gamepad.left_bumper){
            Drona.setPosition(DroneClose);
        }
    }
}
