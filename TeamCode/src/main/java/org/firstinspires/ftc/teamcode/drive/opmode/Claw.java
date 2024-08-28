package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.KeyPair;

import kotlin.Pair;

public class Claw {
    Servo wrist,left,right;
    double close_left=0.5,close_right=0.5;
    double open_outtake_left=0.7,open_outtake_right=0.3;
    boolean isIntaking=false,isOuttaking=false;
    public class DepositPurple implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            wrist.setPosition(0.5);

            right.setPosition(open_outtake_right);
            return false;
        }

    }
    public Action depositPurple(){
        return  new DepositPurple();
    }
    public class WristDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            wrist.setPosition(0.5);
            return false;
        }

    }
    public Action wristDown(){
        return  new WristDown();
    }
    public class OuttakeWrist implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            wrist.setPosition(0.65);
            return false;
        }
    }
    public Action OuttakeWrist(){
        return  new OuttakeWrist();
    }
    public class OuttakeWristMid implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            wrist.setPosition(0.55);
            return false;
        }
    }
    public Action outtakeWristMid(){
        return  new OuttakeWristMid();
    }
    public class DepositYellow implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            left.setPosition(open_outtake_left);
            return false;
        }
    }
    public Action depositYellow(){
        return  new DepositYellow();
    }
    public class CloseYellowandWristDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            left.setPosition(close_left);
            wrist.setPosition(0.85);
            return false;
        }
    }
    public Action closeYellowandWristDown(){
        return  new CloseYellowandWristDown();
    }


    public class ClosePurple implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            wrist.setPosition(0.85);
            right.setPosition(close_right);
            return false;
        }

    }
    public Action closePurple(){
        return  new ClosePurple();
    }


    /*
    public void ClosePurple(){
        wrist.setPosition(0.85);
        right.setPosition(close_right);
    }
    public void DepositYellow(){
        left.setPosition(open_outtake_left);
    }
    public void CloseYellow(){
        left.setPosition(close_left);
    }

     */
    public void Init(HardwareMap hardwareMap)
    {
        wrist = hardwareMap.get(Servo.class,"Wrist");
        left = hardwareMap.get(Servo.class,"Left");
        right = hardwareMap.get(Servo.class,"Right");
        wrist.setPosition(0.85);
        left.setPosition(close_left);
        right.setPosition(close_right);
    }
    public void UpdateClaw(Gamepad gamepad,Gamepad gamepad2){
       if(gamepad2.a){
            wrist.setPosition(0.85);

        }
       if(gamepad.a){
           wrist.setPosition(0.5);
           left.setPosition(0.85);
           right.setPosition(0.15);
       }
       if(gamepad.start){
           left.setPosition(0.85);
           right.setPosition(0.15);
       }
       if(gamepad.b){
           wrist.setPosition(0.85);
       }
       if(gamepad2.x){
           wrist.setPosition(0.7);
       }
       if(gamepad2.y){
           wrist.setPosition(0.65);
       }
       if(gamepad2.b){
           wrist.setPosition(0.5);
       }
       if(gamepad.left_bumper){
           if(isOuttaking) {
               left.setPosition(open_outtake_left);
           }
           if(isIntaking){
               left.setPosition(close_left);
           }
       }
       if(gamepad.right_bumper){
           if(isOuttaking) {
               right.setPosition(open_outtake_right);
           }
           if(isIntaking){
               right.setPosition(close_right);
           }
       }
       if(!isIntaking&&!isOuttaking){
           right.setPosition(0.5);
           left.setPosition(0.5);
       }


    }
    public void UpdateParams(boolean isIntaking1,boolean isOuttaking1){
        isIntaking=isIntaking1;
        isOuttaking=isOuttaking1;
    }
    public Pair<Boolean,Boolean> SentBoolsBack(){
        return new Pair<>(isIntaking,isOuttaking);
    }

}
