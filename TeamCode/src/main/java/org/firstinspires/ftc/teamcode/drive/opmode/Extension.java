package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Extension
{

    DcMotor extension= null;
    int state=0;
    public class Extend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            while(extension.getCurrentPosition()<130){
                extension.setPower(0.8);
            }
            extension.setPower(0);
                return false;
        }

    }
    public Action extend(){
        return  new Extend();
    }
    public class Deextend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            while(extension.getCurrentPosition()>=15){
                extension.setPower(-0.4);
            }
            extension.setPower(0);
            return false;
        }

    }
    public Action deextend(){
        return  new Deextend();
    }
    public void Init(HardwareMap hardwareMap){
        extension = hardwareMap.get(DcMotor.class,"Extension");
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void UpdateExtension(Gamepad gamepad,Gamepad Intake){
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(Intake.dpad_up){
        extension.setPower(0.2);
        state=0;
        }
        else if (Intake.dpad_down&& extension.getCurrentPosition()>=15){
            extension.setPower(-0.2);
            state=0;
        }
        else if(state!=1&&state!=3&&state!=4&&state!=5&&state!=6) extension.setPower(0);
        if(gamepad.a) {
            extension.setTargetPosition(15);
            state=1;
        }
        if(gamepad.x){
            state=4;
            extension.setTargetPosition(60);
        }
        if(gamepad.y){
            state=5;
            extension.setTargetPosition(130);
        }
        if(gamepad.b){
            state=6;
            extension.setTargetPosition(250);
        }
        if(Intake.a){
            state=3;
            extension.setTargetPosition(750);
        }
        if(Intake.b){
            state=1;
        }
        StateMachine();
        ;
    }
    public String ReturnPosition(){
        return String.valueOf(extension.getCurrentPosition());
    }
    private int ConvertFromBool(boolean button){
        if(button){
            return 1;
        }
        else return 0;
    }
    private void StateMachine(){

        if(state==1) {
            if (extension.getCurrentPosition() > 15)
                extension.setPower(-0.4);
            else if (extension.getCurrentPosition() <= 15 ) {
                extension.setPower(0);
                state = 2;
                // extension.setTargetPosition(100);
            }
        }
        if(state==3){
            if(extension.getCurrentPosition()<500) {
                extension.setPower(1-extension.getCurrentPosition()/500);
            }
            else{
                extension.setPower(0);
            }
        }
        if(state==4){
            if(extension.getCurrentPosition()<60) {
                extension.setPower(1-extension.getCurrentPosition()/120);
            }
            else{
                extension.setPower(0);
            }
        }
        if(state==5){
            if(extension.getCurrentPosition()<130) {
                extension.setPower(1-extension.getCurrentPosition()/200);
            }
            else{
                extension.setPower(0);
            }
        }
        if(state==6){
            if(extension.getCurrentPosition()<250) {
                extension.setPower(1-extension.getCurrentPosition()/250);
            }
            else{
                extension.setPower(0);
            }
        }
        if(state==2){
            if(extension.getCurrentPosition()<0){
                extension.setPower(0.1);
            }
           else if(extension.getCurrentPosition()>15){
                extension.setPower(-0.1);
            }

        }
    }
}
