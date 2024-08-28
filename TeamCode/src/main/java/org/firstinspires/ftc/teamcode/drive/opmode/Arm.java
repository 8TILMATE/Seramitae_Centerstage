package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.annotation.Target;

public class  Arm {
    private PIDController controller;
    public static double p=0.00085,i=0.0008,d=0.0001,f=0.01;
    public static int intakepos,outhigh,outlow;
    private final double ticks_in_rotations=2800/180;
    int state=0;
    private int targetpos=10;
    public boolean isIntaking=false,isOuttaking=false;
    public DcMotor Arm;
    public void Init(HardwareMap hardwareMap){
        Arm = hardwareMap.get(DcMotor.class,"Brat");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        controller=new PIDController(p,i,d);
    }
    public class RaiseArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setTargetPosition(150);
            while(Arm.getCurrentPosition()<150){
                Arm.setPower(0.9);
            }
            Arm.setPower(0.15);

            return false;
        }

    }
    public Action raiseArm(){
        return  new RaiseArm();
    }
    public class RaiseArmMid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setTargetPosition(150);
            while(Arm.getCurrentPosition()<150){
                Arm.setPower(0.9);
            }
            Arm.setPower(0.2);

            return false;
        }

    }
    public Action raiseArmMid(){
        return  new RaiseArmMid();
    }
    public class LowerArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){

            Arm.setPower(-0.5);

            return false;
        }

    }
    public Action lowerArm(){
        return  new LowerArm();
    }


        public Thread Pidf = new Thread(){

            @Override
            public void run() {
                while(true){
                    int targetpos=250;
                    controller.setPID(p,i,d);
                    int armpos=Arm.getCurrentPosition();
                    double pid = controller.calculate(armpos,targetpos);
                    double ff = Math.cos(Math.toRadians(targetpos/ticks_in_rotations))*f;
                    double power = pid+ff;
                    Arm.setPower(power);
                }

            }

        };



    public void UpdateArm(Gamepad gamepad,Gamepad gamepad2)
    {

        controller.setPID(p,i,d);
        int armpos=Arm.getCurrentPosition();
            if(gamepad.a){
                Intake();
            }
            if(gamepad2.a){
                Transfer();
            }
            if(gamepad2.x){
                Outtake(150);
            }
            if(gamepad.b){
                Transfer();
            }
            if(gamepad2.y){
                Outtake(250);
            }
            if(gamepad2.b){
                Outtake(350);
            }
            if(gamepad.dpad_left){
                AdjustPos(-10);
            }
            if(gamepad.dpad_right){
                AdjustPos(10);
            }
            if(gamepad2.back){
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        double pid = controller.calculate(armpos,targetpos);
        double ff = Math.cos(Math.toRadians(targetpos/ticks_in_rotations))*f;
        double power = pid+ff;

        Arm.setPower(power);
    }
    private void Intake(){
        targetpos=10;
        isIntaking=true;
        isOuttaking=false;
    }
    private void Outtake(int value){
        targetpos=value;
        isIntaking=false;
        isOuttaking=true;
    }
    private void Transfer(){
        targetpos=40;
        isIntaking=false;
        isOuttaking=false;
    }
    private void AdjustPos(int param){
        targetpos+=param;
    }
    public String ReturnTicks(){
        return String.valueOf(Arm.getCurrentPosition());
    }
}
