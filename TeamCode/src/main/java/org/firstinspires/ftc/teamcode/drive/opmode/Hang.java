package org.firstinspires.ftc.teamcode.drive.opmode;
//package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Hang
{
    DcMotor hangstanga;
    DcMotor hangdreapta;

    public void HangInit(HardwareMap hardwareMap){
        hangstanga= hardwareMap.get(DcMotor.class,"HangStanga");
        hangstanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangdreapta= hardwareMap.get(DcMotor.class,"HangDreapta");
        hangdreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void HangSetPower(HardwareMap hardwareMap, Gamepad gamepad){
        hangstanga.setPower(0+ gamepad.right_trigger-gamepad.left_trigger);
        hangdreapta.setPower(0- gamepad.right_trigger+gamepad.left_trigger);
    }
}
