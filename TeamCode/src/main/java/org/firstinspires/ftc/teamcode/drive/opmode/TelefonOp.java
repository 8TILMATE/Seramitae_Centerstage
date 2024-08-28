package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.Pair;

@TeleOp(name = "TeleOP")
public class TelefonOp extends OpMode
{
    public static double p=0.00085,i=0.0003,d=0.0001,f=0.01;
    Hang hang = new Hang();
    MecanumDrive mecanumDrive = new MecanumDrive();
    Extension extension = new Extension();
    Arm arm = new Arm();
    Claw claw = new Claw();
    Drone drone = new Drone();
    Pair<Boolean,Boolean> BoolsFromClaw;
    @Override
    public void init(){
        hang.HangInit(hardwareMap);
        mecanumDrive.InitDrivetrain(hardwareMap);
        extension.Init(hardwareMap);
        arm.Init(hardwareMap);
        claw.Init(hardwareMap);
        drone.Init(hardwareMap);
    }
    @Override
    public void loop(){
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hang.HangSetPower(hardwareMap,gamepad2);
        mecanumDrive.Drive(gamepad1);
        extension.UpdateExtension(gamepad2,gamepad1);
        arm.UpdateArm(gamepad1,gamepad2);
        claw.UpdateClaw(gamepad1,gamepad2);
        claw.UpdateParams(arm.isIntaking,arm.isOuttaking);
        telemetry.addData("Extension",extension.ReturnPosition());
        telemetry.addData("Brat",arm.ReturnTicks());
        BoolsFromClaw=claw.SentBoolsBack();
        drone.UpdateDrone(gamepad2);
        telemetry.addData("IsIntaking",BoolsFromClaw.component1());
        telemetry.addData("IsOuttaking",BoolsFromClaw.component2());
    }
}
