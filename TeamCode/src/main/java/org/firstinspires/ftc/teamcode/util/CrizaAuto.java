package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "CRIZA")

public class CrizaAuto extends LinearOpMode {

    DcMotor m1;
    Servo I1;
    Servo Inch;
    @Override
    public void runOpMode(){
        SampleMecanumDrive dt = new SampleMecanumDrive(hardwareMap);
        m1= hardwareMap.get(DcMotor.class,"Intake");
        I1=hardwareMap.get(Servo.class,"i1");
        Inch= hardwareMap.get(Servo.class,"Incheietura");
        waitForStart();

        /*Trajectory fata = voiculescu.trajectoryBuilder(new Pose2d(0,0),0)
                .forward(100)
                .build();
        voiculescu.followTrajectory(fata);
        Trajectory stanga = voiculescu.trajectoryBuilder(new Pose2d(100,0),0)
                .strafeLeft(70)
                .build();

         */


        /*Trajectory splina = voiculescu.trajectoryBuilder(new Pose2d(0,0),0)
                .splineToConstantHeading(new Vector2d(100, 100), Math.toRadians(0))
                .build();
        voiculescu.followTrajectory(splina);
        Trajectory splina2= voiculescu.trajectoryBuilder(new Pose2d(100,100),0)
                .splineToSplineHeading(new Pose2d(140, 140, Math.toRadians(90)), Math.toRadians(0))
                .build();
        voiculescu.followTrajectory(splina2);
        I1.setPosition(-1);
        I2.setPosition(1);

         */
        Trajectory fr= dt.trajectoryBuilder(new Pose2d(55,0))
                .forward(10)
                .build();
        dt.followTrajectory(fr);
        Trajectory stg2 = dt.trajectoryBuilder(new Pose2d(55, 0), 30)
                .strafeLeft(140)
                .build();
        dt.followTrajectory(stg2);
        dt.turn(Math.toRadians(90));
        m1.setPower(1);
        I1.setPosition(0.5f);
        Inch.setPosition(1);

        sleep(4000);
        m1.setPower(0);
        telemetry.addData("Kick some ass sergiu","(si tu victor) salut si lui mihai");
    }
}
