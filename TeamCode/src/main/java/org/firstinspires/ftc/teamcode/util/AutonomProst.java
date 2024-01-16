package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Autonprost")

public class AutonomProst extends LinearOpMode {
    Servo I1;
    Servo I2;

    @Override
    public void runOpMode(){
        SampleMecanumDrive voiculescu = new SampleMecanumDrive(hardwareMap);
        I1 = hardwareMap.get(Servo.class,"i1");
        I2=hardwareMap.get(Servo.class,"i2");
        I1.setPosition(1);
        I2.setPosition(-1);
        waitForStart();

        /*Trajectory fata = voiculescu.trajectoryBuilder(new Pose2d(0,0),0)
                .forward(100)
                .build();
        voiculescu.followTrajectory(fata);
        Trajectory stanga = voiculescu.trajectoryBuilder(new Pose2d(100,0),0)
                .strafeLeft(70)
                .build();

         */
        Trajectory splina = voiculescu.trajectoryBuilder(new Pose2d(0,0),0)
                        .splineTo(new Vector2d(120,130),Math.toRadians(60))
                                .build();
        voiculescu.followTrajectory(splina);
        Trajectory splina1 = voiculescu.trajectoryBuilder(new Pose2d(120,70),60)
                .splineToConstantHeading(new Vector2d(120,190),Math.toRadians(0))
                .build();
        voiculescu.followTrajectory(splina1);

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
    }
}
