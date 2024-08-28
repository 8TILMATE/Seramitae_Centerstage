package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionEasy.albastru;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.analysis.function.Tan;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

@Config
@Autonomous(name = "RedNear", group = "Autonomous")

public class AutoTrial extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static  String TFOD_MODEL_ASSET = "Model.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static  String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Model1.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Prop",
    };
    private TfodProcessor tfod;




    private VisionPortal visionPortal;
    public static int SptSH=16;
    public static  int Heading=-150;
    public static int Tangent=0;
    public static int ParkX=3,ParkY=20;
    public static int AlignX=5,AlignY=0;
    Claw claw = new Claw();
    Extension extension = new Extension();
    Arm arm = new Arm();
    @Override
    public void runOpMode() {

        claw.Init(hardwareMap);
        arm.Init(hardwareMap);
        extension.Init(hardwareMap);
        initTfod();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Action right1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(33,33),Math.toRadians(90))
                .build();
        Action right2 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(180))
                .lineToXConstantHeading(8)
                .strafeToConstantHeading(new Vector2d(11,-4))
                .build();
        Action left1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(34,25),Math.toRadians(90))
                .waitSeconds(1)
                .lineToYConstantHeading(23)
                //.splineToSplineHeading(new Pose2d(43,45,Heading),Math.toRadians(SptSH))
                //.waitSeconds(2)
                //.strafeTo(new Vector2d(30,35))
                .build();

        Action left2 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(Heading))
                .waitSeconds(1)
                .lineToYConstantHeading(SptSH)
                .waitSeconds(0.2)
                //.strafeTo(new Vector2d(0,10))
                .build();
        Action end = drive.actionBuilder(drive.pose)
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(ParkX,ParkY))
                .build();
        Action mid1 = drive.actionBuilder(drive.pose)
                .stopAndAdd(claw.wristDown())
                .lineToXConstantHeading(38)
                .stopAndAdd(claw.depositPurple())
                .waitSeconds(0.5)
                .stopAndAdd(claw.OuttakeWrist())
                .splineToLinearHeading(new Pose2d(10,35,Math.toRadians(270)),0)
                .strafeToConstantHeading(new Vector2d(17,42))
                //.splineToConstantHeading(new Vector2d(10,40),0)
                .build();

            TFOD_MODEL_ASSET="Model1.tflite";
            TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Model1.tflite";

        while(opModeInInit()){
            List<Recognition> currentRecognitions;
            currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.size()==1) {
                double x = (tfod.getRecognitions().get(0).getLeft() + tfod.getRecognitions().get(0).getRight()) / 2;
                double y = (tfod.getRecognitions().get(0).getTop() + tfod.getRecognitions().get(0).getBottom() / 2);
//cazurile respective
                if(x<200){

                }
                else if(x>200&&x<500){

                }
            }
            else{

            }
            sleep(500);

        }

        waitForStart();
        List<Recognition> currentRecognitions;
        currentRecognitions = tfod.getRecognitions();
        if(currentRecognitions.size()==1) {
            double x = (tfod.getRecognitions().get(0).getLeft() + tfod.getRecognitions().get(0).getRight()) / 2;
            double y = (tfod.getRecognitions().get(0).getTop() + tfod.getRecognitions().get(0).getBottom() / 2);

            if (x < 200) {
                telemetry.addData("Caz","Stanga");
                Actions.runBlocking(
                        new SequentialAction(
                                claw.wristDown(),
                                left1,
                                new SleepAction(0.5),
                                claw.depositPurple(),
                                new SleepAction(0.5),
                                claw.closePurple(),
                                left2
                        ));

                Actions.runBlocking(
                        new SequentialAction(
                                arm.raiseArm(),
                                new SleepAction(0.5),
                                extension.extend(),
                                new SleepAction(0.1),
                                claw.OuttakeWrist(),
                                new SleepAction(0.5),
                                claw.depositYellow(),
                                new SleepAction(0.5),
                                claw.closeYellowandWristDown(),
                                new SleepAction(0.4),
                                extension.deextend(),
                                arm.lowerArm(),
                                new SleepAction(2),
                                end
                        )

                );
            }
             else if (x > 200 && x < 500) {
                telemetry.addData("Caz","Mijloc");
                Actions.runBlocking(
                        new SequentialAction(
                                mid1,
                                arm.raiseArm(),
                                new SleepAction(0.5),
                                extension.extend(),
                                new SleepAction(0.1),
                                claw.outtakeWristMid(),
                                new SleepAction(0.5),
                                claw.depositYellow(),
                                new SleepAction(0.5),
                                extension.deextend(),
                                new SleepAction(0.4),
                                claw.closeYellowandWristDown(),
                                arm.lowerArm(),
                                new SleepAction(2),
                                end
                        )
                );
            } else if (x > 500) {
                telemetry.addData("Caz","Dreapta");
                Actions.runBlocking(
                        new SequentialAction(
                                claw.wristDown(),
                                right1,
                                new SleepAction(0.5),
                                claw.depositPurple(),
                                new SleepAction(0.5),
                                claw.closePurple(),
                                right2,
                                arm.raiseArm(),
                                new SleepAction(0.5),
                                extension.extend(),
                                new SleepAction(0.1),
                                claw.OuttakeWrist(),
                                new SleepAction(0.5),
                                claw.depositYellow(),
                                new SleepAction(0.5),
                                extension.deextend(),
                                new SleepAction(0.4),
                                claw.closeYellowandWristDown(),
                                arm.lowerArm(),
                                new SleepAction(2),
                                end

                        )
                );
            }
             else{
                Actions.runBlocking(
                        new SequentialAction(
                                claw.wristDown(),
                                right1,
                                new SleepAction(0.5),
                                claw.depositPurple(),
                                new SleepAction(0.5),
                                claw.closePurple(),
                                right2,
                                arm.raiseArm(),
                                new SleepAction(0.5),
                                extension.extend(),
                                new SleepAction(0.1),
                                claw.OuttakeWrist(),
                                new SleepAction(0.5),
                                claw.depositYellow(),
                                new SleepAction(0.5),
                                extension.deextend(),
                                new SleepAction(0.4),
                                claw.closeYellowandWristDown(),
                                arm.lowerArm(),
                                new SleepAction(2),
                                end

                        )
                );
            }
        }
                        /*
                        while (motor1.getCurrentPosition() < 100) {
                            motor1.setPower(0.1f);
                        }
                        motor1.setPower(0);
                        Trajectory fata3 = mecanumDrive.trajectoryBuilder(new Pose2d(0, 0), 0)
                                .forward(57).build();
                        mecanumDrive.followTrajectory(fata3);
                        sleep(100);
                        mecanumDrive.turn(Math.toRadians(-35));
                        Trajectory fata4 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(10).build();
                        mecanumDrive.followTrajectory(fata4);
                        right.setPosition(0.4);                        sleep(1000);
                        right.setPosition(0.0);
                        Trajectory spate1= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(10).build();
                        mecanumDrive.followTrajectory(spate1);
                        mecanumDrive.turn(Math.toRadians(35));
                        sleep(1000);
                        Trajectory back = mecanumDrive.trajectoryBuilder(new Pose2d(57, 0), 0)
                                .back(53).build();
                        mecanumDrive.followTrajectory(back);PPivot.setPosition(1);
                        sleep(100);
                        Trajectory strafe = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(strafevalue).build();
                        mecanumDrive.followTrajectory(strafe);
                        sleep(100);
                        mecanumDrive.turn(Math.toRadians(-70));
                        Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(120).build();
                        mecanumDrive.followTrajectory(rotate);
                        while(motor1.getCurrentPosition()<500){
                            motor1.setPower(0.8);

                        }
        // instantiate your MecanumDrive at a particular pose.

        /*
        left
        Actions.runBlocking(
                new SequentialAction(
                        claw.wristDown(),
                        left1,
                        new SleepAction(0.5),
                        claw.depositPurple(),
                        new SleepAction(0.5),
                        claw.closePurple(),
                        left2
                ));

        Actions.runBlocking(
                new SequentialAction(
                        arm.raiseArm(),
                        new SleepAction(0.5),
                        extension.extend(),
                        new SleepAction(0.1),
                        claw.OuttakeWrist(),
                        new SleepAction(0.5),
                        claw.depositYellow(),
                        new SleepAction(0.5),
                        claw.closeYellowandWristDown(),
                        new SleepAction(0.4),
                        extension.deextend(),
                        arm.lowerArm(),
                        new SleepAction(1),
                        end
                )

        );
        */

        /*
        right
        Actions.runBlocking(
                new SequentialAction(
                claw.wristDown(),
                right1,
                new SleepAction(0.5),
                claw.depositPurple(),
                new SleepAction(0.5),
                claw.closePurple(),
                right2,
                        arm.raiseArm(),
                        new SleepAction(0.5),
                        extension.extend(),
                        new SleepAction(0.1),
                        claw.OuttakeWrist(),
                        new SleepAction(0.5),
                        claw.depositYellow(),
                        new SleepAction(0.5),
                        extension.deextend(),
                        new SleepAction(0.4),
                        claw.closeYellowandWristDown(),
                        arm.lowerArm(),
                        new SleepAction(1),
                        end

                )
        );
         */
        //mid
        /*
        Actions.runBlocking(
                new SequentialAction(
                    mid1,
                        arm.raiseArm(),
                        new SleepAction(0.5),
                        extension.extend(),
                        new SleepAction(0.1),
                        claw.outtakeWristMid(),
                        new SleepAction(0.5),
                        claw.depositYellow(),
                        new SleepAction(0.5),
                        extension.deextend(),
                        new SleepAction(0.4),
                        claw.closeYellowandWristDown(),
                        arm.lowerArm(),
                        new SleepAction(1),
                        end
                )
        );

         */
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);

        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.6f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
}
