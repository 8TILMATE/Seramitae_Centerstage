package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetectionEasy.albastru;

import android.util.Size;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Vision.PropDetectionBlueClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

@Config
@Autonomous(name = "BlueNear", group = "Autonomous")

public class BlueNear extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static  String TFOD_MODEL_ASSET = "Model.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static  String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Model.tflite";
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
        VisionPortal portal;
        claw.Init(hardwareMap);
        arm.Init(hardwareMap);
        extension.Init(hardwareMap);


        PropDetectionBlueClose processor;
            processor = new PropDetectionBlueClose();
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                    .setCameraResolution(new Size(640, 480))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(processor)
                    .enableLiveView(true)
                    .build();
            int detectionCase = 2;

            while(opModeInInit())
            {
                detectionCase= processor.detection;
                telemetry.addData("detection" , detectionCase);
                telemetry.update();
            }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Action right1 = drive.actionBuilder(drive.pose)
                .stopAndAdd(claw.wristDown())
                .lineToXConstantHeading(13)
                .turn(Math.toRadians(-35))
                .stopAndAdd(claw.depositPurple())
                .waitSeconds(0.5)
                .stopAndAdd(claw.OuttakeWrist())
                .turn(Math.toRadians(145))
                .lineToYConstantHeading(-25)
                .build();
        Action right2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(0,1))
                .build();
        Action right3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(0,2))
                .build();
        Action PostLeftumleft = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0,-22))
                .waitSeconds(0.2)
                //.lineToYConstantHeading(-6)

                .build();
        Action left1 = drive.actionBuilder(drive.pose)
               // .lineToXConstantHeading(20)
                .strafeTo(new Vector2d(0,12))
                .turn(Math.toRadians(-40))
                .stopAndAdd(claw.wristDown())
                .stopAndAdd(PostLeftumleft)

                .waitSeconds(0.2)
                .stopAndAdd(claw.depositPurple())
                .waitSeconds(0.5)
                .stopAndAdd(claw.OuttakeWrist())
                .turn(Math.toRadians(40))
                .waitSeconds(0.2)

                //.splineToSplineHeading(new Pose2d(43,45,Heading),Math.toRadians(SptSH))
                //.waitSeconds(2)
                //.strafeTo(new Vector2d(30,35))
                .build();

        Action left2 = drive.actionBuilder(drive.pose)
                .lineToXConstantHeading(-2)
                .turn(Math.toRadians(40))
                .lineToYConstantHeading(-5)
                .waitSeconds(0.1)
                //.turn(Math.toRadians(40))
                //.lineToYConstantHeading(-5)
                //.strafeTo(new Vector2d(0,10))
                .build();
        Action end = drive.actionBuilder(drive.pose)
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(5,-20))
                .build();
        Action mid1 = drive.actionBuilder(drive.pose)
                .lineToXConstantHeading(5)
                .turn(Math.toRadians(150))
                .lineToXConstantHeading(6)
                .stopAndAdd(claw.wristDown())
                .waitSeconds(0.2)
                .stopAndAdd(claw.depositPurple())
                .waitSeconds(0.5)
                .stopAndAdd(claw.OuttakeWrist())
                //.lineToXConstantHeading(5)
                .strafeToConstantHeading(new Vector2d(25,25))
                .waitSeconds(0.1)
                .waitSeconds(0.1)
                .build();
        Action mid2 = drive.actionBuilder(drive.pose)
                //.lineToXConstantHeading(-1.1)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-40))
                .lineToYConstantHeading(1)
                .build();


        //telemetryTfod();

            if(detectionCase==1){
                telemetry.addData("Caz","Stanga");
                Actions.runBlocking(
                new SequentialAction(
                        //Preleft,
                        //claw.wristDown(),
                        left1,
                        new SleepAction(0.5),
                        claw.depositPurple(),
                        new SleepAction(0.5),
                        claw.closePurple(),
                        left2,
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


                ));


            }
            else if(detectionCase==2){
                telemetry.addData("Caz","Mijloc");
                Actions.runBlocking(
                        new SequentialAction(
                                claw.wristDown(),
                                mid1,
                                claw.OuttakeWrist(),
                                mid2,
                                right3,
                                arm.raiseArm(),
                                new SleepAction(0.5),
                                extension.extend(),
                                new SleepAction(0.1),
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
            }

        else{
            telemetry.addData("Caz","Dreapta");
                Actions.runBlocking(
                        new SequentialAction(
                                right1,
                               // right2,
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

        }


        telemetry.addData("Caz","Dreapta");
      /*  Actions.runBlocking(
                new SequentialAction(
                        right1,
          //              right2,
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
        }
//cazurile respective
        // instantiate your MecanumDrive at a particular pose.



        //left

        /*
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
                    right1,
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
        /*mid
        Actions.runBlocking(
                new SequentialAction(
                    mid1,
                        mid2,
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

 //* Add telemetry about TensorFlow Object Detection (TFOD) recognitions.


 private void telemetryTfod() {

 List<Recognition> currentRecognitions = tfod.getRecognitions();
 telemetry.addData("# Objects Detected", currentRecognitions.size());
 FtcDashboard dashboard = FtcDashboard.getInstance();
 TelemetryPacket packet = new TelemetryPacket();
 // Step through the list of recognitions and display info for each one.
 for (Recognition recognition : currentRecognitions) {
 double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
 double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

 telemetry.addData(""," ");
 telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
 telemetry.addData("- Position", "%.0f / %.0f", x, y);
 telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
 if(x<200){
 dashboard.clearTelemetry();

 telemetry.addData("Caz: ", "Stanga");
 packet.put("Caz:","Stanga");
 dashboard.sendTelemetryPacket(packet);

 }

 else if(x>500) {
     dashboard.clearTelemetry();

     telemetry.addData("Caz: ", "Dreapta");
     packet.put("Caz:", "Dreapta");
     dashboard.sendTelemetryPacket(packet);
 }
 else{
 dashboard.clearTelemetry();

 telemetry.addData("Caz: ", "Mijloc");
 packet.put("Caz:","Mijloc");
 dashboard.sendTelemetryPacket(packet);
 }
 dashboard.clearTelemetry();
 }   // end for() loop

 }   // end method telemetryTfod()

 }
