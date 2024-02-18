package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

    /* Copyright (c) 2019 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */


/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RightRed")

public class RedRight extends LinearOpMode {

    Servo I1;
    Servo I2;
    DcMotor pula;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        boolean ver1=true;
        boolean ver2=true;
        boolean run=false;
        pula=hardwareMap.get(DcMotor.class,"Intake");
        pula.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SampleMecanumDrive voiculescu = new SampleMecanumDrive(hardwareMap);
        I1 = hardwareMap.get(Servo.class,"i1");
        I2 = hardwareMap.get(Servo.class,"i2");
        initTfod();
        I2.setPosition(0.5);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {
                telemetryTfod();
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                if(ver1==true) {
                    Trajectory fata1 = voiculescu.trajectoryBuilder(new Pose2d(0, 0), 0)
                            .forward(45)
                            .build();
                    voiculescu.followTrajectory(fata1);
                    sleep(1000);
                    voiculescu.turn(Math.toRadians(30));
                    sleep(1000);
                    currentRecognitions = tfod.getRecognitions();

                }

                if(currentRecognitions.size()==1&&ver1==true) {
                    telemetry.addData("Caz: ", " Stanga");
                    ver1=false;
                    voiculescu.turn(Math.toRadians(5));
                    Trajectory fata2 = voiculescu.trajectoryBuilder(new Pose2d(45, 0), 30)
                            .forward(10)
                            .build();
                    voiculescu.followTrajectory(fata2);
                    sleep(2000);
                    //currentRecognitions = tfod.getRecognitions();

                    I2.setPosition(-1);
                    sleep(500);
                    pula.setPower(-1);
                    sleep(1000);
                    pula.setPower(0);
                    voiculescu.turn(Math.toRadians(-20));
                    Trajectory stg1 = voiculescu.trajectoryBuilder(new Pose2d(55, 0), 30)
                            .strafeRight(50)
                            .build();
                    voiculescu.followTrajectory(stg1);
                    I2.setPosition(0.5);
                    Trajectory stg2 = voiculescu.trajectoryBuilder(new Pose2d(55, 0), 30)
                            .strafeRight(140)
                            .build();
                    voiculescu.followTrajectory(stg2);
                    break;
                }
                else{
                    ver1=false;
                    if(ver2) {
                        voiculescu.turn(Math.toRadians(-40));
                        Trajectory fata2 = voiculescu.trajectoryBuilder(new Pose2d(0, 0), 0)
                                .forward(10
                                ).build();
                        voiculescu.followTrajectory(fata2);
                        sleep(1000);//5000

                        currentRecognitions=tfod.getRecognitions();

                    }
                    if(currentRecognitions.size()==1){
                        telemetry.addData("Caz: ", " Mijlloc");
                        voiculescu.turn(Math.toRadians(10));
                        Trajectory fata3 = voiculescu.trajectoryBuilder(new Pose2d(60, 0), 0)
                                .forward(13).build();
                        voiculescu.followTrajectory(fata3);
                        I2.setPosition(-1);
                        sleep(500);
                        pula.setPower(-0.45);
                        sleep(1500);
                        pula.setPower(0);
                        I2.setPosition(0.5);
                        Trajectory back1 = voiculescu.trajectoryBuilder(new Pose2d(60, 0), 0)
                                .back(50).build();
                        voiculescu.followTrajectory(back1);
                        Trajectory left1 = voiculescu.trajectoryBuilder(new Pose2d(60, 0), 0)
                                .strafeRight(163).build();
                        voiculescu.followTrajectory(left1);
                        break;

                    }
                    else{
                        if(!run) {
                            telemetry.addData("Caz: ", " Dreapta");
                            voiculescu.turn(Math.toRadians(-30));
                            I2.setPosition(-1);
                            sleep(500);
                            pula.setPower(-0.55);
                            sleep(1000);
                            pula.setPower(0);
                            voiculescu.turn(Math.toRadians(30));
                            I2.setPosition(0.5);
                            Trajectory back1 = voiculescu.trajectoryBuilder(new Pose2d(60, 0), 0)
                                    .back(50).build();
                            voiculescu.followTrajectory(back1);
                            Trajectory left1 = voiculescu.trajectoryBuilder(new Pose2d(60, 0), 0)
                                    .strafeRight(163).build();
                            voiculescu.followTrajectory(left1);
                            run=true;

                            break;
                        }
                    }
                    //ver2=false;
                }
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class

