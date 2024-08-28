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
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.

@Autonomous
        (name = "doirosioeri", group = "Concept")

public class doisizerorosu extends LinearOpMode {

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
    DcMotor motor1;
    public int strafevalue=150;
    private PIDController controller;
    public static double p=0.00085,i=0.0003,d=0.0001,f=0.01;
    public static int intakepos=150,outhigh=1950,outlow=1950;
    SampleMecanumDrive mecanumDrive;
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     *
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     *
    private VisionPortal visionPortal;

    public static boolean albastru;
    private final double ticks_in_rotations=28*60*(50/15)/180;//50/15

    Servo left;
    Servo right;
    Servo PPivot;

    @Config
    public static class ServoArm {
        public static boolean albastru1=false;

        public boolean  isAlbastru1() {
            return albastru1;
        }
    }
    Thread target = new Thread(){
        @Override
        public void run(){
            while(opModeIsActive()) {
                int armpos = motor1.getCurrentPosition();

                double pid = controller.calculate(armpos, 1000);
                double ff = Math.cos(Math.toRadians(1000 / ticks_in_rotations)) * f;
                double power = pid + ff;

                motor1.setPower(power);
            }
        }
    };
    @Override
    public void runOpMode() {
        initTfod();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        mecanumDrive= new SampleMecanumDrive(hardwareMap);
        PPivot=hardwareMap.get(Servo.class,"b");
        left= hardwareMap.get(Servo.class,"a");
        right=hardwareMap.get(Servo.class,"c");
        motor1=hardwareMap.get(DcMotor.class,"Spool");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        left.setPosition(1);
        right.setPosition(-1 );
        controller=new PIDController(p,i,d);
        albastru=false;
        if(albastru){
            TFOD_MODEL_ASSET="Model_Albastru_Regio.tflite";
            TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Model_Albastru_Regio.tflite";
        }
        else{
            TFOD_MODEL_ASSET="Model1.tflite";
            TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Model1.tflite";
        }
        while(opModeInInit()){
            List<Recognition> currentRecognitions;
            currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.size()==1) {
                double x = (tfod.getRecognitions().get(0).getLeft() + tfod.getRecognitions().get(0).getRight()) / 2;
                double y = (tfod.getRecognitions().get(0).getTop() + tfod.getRecognitions().get(0).getBottom() / 2);
//cazurile respective
                if(x<200){
                    telemetry.addData("Caz","Stanga");
                }
                else if(x>200&&x<500){
                    telemetry.addData("Caz","Mijloc");
                }
            }
            else{
                telemetry.addData("Caz","Dreapta");
            }
            sleep(500);

        }
        waitForStart();
        ServoArm arm = new ServoArm();
        controller.setPID(p,i,d);
        albastru=arm.isAlbastru1();

        if (opModeIsActive()) {

            PPivot.setPosition(0.5);

            int armpos;
            //target.run();
            //target.
         while (opModeIsActive()) {

          //ridica bratul pentru a vedea camera

                sleep(500);
                telemetry.addData("Wow", "wow");
                //telemetryTfod();
                List<Recognition> currentRecognitions;
                currentRecognitions = tfod.getRecognitions();
                if(currentRecognitions.size()==1) {
                    double x = (tfod.getRecognitions().get(0).getLeft() + tfod.getRecognitions().get(0).getRight()) / 2;
                    double y = (tfod.getRecognitions().get(0).getTop() + tfod.getRecognitions().get(0).getBottom() / 2);
//cazurile respective
                    if(x<200){
                        while(motor1.getCurrentPosition()<170){
                            motor1.setPower(0.1f);
                        }
                        motor1.setPower(0);
                        Trajectory fata3 = mecanumDrive.trajectoryBuilder(new Pose2d(0, 0), 0)
                                .forward(80).build();
                        mecanumDrive.followTrajectory(fata3);
                        mecanumDrive.turn(Math.toRadians(60));
                        Trajectory fata4 = mecanumDrive.trajectoryBuilder(new Pose2d(0, 0), 0)
                                .forward(20).build();
                        mecanumDrive.followTrajectory(fata4);
                        right.setPosition(0.5);
                        sleep(1000);
                        right.setPosition(-1);
                        sleep(1000);
                        mecanumDrive.turn(Math.toRadians(-65));//-57
                        //Trajectory back = mecanumDrive.trajectoryBuilder(new Pose2d(57, 0), 0)
                          //      .back(65).build();
                        // mecanumDrive.followTrajectory(back);
                        PPivot.setPosition(1);
                        sleep(100);
                        Trajectory strafe = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(strafevalue).build();
                        mecanumDrive.followTrajectory(strafe);
                        mecanumDrive.turn(Math.toRadians(-50));//-60
                        Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(50).build();
                        mecanumDrive.followTrajectory(rotate);
                        while(motor1.getCurrentPosition()<500){
                            motor1.setPower(0.8);

                        }
                        motor1.setPower(0);
                        PPivot.setPosition(0.55);
                        Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(60) .build();
                        mecanumDrive.followTrajectory(trajectory);
                        sleep(300);
                        left.setPosition(0.05);
                        sleep(300);
                        Trajectory back2 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(20).build();
                        mecanumDrive.followTrajectory(back2);
                        sleep(100);
                        Trajectory stg =mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(50).build();
                        mecanumDrive.followTrajectory(stg);
                        sleep(100);
                        motor1.setPower(-0.5f);
                        sleep(100);
                        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        motor1.setPower(0);

                        sleep(1000);
                        break;

                    }
                    else if(x>200&&x<500){
                        while(motor1.getCurrentPosition()<100){
                            motor1.setPower(0.1f);
                        }
                        motor1.setPower(0);
                        Trajectory fata3 = mecanumDrive.trajectoryBuilder(new Pose2d(0, 0), 0)
                                .forward(75
                                ).build();
                        mecanumDrive.followTrajectory(fata3);
                        right.setPosition(0.4);                        sleep(500);
                        right.setPosition(0.0);
                        sleep(100);
                        Trajectory back = mecanumDrive.trajectoryBuilder(new Pose2d(57, 0), 0)
                                .back(66).build();
                        mecanumDrive.followTrajectory(back);
                        PPivot.setPosition(1);
                        sleep(100);
                        Trajectory strafe = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(strafevalue).build();
                        mecanumDrive.followTrajectory(strafe);
                        sleep(100);
                        mecanumDrive.turn(Math.toRadians(-70));
                        Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(100).build();
                        mecanumDrive.followTrajectory(rotate);
                        while(motor1.getCurrentPosition()<500){
                            motor1.setPower(0.8);

                        }
                        motor1.setPower(0);
                        PPivot.setPosition(0.55);
                        Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(60) .build();
                        mecanumDrive.followTrajectory(trajectory);
                        sleep(300);
                        left.setPosition(0.05);
                        sleep(300);
                        Trajectory back2 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(20).build();
                        mecanumDrive.followTrajectory(back2);
                        sleep(100);
                        Trajectory stg =mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(100).build();
                        mecanumDrive.followTrajectory(stg);
                        sleep(100);
                        motor1.setPower(-0.5f);
                        sleep(100);
                        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        motor1.setPower(0);

                        sleep(1000);
                        break;
                    }
                    else if(x>500) {
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
                        motor1.setPower(0);
                        PPivot.setPosition(0.55);
                        Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(70) .build();
                        mecanumDrive.followTrajectory(trajectory);
                        sleep(300);
                        left.setPosition(0.05);
                        sleep(300);
                        Trajectory back2 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(20).build();
                        mecanumDrive.followTrajectory(back2);
                        sleep(100);
                        Trajectory stg =mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(120).build();
                        mecanumDrive.followTrajectory(stg);
                        sleep(100);
                        motor1.setPower(-0.5f);
                        sleep(100);
                        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        motor1.setPower(0);

                        sleep(1000);
                        break;
                        *
                        while (motor1.getCurrentPosition() < 130) {
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
                        //Trajectory back = mecanumDrive.trajectoryBuilder(new Pose2d(57, 0), 0)
                        //      .back(53).build();
                        //mecanumDrive.followTrajectory(back);
                        PPivot.setPosition(1);
                        sleep(100);
                        Trajectory strafe = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(strafevalue).build();
                        mecanumDrive.followTrajectory(strafe);
                        sleep(100);
                        mecanumDrive.turn(Math.toRadians(-70));
                        //Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(120).build();
                        //mecanumDrive.followTrajectory(rotate);
                        while(motor1.getCurrentPosition()<500){
                            motor1.setPower(0.8);

                        }
                        motor1.setPower(0);
                        PPivot.setPosition(0.55);
                        Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(40).build();
                        mecanumDrive.followTrajectory(rotate);
                        Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(70) .build();
                        mecanumDrive.followTrajectory(trajectory);

                        sleep(100);
                        left.setPosition(0.05);
                        sleep(100);
                        Trajectory back2 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(20).build();
                        mecanumDrive.followTrajectory(back2);
                        sleep(100);
                        Trajectory stg =mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(50).build();
                        mecanumDrive.followTrajectory(stg);
                        sleep(100);
                        motor1.setPower(-0.5f);
                        sleep(100);
                        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        motor1.setPower(0);

                        sleep(1000);
                        break;
                    }
                    //telemetry.addData("X","y");
                }
                else{
                    while (motor1.getCurrentPosition() < 130) {
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
                    //Trajectory back = mecanumDrive.trajectoryBuilder(new Pose2d(57, 0), 0)
                      //      .back(53).build();
                    //mecanumDrive.followTrajectory(back);
                    PPivot.setPosition(1);
                    sleep(100);
                    Trajectory strafe = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(strafevalue).build();
                    mecanumDrive.followTrajectory(strafe);
                    sleep(100);
                    mecanumDrive.turn(Math.toRadians(-70));
                    //Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(120).build();
                    //mecanumDrive.followTrajectory(rotate);
                    while(motor1.getCurrentPosition()<500){
                        motor1.setPower(0.8);

                    }
                    motor1.setPower(0);
                    PPivot.setPosition(0.55);
                    Trajectory rotate= mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeLeft(40).build();
                    mecanumDrive.followTrajectory(rotate);
                    Trajectory trajectory = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).forward(70) .build();
                    mecanumDrive.followTrajectory(trajectory);

                    sleep(100);
                    left.setPosition(0.05);
                    sleep(100);
                    Trajectory back2 = mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).back(20).build();
                    mecanumDrive.followTrajectory(back2);
                    sleep(100);
                    Trajectory stg =mecanumDrive.trajectoryBuilder(new Pose2d(57,0),0).strafeRight(50).build();
                    mecanumDrive.followTrajectory(stg);
                    sleep(100);
                    motor1.setPower(-0.5f);
                    sleep(100);
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor1.setPower(0);

                    sleep(1000);
                    break;
                }
                    //telemetry.addData("a","a");

                //sleep(2000);
                //telemetryTfod();
            }

            /*
            while (opModeIsActive()) {

                telemetryTfod();

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
        tfod.setMinResultConfidence(0.6f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.


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

           else if(x>500){
                dashboard.clearTelemetry();

                telemetry.addData("Caz: ", "Dreapta");
                packet.put("Caz:","Dreapta");
                dashboard.sendTelemetryPacket(packet);
                if(motor1.getCurrentPosition()>250){
                    motor1.setPower(-0.5f);
                }
                motor1.setPower(0);
            }
           else{
                dashboard.clearTelemetry();

                    telemetry.addData("Caz: ", "Mijloc");
                packet.put("Caz:","Mijloc");
                dashboard.sendTelemetryPacket(packet);
                //if(motor1.getCurrentPosition()>250){
                  //  motor1.setPower(-0.5f);
                //}
                //motor1.setPower(0);

            }
           dashboard.clearTelemetry();
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class

192.168.43.1:5555

             */