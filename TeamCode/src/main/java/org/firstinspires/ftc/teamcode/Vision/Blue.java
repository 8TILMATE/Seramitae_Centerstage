package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class Blue extends LinearOpMode {


    VisionPortal portal;
    PropDetectionBlueClose processor;
    @Override
    public void runOpMode() throws InterruptedException {

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
    }
}
