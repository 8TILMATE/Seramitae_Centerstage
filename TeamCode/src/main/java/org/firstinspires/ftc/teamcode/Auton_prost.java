package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Auton_prost")
public class Auton_prost extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    public float denominator=1;
    @Override


    public void init() {
        motor1=hardwareMap.get(DcMotor.class,"LF");
        motor2=hardwareMap.get(DcMotor.class,"LR");
        motor3=hardwareMap.get(DcMotor.class,"RF");
        motor4=hardwareMap.get(DcMotor.class,"RR");
        telemetry.addData("dute:"," in plm robert");
        telemetry.update();
    }
    /*
    public void getdata(){
        leftstickx=gamepad1.left_stick_x;
        leftsticky=gamepad1.left_stick_y;
        pivot=gamepad1.right_stick_x;
    }
    */


    /*public void drive(){
        motor1.setPower(pivot+ (-leftsticky-leftstickx));
        motor2.setPower(pivot+ (-leftsticky+leftstickx));
        motor3.setPower(pivot+ (-leftsticky+leftstickx));
        motor4.setPower(pivot+ (-leftsticky-leftstickx));

    }*/
    @Override
    public void loop() {
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam1"))
                .setCameraResolution(new android.util.Size(640,480))
                .build();
        if(processor.getDetections().size()>0){
            AprilTagDetection tag = processor.getDetections().get(0);
            telemetry.addData("x",tag.ftcPose.x);
            telemetry.addData("y",tag.ftcPose.y);
            telemetry.addData("z",tag.ftcPose.z);
            telemetry.addData("dis",tag.ftcPose.range);
            telemetry.addData("unghirob",tag.ftcPose.pitch);
            telemetry.addData("unghitag",tag.ftcPose.yaw);
            if(tag.ftcPose.x>-0.5&&tag.ftcPose.x<0.5) {
                if (tag.ftcPose.y > 6f) {
                    stf();
                } else {
                    motor1.setPower(0.0f);
                    motor2.setPower(0.0f);
                    motor3.setPower(0.0f);
                    motor4.setPower(0.0f);
                }
            }
            else{
                if(tag.ftcPose.x<-0.5){
                    stl();
                }
                else if(tag.ftcPose.x>0.5){
                    str();
                }
            }
        }
        else{
            telemetry.addData("idk",0);
            motor1.setPower(0.0f);
            motor2.setPower(0.0f);
            motor3.setPower(0.0f);
            motor4.setPower(0.0f);
        }
        telemetry.update();
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void stf(){
        motor1.setPower((0.32f)/denominator);
        motor2.setPower((0.32f)/denominator);
        motor3.setPower((0.32f)/denominator);
        motor4.setPower((0.32f)/denominator);
    }
    public void str(){
        motor1.setPower((+0.32)/denominator);
        motor2.setPower((-0.32)/denominator);
        motor3.setPower((-0.32)/denominator);
        motor4.setPower((+0.32)/denominator);
    }
    public void stl(){
        motor1.setPower((-0.32)/denominator);
        motor2.setPower((+0.32)/denominator);
        motor3.setPower((+0.32)/denominator);
        motor4.setPower((-0.32)/denominator);
    }
    public void stb(){
        motor1.setPower((-0.32f)/denominator);
        motor2.setPower((-0.32f)/denominator);
        motor3.setPower((-0.32f)/denominator);
        motor4.setPower((-0.32f)/denominator);

    }

}
