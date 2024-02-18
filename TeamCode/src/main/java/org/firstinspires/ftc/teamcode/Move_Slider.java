package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Move_slider")
public class Move_Slider extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    @Override


    public void init() {
        motor1=hardwareMap.get(DcMotor.class,"Intake");
        motor2=hardwareMap.get(DcMotor.class,"nez");

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

            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            if(gamepad1.right_trigger>0){
                motor2.setPower(gamepad1.right_trigger);
            }
            if(gamepad1.left_trigger>0){
                motor2.setPower(-gamepad1.left_trigger);
            }
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}
