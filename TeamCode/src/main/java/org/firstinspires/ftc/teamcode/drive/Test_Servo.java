package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servo")
public class Test_Servo extends OpMode {
    Servo Inch;
    Servo Br1;
    Servo Br2;

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;DcMotor intake;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    public int toggler=1;

    @Override

    public void init() {
        Inch= hardwareMap.get(Servo.class,"Incheietura");
        motor1=hardwareMap.get(DcMotor.class,"LF");
        motor2=hardwareMap.get(DcMotor.class,"LR");
        motor3=hardwareMap.get(DcMotor.class,"RF");
        motor4=hardwareMap.get(DcMotor.class,"RR");
        intake=hardwareMap.get(DcMotor.class,"Intake");
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

        leftstickx=gamepad1.left_stick_x;
        leftsticky=gamepad1.left_stick_y;
        pivot=gamepad1.right_stick_x/0.75f;
        double denominator = Math.max(Math.abs(leftstickx)+Math.abs(leftsticky)+ Math.abs(pivot),1);
        motor1.setPower((pivot+ -leftsticky+leftstickx)/denominator);
        motor2.setPower((pivot+ -leftsticky-leftstickx)/denominator);
        motor3.setPower((-pivot+ -leftsticky-leftstickx)/denominator);
        motor4.setPower((-pivot+ -leftsticky+leftstickx)/denominator);
        telemetry.addData("leftsticky:", String.valueOf(-leftsticky));
        telemetry.addData("leftstickx:", String.valueOf(leftstickx));


        if(gamepad1.dpad_up){
            motor1.setPower((0.32f)/denominator);
            motor2.setPower((0.32f)/denominator);
            motor3.setPower((0.32f)/denominator);
            motor4.setPower((0.32f)/denominator);
        }
        if(gamepad1.dpad_down){
            motor1.setPower((-0.32f)/denominator);
            motor2.setPower((-0.32f)/denominator);
            motor3.setPower((-0.32f)/denominator);
            motor4.setPower((-0.32f)/denominator);
        }
        if(gamepad1.dpad_right){
            motor1.setPower((+0.32)/denominator);
            motor2.setPower((-0.32)/denominator);
            motor3.setPower((-0.32)/denominator);
            motor4.setPower((+0.32)/denominator);
        }
        if(gamepad1.dpad_left){
            motor1.setPower((-0.32)/denominator);
            motor2.setPower((+0.32)/denominator);
            motor3.setPower((+0.32)/denominator);
            motor4.setPower((-0.32)/denominator);
        }
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad1.a){
            Inch.setPosition(1);
        }
        if(gamepad1.b){
            Inch.setPosition(-1);
        }
    }
}
