package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Move_slider2")
public class Move_Slider2 extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor Intake;
    Servo I1;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    public float indiceputere=1;
    @Override


    public void init() {
        Intake = hardwareMap.get(DcMotor.class,"Intake");
        motor1=hardwareMap.get(DcMotor.class,"nez");
        motor2=hardwareMap.get(DcMotor.class,"Brat");
        I1=hardwareMap.get(Servo.class,"i1");
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

            if(gamepad1.ps){
                if(indiceputere==1){
                    indiceputere=0.1f;
                }
                else{
                    indiceputere=1;
                }
            }
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad1.right_bumper){
                motor2.setPower(1f);
                motor1.setPower(1f);
            }
            if(gamepad1.left_bumper){
                motor2.setPower(-indiceputere);
                motor1.setPower(-indiceputere);
            }
            else{
                motor2.setPower(0.0f);
                motor1.setPower(0.0f);
            }
            if(gamepad1.left_trigger>0){
                Intake.setPower(-gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger>0){
                Intake.setPower(gamepad1.right_trigger);

            }
            else{
                Intake.setPower(0);
            }
        /*
            if(gamepad1.a){
                I1.setPosition(-1);
            }
            if(gamepad1.b){
                I1.setPosition(1);
            }

         */
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
