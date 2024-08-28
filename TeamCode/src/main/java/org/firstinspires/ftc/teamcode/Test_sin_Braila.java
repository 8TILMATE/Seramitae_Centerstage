package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.fasterxml.jackson.databind.util.LRUMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "testbrat" +
        "Braila")
public class Test_sin_Braila extends OpMode {

    DcMotor motor1;
    DcMotor motor2;
    private ElapsedTime runtime = new ElapsedTime();
    private double servoPosition;
    private double servoDelta = 0.01;
    public int ticks=537;
    public int rotations=0;
    private double servoDelayTime = 0.02;

    //DistanceSensor  stanga,dreapta;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    public int toggler=1;
    long setTime = System.currentTimeMillis();
    long setTime1 = System.currentTimeMillis();
    private int state=0;
    private int stage=0;
    Servo Left;
    Servo Right;
    Servo Inch;
    public static double poz0=0;
    private int targetpos=0,targetpos1=0;
    DcMotor LR;
    DcMotor LF;
    DcMotor RF;
    DcMotor RR;

   public boolean hasRun = true;
    boolean hasRun1 = true;


    @Override

    public void init() {
        //I2= hardwareMap.get(Servo.class,"Jugule");
        //I1 = hardwareMap.get(Servo.class,"i1");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I3 = hardwareMap.get(Servo.class,"i3");
        LF=hardwareMap.get(DcMotor.class,"LF");
        LR =hardwareMap.get(DcMotor.class,"LR");
        RF=hardwareMap.get(DcMotor.class,"RF");
        RR=hardwareMap.get(DcMotor.class,"RR");
        motor1=hardwareMap.get(DcMotor.class,"Intake");
        motor2=hardwareMap.get(DcMotor.class,"Spool");
        Left=hardwareMap.get(Servo.class,"i1");
        Right=hardwareMap.get(Servo.class,"i2");
        Inch=hardwareMap.get(Servo.class,"scor");
        //motor3=hardwareMap.get(DcMotor.class,"RF");
        //motor4=hardwareMap.get(DcMotor.class,"RR");
        //intake=hardwareMap.get(DcMotor.class,"Intake");
       // stanga=hardwareMap.get(DistanceSensor.class,"stanga");
        //dreapta=hardwareMap.get(DistanceSensor.class,"dreapta");
        //pula=hardwareMap.get(DcMotor.class,"Spool");
        //Brat=hardwareMap.get(DcMotor.class,"Brat");
        //Brat2=hardwareMap.get(DcMotor.class,"Brat2");
        telemetry.addData("dute:"," in plm robert");
        telemetry.update();
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetpos=motor2.getCurrentPosition();
        Inch.setPosition(1);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //4000 ticks for full
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
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Ticks",motor2.getCurrentPosition());
        telemetry.addData("State",stage);
        telemetry.addData("M1",motor1.getCurrentPosition());
        leftstickx=gamepad1.left_stick_x/1.25f;
        leftsticky=gamepad1.left_stick_y/1.25f;
        pivot=gamepad1.right_stick_x/3f;
        double denominator = Math.max(Math.abs(leftstickx)+Math.abs(leftsticky)+ Math.abs(pivot),1);
        LF.setPower((pivot+ -leftsticky+leftstickx)/denominator);
        LR.setPower((pivot+ -leftsticky-leftstickx)/denominator);
        RF.setPower((-pivot+ -leftsticky-leftstickx)/denominator);
        RR.setPower((-pivot+ -leftsticky+leftstickx)/denominator);
        telemetry.addData("leftsticky:", String.valueOf(-leftsticky));
        telemetry.addData("leftstickx:", String.valueOf(leftstickx));
        //telemetry.update();
        //motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //if(gamepad1.left_trigger>0){
          //      motor1.setPower(-gamepad1.left_trigger/2);
        //}
        //else if(gamepad1.right_trigger>0){
          //  motor1.setPower(gamepad1.right_trigger/2);
        //}
       // else motor1.setPower(0);
        if(gamepad2.a){
            Left.setPosition(1);
            Right.setPosition(-1);
        }
        if(gamepad2.b){
            Left.setPosition(0);
            Right.setPosition(1);
        }
        if(gamepad2.x){
            Inch.setPosition(1);
        }
        if(gamepad2.y){
            Inch.setPosition(poz0);
        }
        if (gamepad1.left_bumper&&state==0&&stage<4) {
                state=1;
                targetpos=motor2.getCurrentPosition()+1008;
                stage++;
                //rotations+=3;
            } else if (gamepad1.right_bumper&&state==0&&stage>0) {
                state=-1;
                targetpos=motor2.getCurrentPosition()-1008;
                stage--;

                //rotations-=3;

            }// else if (gamepad1.a) {
               // motor2.setPower(0.3);
                //motor2.setTargetPosition(0);
                //motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              //  rotations = 0;
            //} else {
               // motor2.setPower(0);

            //}
        if(gamepad1.a){
            state=5;
            motor1.setTargetPosition(200);

            motor2.setTargetPosition(2016);
            stage=2;
            targetpos1=200;
            targetpos=2016;
        }
        if(gamepad1.b){
            state=6;
            motor1.setTargetPosition(200);
            motor2.setTargetPosition(0);
            targetpos1=200;
            targetpos=0;
        }
        if(gamepad1.x){
            state=7;
            motor1.setTargetPosition(2000);
            targetpos1=2000;
        }
        if(gamepad1.y){
            state=8;
            motor1.setTargetPosition(200);
            targetpos1=200;
        }
        if(stage==0&&motor2.getCurrentPosition()<-15&&state==0){
            state=2;
            motor2.setTargetPosition(1);
        }
        if(stage==0&&motor2.getCurrentPosition()>15&&state==0){
            state=3;
            motor2.setTargetPosition(5);
        }


        if(state==1){
            if(motor2.getCurrentPosition()<targetpos){
               motor2.setPower(0.8);

            }
            else{
                motor2.setPower(0);
                state=0;
            }

        }
        if(state==-1){
            if(motor2.getCurrentPosition()>targetpos){
                motor2.setPower(-0.8);

            }
            else{
                motor2.setPower(0);
                state=0;
            }
        }
        if(state==2){
            if(motor2.getCurrentPosition()<1){
                motor2.setPower(0.2);
            }
            else{
                motor2.setPower(0f);
                state=0;
            }
        }
        if(state==3){
            if(motor2.getCurrentPosition()>5){
                motor2.setPower(-0.2);
            }
            else{
                motor2.setPower(0f);
                state=0;
            }
        }
        if(state==5){
            if(motor1.getCurrentPosition()<targetpos1){
                motor1.setPower(1f);

            }
            else{
                Left.setPosition(0.01);
                Right.setPosition(0.01);
                motor1.setPower(0);
                Inch.setPosition(poz0);
                if(motor2.getCurrentPosition()<targetpos){
                    motor2.setPower(0.8f);
                }
                else{
                    motor2.setPower(0);
                    state=0;
                }
            }
        }
        if(state==6){
            if(motor2.getCurrentPosition()>targetpos){
                motor2.setPower(-0.8);

            }
            else{
                motor2.setPower(0);
                Inch.setPosition(1);
                if(motor1.getCurrentPosition()<targetpos1){
                    motor1.setPower(1f);
                }
                else{
                    motor1.setPower(0f);
                    state=0;

                }
            }
        }
        if(state==7){
            if(motor1.getCurrentPosition()<targetpos1){
                motor1.setPower(1);
            }
            else{
                motor1.setPower(0);
                state=0;
            }
        }
        if(state==8){
            if(motor1.getCurrentPosition()>targetpos1){
                motor1.setPower(-1);
            }
            else{
                motor1.setPower(0);
                state=0;
            }
        }
    }

}
