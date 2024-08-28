package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;


@TeleOp(name = "testmuiesin." +
        "_sin")
public class Test_Muie_Sin extends OpMode {
    Servo Inch;
    Servo I1;
    Servo I2;
    Servo I3;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;DcMotor intake;
    DcMotor pula;
    DcMotor Brat;
    DcMotor Brat2;
    private ElapsedTime runtime = new ElapsedTime();
    private double servoPosition;
    private double servoDelta = 0.01;
    private double servoDelayTime = 0.02;

    //DistanceSensor  stanga,dreapta;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    public int toggler=1;
    long setTime = System.currentTimeMillis();
    long setTime1 = System.currentTimeMillis();


    boolean hasRun = true;
    boolean hasRun1 = true;


    @Override

    public void init() {
        I2= hardwareMap.get(Servo.class,"Jugule");
        I1 = hardwareMap.get(Servo.class,"i1");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I3 = hardwareMap.get(Servo.class,"i3");
        motor1=hardwareMap.get(DcMotor.class,"LF");
        motor2=hardwareMap.get(DcMotor.class,"LR");
        motor3=hardwareMap.get(DcMotor.class,"RF");
        motor4=hardwareMap.get(DcMotor.class,"RR");
        intake=hardwareMap.get(DcMotor.class,"Intake");
       // stanga=hardwareMap.get(DistanceSensor.class,"stanga");
        //dreapta=hardwareMap.get(DistanceSensor.class,"dreapta");
        pula=hardwareMap.get(DcMotor.class,"Spool");
        Brat=hardwareMap.get(DcMotor.class,"Brat");
        Brat2=hardwareMap.get(DcMotor.class,"Brat2");
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

        leftstickx=gamepad1.left_stick_x/1.25f;
        leftsticky=gamepad1.left_stick_y/1.25f;
        pivot=gamepad1.right_stick_x/3f;
        double denominator = Math.max(Math.abs(leftstickx)+Math.abs(leftsticky)+ Math.abs(pivot),1);
        motor1.setPower((pivot+ -leftsticky+leftstickx)/denominator);
        motor2.setPower((pivot+ -leftsticky-leftstickx)/denominator);
        motor3.setPower((-pivot+ -leftsticky-leftstickx)/denominator);
        motor4.setPower((-pivot+ -leftsticky+leftstickx)/denominator);
        telemetry.addData("leftsticky:", String.valueOf(-leftsticky));
        telemetry.addData("leftstickx:", String.valueOf(leftstickx));

        // telemetry.addData("Dreapta:",String.valueOf(dreapta.getDistance(DistanceUnit.CM)));
        // telemetry.addData("Stanga:",String.valueOf(stanga.getDistance(DistanceUnit.CM)));



        if(gamepad1.dpad_up){
            motor1.setPower((0.52f)/denominator);
            motor2.setPower((0.52f)/denominator);
            motor3.setPower((0.52f)/denominator);
            motor4.setPower((0.52f)/denominator);
        }
        if(gamepad1.dpad_down){
            motor1.setPower((-0.52f)/denominator);
            motor2.setPower((-0.52f)/denominator);
            motor3.setPower((-0.52f)/denominator);
            motor4.setPower((-0.52f)/denominator);
        }
        if(gamepad1.dpad_right){
            motor1.setPower((+0.52)/denominator);
            motor2.setPower((-0.52)/denominator);
            motor3.setPower((-0.52)/denominator);
            motor4.setPower((+0.52)/denominator);
        }
        if(gamepad1.dpad_left){
            motor1.setPower((-0.52)/denominator);
            motor2.setPower((+0.52)/denominator);
            motor3.setPower((+0.52)/denominator);
            motor4.setPower((-0.52)/denominator);
        }
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pula.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motor",String.valueOf(pula.getPower()));
        Brat2.setDirection(DcMotorSimple.Direction.REVERSE);
        Brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Brat2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*if(gamepad1.a){
            I2.setPosition(1);
            I1.setPosition(-1);
        }
        if(gamepad1.b){
            I2.setPosition(-1);
            I1.setPosition(1);
        }
        if(gamepad1.a&&hasRun1==true){
            if(toggler==1){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }
            toggler=-toggler;
            hasRun1=false;
        }
        if(gamepad1.b){
            if(toggler==1){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }
            toggler=-toggler;
        }
        */
        //Brat2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pula.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(gamepad2.a)
        {
            I1.setPosition(0);
            //I2.setPosition(.7f);
        }
        if(gamepad2.b){
            I1.setPosition(1f);
            //I2.setPosition(.3f);

        }
        if(gamepad2.x){
            I1.setPosition(1f);
            I2.setPosition(1f);
        }
        if(gamepad1.right_trigger>0) {
            Brat2.setPower(gamepad1.right_trigger);
            Brat.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0){
            Brat2.setPower(-gamepad1.left_trigger);
            Brat.setPower(-gamepad1.left_trigger);

        }
        else{
            Brat.setPower(0);
            Brat2.setPower(0);

        }
        if(gamepad1.left_bumper){
            pula.setPower(-0.3f);

        }
        if(gamepad1.right_bumper){
            pula.setPower(0.3f);


        }

        if(gamepad2.y)
        {
            ElapsedTime time1=new ElapsedTime();
            time1.startTime();
            if(time1.time()>0.1) {
                I1.setPosition(I1.getPosition() + 0.05);
            }
        }

       /* if(gamepad1.guide){
            Brat.setTargetPosition(0);//1500
            Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Brat.setPower(1);

        }


        if(gamepad1.a){
            intake.setPower(1);
        }
        if(gamepad1.b){
            intake.setPower(0);
        }
        /*
        if(gamepad1.x){
            I1.setPosition(-1);
            //I2.setPosition(0.47);

        }

         */
        if(gamepad2.left_trigger>0){
            intake.setPower(-gamepad2.left_trigger);
        }
        if(gamepad2.right_trigger>0){
            intake.setPower(gamepad2.right_trigger);
        }
        else{
            intake.setPower(0);
        }
        /*
        if(gamepad1.a){
            I3.setPosition(0.5);
        }
        if(gamepad2.b){
            I3.setPosition(1);
        }
        if (System.currentTimeMillis() - setTime > 500 && !hasRun){
            I1.setPosition(-0.9);
            hasRun = true;
        }
        if (System.currentTimeMillis() - setTime1 > 500 && !hasRun1){
            hasRun1 = true;
        }
        */

    }

}
