import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Apasa" +
        "")
public class TeleOp extends OpMode {
private PIDController controller;
public static double p=0.00085,i=0.0003,d=0.0001,f=0.01;
public static int intakepos=100,outhigh=1850,outlow=1950;


public static int target=150;

private final double ticks_in_rotations=28*60*(50/15)/180;//50/15
    DcMotor motor1;
    DcMotor motor2;
    DcMotor LR;
    DcMotor LF;
    DcMotor RF;
    DcMotor RR;
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
    Servo left;
    Servo right;
    Servo PPivot;
    Servo launch;
    private int orientation=0;
    long setTime = System.currentTimeMillis();
    long setTime1 = System.currentTimeMillis();
    private int state=0;
    private boolean lansat=false;
    private boolean operable=true;
    private float pos1=0;
    private boolean operable1=true;
    private float pos2=0;
    Servo Drona;
    private int stage=0;
    private int targetpos=0;
   public boolean hasRun = true;
    boolean hasRun1 = true;


    @Override

    public void init() {
        //I2= hardwareMap.get(Servo.class,"Jugule");
        //I1 = hardwareMap.get(Servo.class,"i1");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I2 = hardwareMap.get(Servo.class,"i2");
        //I3 = hardwareMap.get(Servo.class,"i3");
         PPivot=hardwareMap.get(Servo.class,"b");
        left= hardwareMap.get(Servo.class,"a");
        right=hardwareMap.get(Servo.class,"c");
        Drona=hardwareMap.get(Servo.class,"scor");
        //launch=hardwareMap.get(Servo.class,"d");
        motor1=hardwareMap.get(DcMotor.class,"Spool");
        motor2=hardwareMap.get(DcMotor.class,"Intake");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");
        //motor2=hardwareMap.get(DcMotor.class,"LR");
        //motor3=hardwareMap.get(DcMotor.class,"RF");
        //motor4=hardwareMap.get(DcMotor.class,"RR");
        //intake=hardwareMap.get(DcMotor.class,"Intake");
        // stanga=hardwareMap.get(DistanceSensor.class,"stanga");
        //dreapta=hardwareMap.get(DistanceSensor.class,"dreapta");
        //pula=hardwareMap.get(DcMotor.class,"Spool");
        //Brat=hardwareMap.get(DcMotor.class,"Brat");
        //Brat2=hardwareMap.get(DcMotor.class,"Brat2");
        //telemetry.addData("dute:"," in plm robert");
        //telemetry.update();
       motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        PPivot.setPosition(PPivot.getPosition());
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //targetpos=motor2.getCurrentPosition();
        //4000 ticks for full*/
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

        controller.setPID(p,i,d);
        int armpos=motor1.getCurrentPosition();
        if(!operable){
            telemetry.addData("pos1",left.getPosition());

            if(Math.abs(left.getPosition()-pos1)<=0.1){
                operable=true;
            }
        }
        if(!operable1){
            telemetry.addData("pos2",right.getPosition());

            if(Math.abs(right.getPosition()-pos2)<=0.1){
                operable1=true;
            }
        }
        if(gamepad1.a){
            target=intakepos;
            targetpos=666;//2016
            stage=2;
            state=4;
        }
        if(gamepad2.right_bumper){
            Drona.setPosition(0.5);
        }
        if(gamepad2.left_bumper){
            Drona.setPosition(-0.5);
        }
        if(gamepad1.b){
            left.setPosition(0.5);
            //right.setPosition(-1);
            right.setPosition(0.0);
        }
        if(gamepad1.x){
            left.setPosition(0.05);
            right.setPosition(0.4);
        }
        if(gamepad1.left_trigger>0){
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target=motor1.getCurrentPosition();
            motor1.setPower(-gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger>0){
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target=motor1.getCurrentPosition();
            motor1.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.y){
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            target=0;
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(gamepad2.x){
            target=550;
            //launch.setPosition(1);
            lansat=true;
        }
        if(gamepad2.start){
           // target = 1100;
        }
        if(gamepad1.back&&operable){
            if(left.getPosition()==0.05){
                left.setPosition(0.5);
                pos1=0.5f;
            }else{
                left.setPosition(0.05);
                pos1=0.05f;
            }
            operable=false;

        }
        if(gamepad1.start&&operable1){
            if(right.getPosition()==0.4){
                right.setPosition(0.0);
                pos2=0.0f;
            }else{
                right.setPosition(0.4);
                pos2=0.4f;
            }
            operable1=false;
        }
        if(gamepad2.a){
            target=intakepos;
            targetpos=0;
            stage=0;
            state=5;
            PPivot.setPosition(0.5);

            if(motor2.getCurrentPosition()>15){
                orientation=-1;
            }
            else{
                orientation=0;
            }

        }
        if(gamepad2.b){
            target=outhigh;
            PPivot.setPosition(0.45);
        }
        if(gamepad2.y){
            target=outlow;
            PPivot.setPosition(1);

        }
        double pid = controller.calculate(armpos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_rotations))*f;
        double power = pid+ff;

        motor1.setPower(power);
        telemetry.addData("pos",armpos);
        telemetry.addData("target",target);
       // telemetry.addData("extensie",motor2.getCurrentPosition());
        telemetry.update();

        if (gamepad1.left_bumper&&state==0&&stage<4) {
            state=1;
            targetpos=motor2.getCurrentPosition()+333;//1008
            stage++;
            //rotations+=3;
        } else if (gamepad1.right_bumper&&state==0&&stage>0) {
            state=-1;
            targetpos=motor2.getCurrentPosition()-333;//1008
            stage--;

            //rotations-=3;

        }
        if(lansat&&motor1.getCurrentPosition()<600&&motor1.getCurrentPosition()>500){
            launch.setPosition(0);
        }
        if(stage==0&&motor2.getCurrentPosition()<-15&&state==0){
            state=2;
            motor2.setTargetPosition(1);
        }
        if(stage==0&&motor2.getCurrentPosition()>15&&state==0){
            state=3;
            motor2.setTargetPosition(5);
        }


        Drive();
        StateMachineExtendCheck();
    }
    private void Drive()
    {
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        leftstickx=gamepad1.left_stick_x/1.25f;
        leftsticky=gamepad1.left_stick_y/1.25f;
        pivot=gamepad1.right_stick_x/3f;
        double denominator = Math.max(Math.abs(leftstickx)+Math.abs(leftsticky)+ Math.abs(pivot),1);
        LF.setPower((pivot+ -leftsticky+leftstickx)/denominator);
        LR.setPower((pivot+ -leftsticky-leftstickx)/denominator);
        RF.setPower((-pivot+ -leftsticky-leftstickx)/denominator);
        RR.setPower((-pivot+ -leftsticky+leftstickx)/denominator);

        if(gamepad1.dpad_up){
            LF.setPower((0.52f)/denominator);
            LR.setPower((0.52f)/denominator);
            RF.setPower((0.52f)/denominator);
            RR.setPower((0.52f)/denominator);
        }
        if(gamepad1.dpad_down){
            LF.setPower((-0.52f)/denominator);
            LR.setPower((-0.52f)/denominator);
            RF.setPower((-0.52f)/denominator);
            RR.setPower((-0.52f)/denominator);
        }
        if(gamepad1.dpad_right){
            LF.setPower((+0.52f)/denominator);
            LR.setPower((-0.52f)/denominator);
            RF.setPower((-0.52f)/denominator);
            RR.setPower((+0.52f)/denominator);
        }
        if(gamepad1.dpad_left){
            LF.setPower((-0.52f)/denominator);
            LR.setPower((+0.52f)/denominator);
            RF.setPower((+0.52f)/denominator);
            RR.setPower((-0.52f)/denominator);
        }
    }

    private void StateMachineExtendCheck(){
        if(state==0){
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
        if(state==4)//intake pos
        {
            if(motor2.getCurrentPosition()<targetpos){
                motor2.setPower(0.8);
                if(motor2.getCurrentPosition()>=targetpos/2) {
                    PPivot.setPosition(0.5);
                }
            }
            else{
                motor2.setPower(0f);
                state=0;
            }
        }
        if(state==5)//transfer pos
        {
            if(motor2.getCurrentPosition()>targetpos){
                motor2.setPower(-0.8);
                PPivot.setPosition(0.5);
            }
            else{
                motor2.setPower(0f);
                state=0;
            }

        }

    }


}
