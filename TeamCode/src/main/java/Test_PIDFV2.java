import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "testbrat" +
        "PIDFV2")
public class Test_PIDFV2 extends OpMode {
private PIDController controller;
public static double p=0,i=0,d=0,f=0;
public static int intakepos,outhigh,outlow;

public static int target;
private final double ticks_in_rotations=28*60*(50/15)/180;//50/15
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

        motor1=hardwareMap.get(DcMotor.class,"Spool");
        motor2=hardwareMap.get(DcMotor.class,"Intake");
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //targetpos=motor2.getCurrentPosition();
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

        controller.setPID(p,i,d);
        int armpos=motor1.getCurrentPosition();
        if(gamepad1.a){
            target=intakepos;
        }
        if(gamepad1.b){
            target=outhigh;
        }
        if(gamepad1.y){
            target=outlow;
        }
        double pid = controller.calculate(armpos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_rotations))*f;
        double power = pid+ff;

        motor1.setPower(power);
        telemetry.addData("pos",armpos);
        telemetry.addData("target",target);
        telemetry.addData("extensie",motor2.getCurrentPosition());
        telemetry.update();
    }

}
