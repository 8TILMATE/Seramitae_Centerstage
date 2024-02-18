package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "testBR")
public class TestBrat extends OpMode {

    ServoImplEx I1;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    private ElapsedTime timer = new ElapsedTime();

    private TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(.8, 1.8);
    //Motion profile, initialize to down
    private TrapezoidProfile armProfile =
            new TrapezoidProfile(constraints, new TrapezoidProfile.State(1, 0),
                    new TrapezoidProfile.State(1, 0)
            );
    @Override


    public void init() {
        I1=hardwareMap.get(ServoImplEx.class,"i1");
        telemetry.addData("dute:"," in plm robert");
        telemetry.update();
        setPosition(0.5);
    }
    private double prevTarget=0.5;
    public void periodic(){


        if(!armProfile.isFinished(timer.seconds())) {
            //Read the current target for the profile
            double position = armProfile.calculate(timer.seconds()).position;

            //Set servo positions
            I1.setPosition(position);

        }


    }
    public void setPosition(double target) {

        //Create a new profile starting from the last position command
        if(prevTarget != target){
            armProfile = new TrapezoidProfile(
                    constraints,
                    new TrapezoidProfile.State(target, 0),
                    new TrapezoidProfile.State(I1.getPosition(), 0)
            );


            //Reset the timer
            timer.reset();
        }

        prevTarget = target;

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

        if(gamepad1.y){
            setPosition(1);
            periodic();

        }
        if(gamepad1.x){
            I1.setPosition(0.5f);
            //I2.setPosition(0.47);

        }

    }
}
