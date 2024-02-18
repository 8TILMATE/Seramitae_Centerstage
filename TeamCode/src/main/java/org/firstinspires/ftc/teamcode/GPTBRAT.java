package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "testGPTBR")
public class GPTBRAT extends OpMode {

    Servo armServo;
    public float leftstickx;
    public float leftsticky;
    public float pivot;
    private ElapsedTime timer = new ElapsedTime();

    // Define servo position constants
    private static final double POSITION_0 = 0.0;
    private static final double POSITION_0_5 = 0.5;
    private static final double POSITION_1 = 1.0;

    // Define motion profile parameters
    private static final double MOTION_DURATION_SECONDS = 4.0;
    private static final int LOOP_HZ = 50;
    private static final double ACCELERATION = 0.5;
    private static final double DECELERATION = 0.5;

    private double startTime;
    private double initialSpeed;
    public boolean exit;
    @Override
    public void init() {
        // Initialize hardware
        armServo = hardwareMap.get(Servo.class, "i1");

        // Set the initial position of the servo
        armServo.setPosition(POSITION_0);
        // Record the start time when OpMode starts
        startTime = getRuntime();
        initialSpeed = 0.0;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Check if button 'A' is pressed
        boolean isPressed = false;

        if (gamepad1.a  && !isPressed ) {
            isPressed = true;
            //moveToPosition(POSITION_1);
            double targetPosition=POSITION_1;
            exit=true;
            new Thread (){

                @Override
                public void run() {

                        // Calculate the progress ratio

                        double progress = (getRuntime() - startTime) / MOTION_DURATION_SECONDS;

                        // Calculate current speed using acceleration and deceleration
                        double currentSpeed;
                        while(progress<1&&exit) {
                            if (progress < 0.5) {
                                currentSpeed = initialSpeed + 2 * ACCELERATION * progress;
                            } else {
                                double decelerationProgress = 2 * (progress - 0.5);
                                currentSpeed = initialSpeed + 2 * DECELERATION * decelerationProgress;
                            }

                            // Limit speed to prevent overshooting
                            currentSpeed = Math.min(currentSpeed, 1.0);

                            // Interpolate servo position based on current speed
                            double currentPosition = armServo.getPosition();

                            // Update the position using the calculated speed
                            currentPosition += currentSpeed * (targetPosition - currentPosition) / LOOP_HZ;

                            // Set the servo position
                            armServo.setPosition(currentPosition);

                            // Store current speed for the next iteration
                            initialSpeed = currentSpeed;
                        }
                        // Check if the motion profile is complete
                        if (progress >= 1.0) {
                            armServo.setPosition(targetPosition);
                            startTime = getRuntime(); // Reset start time for the next motion profile
                            telemetry.addData("dute:"," in plm robert");
                            exit=false;
                        }

                    }

            }. start();

        }
        // Check if button 'B' is pressed
        if (gamepad1.b) {
            //moveToPosition(POSITION_1);
            double targetPosition=POSITION_0_5;
            exit=true;
            new Thread (){

                @Override
                public void run() {
                    // Calculate the progress ratio

                    double progress = (getRuntime() - startTime) / MOTION_DURATION_SECONDS;

                    // Calculate current speed using acceleration and deceleration
                    double currentSpeed;
                    while(progress<1&&exit) {
                        if (progress < 0.5) {
                            currentSpeed = initialSpeed + 2 * ACCELERATION * progress;
                        } else {
                            double decelerationProgress = 2 * (progress - 0.5);
                            currentSpeed = initialSpeed + 2 * DECELERATION * decelerationProgress;
                        }

                        // Limit speed to prevent overshooting
                        currentSpeed = Math.min(currentSpeed, 1.0);

                        // Interpolate servo position based on current speed
                        double currentPosition = armServo.getPosition();

                        // Update the position using the calculated speed
                        currentPosition += currentSpeed * (targetPosition - currentPosition) / LOOP_HZ;

                        // Set the servo position
                        armServo.setPosition(currentPosition);

                        // Store current speed for the next iteration
                        initialSpeed = currentSpeed;
                        if(armServo.getPosition()==POSITION_0_5){
                            break;
                        }
                    }
                    // Check if the motion profile is complete
                    if (progress >= 1.0) {
                        armServo.setPosition(targetPosition);
                        startTime = getRuntime(); // Reset start time for the next motion profile
                        telemetry.addData("dute:"," in plm robert");

                        exit=false;
                    }

                }

            }.start();


        }


        // Wait to maintain the loop frequency
        while (getRuntime() - startTime < 1.0 / LOOP_HZ) {
            // Continue looping until the desired time has passed
        }
        startTime = getRuntime(); // Reset the start time for the next iteration
    }

    private void moveToPosition(double targetPosition) {
        // Calculate the progress ratio

        double progress = (getRuntime() - startTime) / MOTION_DURATION_SECONDS;

        // Calculate current speed using acceleration and deceleration
        double currentSpeed;
        if (progress < 0.5) {
            currentSpeed = initialSpeed + 2 * ACCELERATION * progress;
        } else {
            double decelerationProgress = 2 * (progress - 0.5);
            currentSpeed = initialSpeed + 2 * DECELERATION * decelerationProgress;
        }

        // Limit speed to prevent overshooting
        currentSpeed = Math.min(currentSpeed, 1.0);

        // Interpolate servo position based on current speed
        double currentPosition = armServo.getPosition();

        // Update the position using the calculated speed
        currentPosition += currentSpeed * (targetPosition - currentPosition) / LOOP_HZ;

        // Set the servo position
        armServo.setPosition(currentPosition);

        // Store current speed for the next iteration
        initialSpeed = currentSpeed;

        // Check if the motion profile is complete
        if (progress >= 1.0) {
            armServo.setPosition(targetPosition);
            startTime = getRuntime(); // Reset start time for the next motion profile
        }

    }

    @Override
    public void stop() {
        // Perform any necessary cleanup when OpMode stops
    }
}