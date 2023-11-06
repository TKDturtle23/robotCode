package org.firstinspires.ftc.teamcode.autonPackage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonPackage.IK.FABRIK;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Joint;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Vector2;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Drive Tank", group="Exercises")
//@Disabled
public class DriveTank extends LinearOpMode
{
    DcMotor clawRotate,leftMotor, rightMotor, armMotor2;
    DcMotor armMotor1L, armMotor1R; // when looking from behind the bot
    Servo  clawGrabL, clawGrabR;
    float   leftY, rightY;
    boolean   clawGrabState;
    boolean   prevClawGrapState;
    double baseArmPower = -0.30;
    boolean prevdPad2_down, prevdPad2_up;
    // called when init button is  pressed
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("motorLeft");
        rightMotor = hardwareMap.dcMotor.get("motorRight");

        clawRotate = hardwareMap.dcMotor.get("clawRotate");

        armMotor1L = hardwareMap.dcMotor.get("armMotor1L");
        armMotor1R = hardwareMap.dcMotor.get("armMotor1R");
        armMotor2 = hardwareMap.dcMotor.get("armMotor1");



//        armMotor1L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets encoders
//        armMotor1R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        //clawRotate = hardwareMap.servo.get("clawRotate");
        clawGrabL = hardwareMap.servo.get("clawGrabL");
        clawGrabR = hardwareMap.servo.get("clawGrabR");
        clawGrabL.scaleRange(0.0, 1.0);
        clawGrabR.scaleRange(0.0, 1.0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // instead of -forward, we are just reversing the motor direction
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        clawGrabL.setDirection(Servo.Direction.FORWARD);
        clawGrabR.setDirection(Servo.Direction.REVERSE); // other side so reverse direction



        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

//        List<Joint> joints = new ArrayList<Joint>();
//        List<Double> armLengths = new ArrayList<Double>();
//        Joint start = new Joint(), mid = new Joint(), end = new Joint();
//        start.location = new Vector2(0.0, 0.0); // always 0, 0
//        mid.location = new Vector2(0.0, 0.0e); // TODO: set to the starting location
//        end.location = new Vector2(0.0, 0.0e); // TODO: set to the starting location
//
//        armLengths.add(0.0e); // arm part 1 TODO: measure arm and set to correct length
//        armLengths.add(0.0e); // arm part 2 TODO: measure arm and set to correct length
//        FABRIK IK = new FABRIK(joints, armLengths);
        Vector2 endPosition = new Vector2(0.0, 0.0); // TODO: set correct end position
        waitForStart();

//        armMotor1L.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets encoders to go to a specific tick TODO: test the ticks per motor
//        armMotor1R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        long last_time = System.nanoTime();




        while (opModeIsActive())
        {

            //double deltaTime;
            long time = System.nanoTime();
            double deltaTime = ((time - last_time) / 1000000) / 1000;
            last_time = time;

            float forward = gamepad1.left_stick_y;
            float LOR = gamepad1.right_stick_x;
            double armV, armH ;


            float armPower = 0.40f;

            float clawRotPower = 0.35f; // does not actually change the power to the servo, just in programming the values sent
            float clawRot = 0.0f; // actual rotation per call
            float power = 0.6f;

            clawRot = (gamepad1.left_trigger - gamepad1.right_trigger * clawRotPower);


            if (gamepad1.b) { // movement speed control
                power = 1.0f; // if gamepad1 b is down then speed
            } else if(gamepad1.a) {
                power = 0.3f; // if gamepad1 a is up then slow
            }
//            if (gamepad1.dpad_down) { // arm rotation
//                armV = armPower; // if dpad1 is down then rotate [a direction]
//            } else if(gamepad1.dpad_up) {
//                armV = -armPower; // if dpad1 is up then rotate [a direction]
//            } else {
//                armV = 0.0f; // if dpad1 is not up or down then do nothing [might want to add a small amount of back just in case. also add a locking mechanism for power]
//            }
            armV = (gamepad2.left_stick_y * armPower) + baseArmPower; // up/down
            armH = (gamepad2.right_stick_y * armPower) + baseArmPower; // forward/backward
            if (gamepad1.x) {
                clawGrabState = true;
            }
            if (gamepad1.y) {
                clawGrabState = false;
            }
            if (gamepad2.dpad_up && baseArmPower >= -0.60f ) {
                baseArmPower -= 0.05f * deltaTime;
            } else if (gamepad2.dpad_down && baseArmPower <= 0.40f ) {
                baseArmPower += 0.05f * deltaTime;
            }

            clawRotate.setPower(Range.clip(clawRot, -1.0f, 1.0f));
            //if (clawGrabState != prevClawGrapState) { // claw grab
            clawGrabL.setPosition(Range.clip((clawGrabState?0.57:0.0), 0.1f, 1.0f) );
            clawGrabR.setPosition(Range.clip(clawGrabState?0.52:0.0, 0.1f, 1.0f));
            //}

            leftY = (float)(forward * power * (LOR +1)); // power for the left motor(s); again, instead of -forward, we are just reversing the motor direction
            rightY = (float)(forward * power * (-LOR + 1)); // power for the right motor(s)
            armMotor2.setPower(Range.clip(armV, -1.0, 1.0));
            leftMotor.setPower(Range.clip(leftY, -1.0, 1.0)); // again, instead of -forward, we are just reversing the motor direction
            rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left: " + leftY + "  right: " + rightY + "  armPower: " + armV);
            telemetry.addData("powers", "baseArm: " + baseArmPower + "  servo's spot: " + clawGrabState + "  test: " + deltaTime);
            telemetry.update();


            prevdPad2_down = gamepad2.dpad_down; // updating previous inputs
            prevdPad2_up = gamepad2.dpad_up;
            idle();
        }
    }
}