package org.firstinspires.ftc.teamcode.autonPackage;

import static java.lang.Math.ceil;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.time.LocalTime;

@TeleOp(name="Drive Tank", group="Exercises")
//@Disabled
public class DriveTank extends LinearOpMode
{
    DcMotor clawRotate,leftMotor, rightMotor, armMotor1;
    Servo  clawGrabL, clawGrabR;
    float   leftY, rightY;
    boolean   clawGrabState;
    boolean   prevClawGrapState;
    float baseArmPower = -0.25f;
    boolean prevdPad2_down, prevdPad2_up;
    // called when init button is  pressed
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("motorLeft");
        rightMotor = hardwareMap.dcMotor.get("motorRight");
        armMotor1 = hardwareMap.dcMotor.get("armMotor1");
        clawRotate = hardwareMap.dcMotor.get("clawRotate");


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

        waitForStart();
        long last_time = System.nanoTime();




        while (opModeIsActive())
        {

            //double deltaTime;
            long time = System.nanoTime();
            double deltaTime = (int) ((time - last_time) / 1000000) / 1000;
            last_time = time;

            float forward = gamepad1.left_stick_y;
            float LOR = gamepad1.right_stick_x;
            float armRot ;


            float armPower = 0.40f;

            float clawRotPower = 0.35f; // does not actually change the power to the servo, just in programming the values sent
            float clawRot = 0.0f; // actual rotation per call
            float power = 0.7f;

            clawRot = (gamepad1.left_trigger - gamepad1.right_trigger * clawRotPower);


            if (gamepad1.b) { // movement speed control
                power = 1.0f; // if gamepad1 b is down then speed
            } else if(gamepad1.a) {
                power = 0.3f; // if gamepad1 a is up then slow
            } else {
                power = 0.7f; // if none then normal
            }

//            if (gamepad1.dpad_down) { // arm rotation
//                armRot = armPower; // if dpad1 is down then rotate [a direction]
//            } else if(gamepad1.dpad_up) {
//                armRot = -armPower; // if dpad1 is up then rotate [a direction]
//            } else {
//                armRot = 0.0f; // if dpad1 is not up or down then do nothing [might want to add a small amount of back just in case. also add a locking mechanism for power]
//            }
            armRot = (gamepad2.left_stick_y * armPower) + baseArmPower;
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
            armMotor1.setPower(Range.clip(armRot, -1.0, 1.0));
            leftMotor.setPower(Range.clip(leftY, -1.0, 1.0)); // again, instead of -forward, we are just reversing the motor direction
            rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left: " + leftY + "  right: " + rightY + "  armPower: " + armRot);
            telemetry.addData("powers", "baseArm: " + baseArmPower + "  servo's spot: " + clawGrabState + "  test: " + deltaTime);
            telemetry.update();


            prevdPad2_down = gamepad2.dpad_down; // updating previous inputs
            prevdPad2_up = gamepad2.dpad_up;
            idle();
        }
    }
}