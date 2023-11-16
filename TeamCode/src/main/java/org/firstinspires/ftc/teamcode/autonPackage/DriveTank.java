package org.firstinspires.ftc.teamcode.autonPackage;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonPackage.IK.FABRIK;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Joint;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Vector2;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Drive Tank", group="Exercises")
//@Disabled
public class DriveTank extends LinearOpMode
{
    DcMotorEx clawRotate,leftMotor, rightMotor;
    DcMotorEx armMotor1L, armMotor1R, armMotor2; // when looking from behind the bot
    Servo  clawGrabL, clawGrabR;
    float   leftY, rightY;
    boolean   clawGrabState;



    //double pi = 3.14159265358979323846264338327950288419716939937510;
    // called when init button is  pressed

    public int degToTicksHD25(double degrees) {
        if (degrees == 0) {
            return 0;
        }
        //double NDegrees = ;
        return (int)Math.floor(950 * (degrees / 360));
        //return 0;
    }
    public int degToTicksCore(double degrees) {
        if (degrees == 0) {
            return 0;
        }
        double NDegrees = degrees / 360;
        return (int)Math.floor(280.0 * (degrees / 360));
        //return 0;
    }
    public int degToTicksBase(double degrees) {
       if (degrees == 0) {
            return 0;
        }
        double NDegrees = degrees / 360;
        return (int)Math.floor(450.0 * (degrees / 360));
    }
    public static double angleBTP(Vector2 p1, Vector2 p2) {
        double dot = (p1.e0 * p2.e0) + (p2.e1 * p1.e1);
        double res = dot / (p1.magnitude() * p2.magnitude());

        return Math.acos(res) * 180.0 / 3.14159265358979323846264338327950288419716939937510; // acos returns radians so converting to degrees
    }
    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = BigDecimal.valueOf(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.get(DcMotorEx.class, "motorLeft");
        rightMotor = hardwareMap.get(DcMotorEx.class,"motorRight");
        clawRotate = hardwareMap.get(DcMotorEx.class,"clawRotate");

        // arm
        armMotor1L = hardwareMap.get(DcMotorEx.class,"armMotor1L");
        armMotor1R = hardwareMap.get(DcMotorEx.class,"armMotor1R");
        armMotor2 = hardwareMap.get(DcMotorEx.class,"armMotor2");
        armMotor1L.setDirection(DcMotor.Direction.REVERSE); // resets encoders
        armMotor1R.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        armMotor1L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets encoders
        armMotor1R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1L.setTargetPosition(0); // resets encoders
        armMotor1R.setTargetPosition(0);
        armMotor2.setTargetPosition(0);
        //claw
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

        List<Joint> joints = new ArrayList<Joint>();
        List<Double> armLengths = new ArrayList<Double>();
        Joint start = new Joint(), mid = new Joint(), end = new Joint();
        start.location = new Vector2(0.0, 0.0); // always 0, 0
        mid.location = new Vector2(10.9, 0.1); //
        end.location = new Vector2(24.9999, 0.0001); //
        joints.add(start);
        joints.add(mid);
        joints.add(end);

        armLengths.add(11.0); // arm part 1
        armLengths.add(15.0); // arm part 2
        FABRIK IK = new FABRIK(joints, armLengths);
        Vector2 endPosition = new Vector2(24.9, 0.0); //
        waitForStart();



        armMotor1L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1L.setVelocity(100);
        armMotor1R.setVelocity(100);
        armMotor2.setVelocity(200);

        long last_time = System.nanoTime();

        while (opModeIsActive())
        {
            //double deltaTime;
            long time = System.nanoTime();

            double deltaTime = ((double)(time - last_time) / 1000000.0) / 1000.0;

            last_time = time;

                // controls
                    float armPower = 1.0f;
                    float power = 0.6f;
                // controller 1
                    float forward = gamepad1.left_stick_y;
                    float LOR = gamepad1.right_stick_x; // left/right
                    boolean turningPositive = false;
                    if (Math.abs(LOR) == LOR) {
                        turningPositive = true;
                    }
                    if (gamepad1.b) { // movement speed control
                         power = 0.8f; // if gamepad1 b is down then speed
                    } else if(gamepad1.a) {
                        power = 0.4f; // if gamepad1 a is up then slow
                    }
                    leftY = (float)(forward * power + (LOR * (turningPositive ? 2.0:1.0))); // power for the left motor(s); again, instead of -forward, we are just reversing the motor direction
                    rightY = (float)(forward * power + (-LOR * (turningPositive ? 1.0:2.0))); // power for the right motor(s)
                 //controller 2
                    float clawRotPower = 0.35f; // does not actually change the power to the servo, just in programming the values sent
                    float clawRot = (gamepad2.left_trigger - gamepad2.right_trigger * clawRotPower);

                    if (gamepad2.x) {
                        clawGrabState = true;
                    }
                    if (gamepad2.y) {
                        clawGrabState = false;
                    }
                    double armForward = gamepad2.right_stick_y;
                    double armUp = gamepad2.left_stick_y;




            endPosition.e1 += (armUp * armPower) * deltaTime; // up/down
            endPosition.e0 += (armForward * armPower) * deltaTime; // forward/backward
            joints = IK.IK(new Vector2(0.0, 0.0), endPosition, 5000L, 0.001);
            clawRotate.setPower(Range.clip(clawRot, -1.0f, 1.0f));
            clawGrabL.setPosition(Range.clip(clawGrabState?0.57:0.0, 0.1f, 1.0f));
            clawGrabR.setPosition(Range.clip(clawGrabState?0.52:0.0, 0.1f, 1.0f));




            int baseArm = degToTicksBase(angleBTP(new Vector2(1.0, 0.0), joints.get(1).location));
            int middleArm = degToTicksHD25(angleBTP (new Vector2(1.0, 0.0), joints.get(2).location.subtract ( joints.get(1).location ) ));
            armMotor1L.setTargetPosition(baseArm);
            armMotor1R.setTargetPosition(baseArm);
            armMotor2.setTargetPosition(middleArm);

            //armMotor2.setPower(Range.clip(armV, -1.0, 1.0));

            leftMotor.setPower(Range.clip(leftY, -1.0, 1.0)); // again, instead of -forward, we are just reversing the motor direction
            rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("Joint Rotations", "base: " + baseArm + "  middle: " + middleArm);
            telemetry.addData("base: "  , round(joints.get(0).location.e0, 1) +  " | " + round(joints.get(0).location.e1, 1));

            telemetry.addData("middle: ", round(joints.get(1).location.e0, 1) +  " | " + round(joints.get(1).location.e1, 1));
            telemetry.addData("end: "   , round(joints.get(2).location.e0, 1) +  " | " + round(joints.get(2).location.e1, 1));
            telemetry.update();



            idle();
        }
    }
}