package org.firstinspires.ftc.teamcode.autonPackage;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonPackage.IK.FABRIK;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Joint;
import org.firstinspires.ftc.teamcode.autonPackage.IK.Vector2;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.apriltag.AprilTagDetection; // april tag stuff

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class autonStage extends LinearOpMode{
    DcMotorEx clawRotate,leftMotor, rightMotor;
    DcMotorEx armMotor1L, armMotor1R, armMotor2; // when looking from behind the bot
    Servo  clawGrabL, clawGrabR;
    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.autonPackage.aprilTagDetectionPipeline aprilTagDetectionPipeline;
    float   leftY, rightY;
    boolean   clawGrabState;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // You will need to do your own calibration!
    double fx = 0.0; // TODO: calibrate whatever camera we are using
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    // UNITS ARE METERS
    double tagsize = 0.166;

    static int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    static AprilTagDetection tagOfInterest = null;

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
    // gets
    public void getDetections(org.firstinspires.ftc.teamcode.autonPackage.aprilTagDetectionPipeline pipeline, ArrayList<AprilTagDetection> currentDetections) {
        currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == ID_TAG_OF_INTEREST)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

    }

    @SuppressLint("DefaultLocale") // just hiding warnings
    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
    @Override
    public void runOpMode() throws InterruptedException {
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




        armMotor1L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1L.setVelocity(100);
        armMotor1R.setVelocity(100);
        armMotor2.setVelocity(200);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new aprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT); // TODO: get camera
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = new ArrayList<>();
            getDetections(aprilTagDetectionPipeline, currentDetections);
            if (tagOfInterest != null) {
                // TODO: implement movements based on location of april tags
            }
        }
    }
}
