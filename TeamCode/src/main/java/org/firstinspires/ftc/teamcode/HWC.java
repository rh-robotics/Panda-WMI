package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class HWC {
    // Declare empty variables for robot hardware
    public DcMotorEx leftFront, rightFront, leftBack, rightBack, leftLift, rightLift, armFlipper;
    public CRServo clawFlipper;
    public Servo clawWrist, claw;
    public int cameraMonitorViewId;

    public TwinRobotComponents liftComponents;
    public RobotComponents armFlipperComponent;

    // CV vars
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    SleeveDetection sleeveDetection = new SleeveDetection(145,168,30,50);

    // Declare other variables to be used here
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftLift = hardwareMap.get(DcMotorEx.class,"leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        armFlipper = hardwareMap.get(DcMotorEx.class, "armFlipper");

        //declare all arm components with PID values, 435rpm motors have 384.5 ppr, 60rpm has 2786.2 ppr multiplied by gear ratio
        liftComponents = new TwinRobotComponents (leftLift, rightLift, 384.5, 0.005, 0.2, 0.0008, 0.07);
        armFlipperComponent = new RobotComponents (armFlipper, 2786.2 * 24, 0.008, 0.0, 0.0000, 0);

        // Declare servos
        clawFlipper = hardwareMap.get(CRServo.class, "clawFlipper");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // Camera
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        armFlipper.setDirection(DcMotorEx.Direction.FORWARD);

        //Sets the wheels to break on zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set CRServo Directions
        claw.setPosition(0.25);
        clawFlipper.setPower(0);
        clawWrist.setPosition(0.0);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Resets encoder position to zero
        armFlipper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    public boolean closeEnough (int pos, int target, int range) {
        if ((target - range <= pos) && (target + range >= pos)) return true;
        return false;
    }

    public void manualDrive(double leftStickY, double leftStickX, double rightStickX, double rightTrigger) {
        //robot too fast and Jack bad at driving
        double turnSpeed = 1 - rightTrigger + 0.2;
        double drive = -leftStickY * .8;
        double strafe = -leftStickX * .8;
        double turn = rightStickX * .6 * turnSpeed;

        double leftFPwr;
        double rightFPwr;
        double leftBPwr;
        double rightBPwr;

        //calculate drive pwr
        if (drive != 0 || turn != 0) {
            leftFPwr = Range.clip(drive + turn, -1.0, 1.0);
            rightFPwr = Range.clip(drive - turn, -1.0, 1.0);
            leftBPwr = Range.clip(drive + turn, -1.0, 1.0);
            rightBPwr = Range.clip(drive - turn, -1.0, 1.0);
        } else if (strafe != 0) {
            /* Strafing */
            leftFPwr = -strafe;
            rightFPwr = strafe;
            leftBPwr = strafe;
            rightBPwr = -strafe;
        } else {
            leftFPwr = 0;
            rightFPwr = 0;
            leftBPwr = 0;
            rightBPwr = 0;
        }
        leftFront.setPower(leftFPwr);
        leftBack.setPower(leftBPwr);
        rightFront.setPower(rightFPwr);
        rightBack.setPower(rightBPwr);
    }

    public void cv() {
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                FtcDashboard.getInstance().startCameraStream(camera, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

}