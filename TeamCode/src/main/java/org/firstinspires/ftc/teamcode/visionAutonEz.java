package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class visionAutonEz extends LinearOpMode {
    // Variables
    HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
    HWC.armPositions frontArmPosition = HWC.armPositions.RESTING;
    HWC.armPositions backArmPosition = HWC.armPositions.RESTING;
    ElapsedTime timer = new ElapsedTime();

    // Target Pos
    int frontArmTarget, frontElbowTarget, backArmTarget, backElbowTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        // Tell driver bronto is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Run any initialization code necessary
        HWC bronto = new HWC(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        bronto.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bronto.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bronto.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bronto.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bronto.frontElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.backElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        bronto.frontElbow.setDirection(FORWARD);
        bronto.frontElbow.setDirection(FORWARD);


        // Tell driver bronto is ready and waiting for start
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        bronto.cv();
        while (!isStarted()) {
            telemetry.addData("Status", "Scanning for Cone");
            telemetry.addData("Parking Position", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }


        telemetry.addData("Status", "Running");
        telemetry.update();


        sleep(12000);
        bronto.frontElbow.setTargetPosition(400);
        bronto.backElbow.setTargetPosition(-400);
        bronto.frontElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.backElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.backElbow.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.frontElbow.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.frontElbow.setPower(0.15);
        bronto.backElbow.setPower(0.15);
        sleep(1000);
        bronto.frontElbow.setPower(0.5);
        bronto.backElbow.setPower(0.5);



        if (bronto.parkingZone == 3) {
            bronto.leftFront.setPower(1);
            bronto.leftRear.setPower(-1);
            bronto.rightFront.setPower(-1);
            bronto.rightRear.setPower(1);
            sleep(1500);
            bronto.leftFront.setPower(0);
            bronto.leftRear.setPower(0);
            bronto.rightFront.setPower(0);
            bronto.rightRear.setPower(0);

        } else if (bronto.parkingZone == 1) {
            bronto.leftFront.setPower(-1);
            bronto.leftRear.setPower(1);
            bronto.rightFront.setPower(1);
            bronto.rightRear.setPower(-1);
            sleep(1800);
            bronto.leftFront.setPower(0);
            bronto.leftRear.setPower(0);
            bronto.rightFront.setPower(0);
            bronto.rightRear.setPower(0);
        }

        sleep(1700);

        if (bronto.parkingZone != 0) {
            bronto.leftFront.setPower(0.5);
            bronto.leftRear.setPower(0.5);
            bronto.rightFront.setPower(0.5);
            bronto.rightRear.setPower(0.5);
            sleep(2000);
            bronto.leftFront.setPower(0);
            bronto.leftRear.setPower(0);
            bronto.rightFront.setPower(0);
            bronto.rightRear.setPower(0);


            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", bronto.leftFront.getPower(), bronto.rightFront.getPower(), bronto.leftRear.getPower(), bronto.rightRear.getPower());
            telemetry.update();

        } else {
            bronto.leftFront.setPower(0.5);
            bronto.leftRear.setPower(0.5);
            bronto.rightFront.setPower(0.5);
            bronto.rightRear.setPower(0.5);
            sleep(1000);
            bronto.leftFront.setPower(0);
            bronto.leftRear.setPower(0);
            bronto.rightFront.setPower(0);
            bronto.rightRear.setPower(0);
            telemetry.addData("ParkingZone", "ERROR");
            telemetry.update();
        }
    }
}