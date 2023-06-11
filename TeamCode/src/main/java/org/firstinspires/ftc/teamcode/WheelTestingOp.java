package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name="Wheel Testing Op", group="Iterative Opmode")
public class WheelTestingOp extends OpMode {
    //Declaring outside classes
    HWC panda;

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    @Override
    public void init() {

        panda = new HWC(hardwareMap, telemetry);

        telemetry.addData("Status: ", "Initializing");

        //set driving motor directions
        panda.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        panda.leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        panda.rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        panda.leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        panda.rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void start() {runTime.reset();}
    @Override
    public void loop() {
        double currentTime = getRuntime();
        //------------------------------------ GAMEPAD 1 INPUT ----------------------------------//

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            panda.leftLift.setPower(0);
            panda.rightLift.setPower(0);
            panda.armFlipper.setPower(0);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);
        }



        if (gamepad1.left_stick_y != 0) {
            panda.leftFront.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_x != 0) {
            panda.leftBack.setPower(gamepad1.left_stick_x);
        }
        if (gamepad1.right_stick_y != 0) {
            panda.rightFront.setPower(gamepad1.right_stick_y);
        } else if (gamepad1.right_stick_x != 0) {
            panda.rightBack.setPower(gamepad1.right_stick_x);
        }



        //--------------------------------------- TELEMETRY ------------------------------------//
        telemetry.addLine();
        telemetry.addData("Loop Time w/o tmtry ", getRuntime() - currentTime);
        telemetry.addLine();
        telemetry.addData("Front Left Wheel Pwr", panda.leftFront.getPower());
        telemetry.addData("Front Right Wheel Pwr", panda.rightFront.getPower());
        telemetry.addData("Back Left Wheel Pwr", panda.leftBack.getPower());
        telemetry.addData("Back Right Wheel Pwr", panda.rightBack.getPower());
        telemetry.addLine();
        telemetry.addData("Loop Time w/tmtry ", getRuntime() - currentTime);
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", panda.frontArm.getCurrentPosition(), panda.backArm.getCurrentPosition(), panda.frontElbow.getCurrentPosition(), panda.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", panda.leftFront, panda.rightFront, panda.leftRear, panda.rightRear, panda.frontArm.getPower(), panda.frontElbow.getPower());
        telemetry.update();

    }

}