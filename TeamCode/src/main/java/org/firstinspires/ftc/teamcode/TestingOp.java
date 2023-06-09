package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="WMI TeleOp", group="Iterative Opmode")
public class TestingOp extends OpMode {
    //Declaring outside classes
    HWC panda;

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    @Override
    public void init() {

        panda = new HWC(hardwareMap, telemetry);

        telemetry.addData("Status: ", "Initializing");
        //stop and reset and set to run w/o encoder
        panda.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panda.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panda.armFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set driving motor directions
        panda.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //set arm motor directions
        panda.leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.armFlipper.setDirection(DcMotorSimple.Direction.FORWARD);

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

        if (gamepad1.dpad_right) {

        }

        //------------------------------------ GAMEPAD 2 INPUT ---------------------------------//
        /*
        this is placed AFTER cycle setting since it will override any
        power setting for stuff like intake
         */
        if (gamepad2.left_stick_y != 0) {
            panda.leftLift.setPower(gamepad2.left_stick_y);
            panda.rightLift.setPower(gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_x != 0) {
            panda.armFlipper.setPower(gamepad2.right_stick_x);
        }

        //------------------------------------ DRIVING & MOTOR SETS ----------------------------------//
        //HWC drive pwr and calculations
        panda.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

        //motor pid
        panda.liftComponents.moveUsingPID();
        panda.armFlipperComponent.moveUsingPID();

        //--------------------------------------- TELEMETRY ------------------------------------//
        telemetry.addLine();
        telemetry.addData("Loop Time w/o tmtry ", getRuntime() - currentTime);
        telemetry.addLine();
        telemetry.addData("Left Lift Pos", panda.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift Pos", panda.rightLift.getCurrentPosition());
        telemetry.addData("Arm Flipper Pos", panda.armFlipper.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Loop Time w/tmtry ", getRuntime() - currentTime);
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", panda.frontArm.getCurrentPosition(), panda.backArm.getCurrentPosition(), panda.frontElbow.getCurrentPosition(), panda.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", panda.leftFront, panda.rightFront, panda.leftRear, panda.rightRear, panda.frontArm.getPower(), panda.frontElbow.getPower());
        telemetry.update();

    }

}