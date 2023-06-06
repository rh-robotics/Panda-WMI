package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="WMI TeleOp", group="Iterative Opmode")
public class WMITeleOp extends OpMode {
    //Declaring outside classes
    HWC panda;

    int jctHeight = 3;

    int liftIntakePos = 0;
    int liftLowPos = 250;
    int liftMedPos = 500;
    int liftHighPos = 1000;
    int armFlipperIntakePos = 0;
    int armFlipperDeliveryPos = 500;
    double clawFlipperIntakePos = 0;
    double clawFlipperDeliveryPos = 1;
    double clawWristIntakePos = 0;
    double clawWristDeliveryPos = 1;
    double clawOpenPos = 0;
    double clawClosedPos = 1;

    //targets
    double clawFlipperTarget = 0;
    double clawWristTarget = 0;
    double clawTarget = 0;

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

        if (gamepad1.dpad_down) {
            jctHeight = 1;
        } else if (gamepad1.dpad_right) {
            jctHeight = 2;
        } else if (gamepad1.dpad_up) {
            jctHeight = 3;
        }

        if (gamepad1.x) {
            panda.liftComponents.setTarget(liftIntakePos);
            panda.armFlipperComponent.setTarget(armFlipperIntakePos);
            clawFlipperTarget = clawFlipperIntakePos;
            clawWristTarget = clawWristIntakePos;
            clawTarget = clawOpenPos;
        } else if (gamepad1.y) {
            if (panda.claw.getPosition() == clawOpenPos) {
                panda.claw.setPosition(clawClosedPos);
            } else {
                panda.claw.setPosition(clawOpenPos);
            }
        } else if (gamepad1.b) {
            if (jctHeight == 3) {
                panda.liftComponents.setTarget(liftHighPos);
            } else if (jctHeight == 2) {
                panda.liftComponents.setTarget(liftMedPos);
            } else if (jctHeight == 1) {
                panda.liftComponents.setTarget(liftLowPos);
            }
            panda.armFlipperComponent.setTarget(armFlipperDeliveryPos);
            clawFlipperTarget = clawFlipperDeliveryPos;
            clawWristTarget = clawWristDeliveryPos;
            clawTarget = clawClosedPos;
        }


        //------------------------------------ GAMEPAD 2 INPUT ---------------------------------//
        /*
        this is placed AFTER cycle setting since it will override any
        power setting for stuff like intake
         */
        if (gamepad2.left_stick_y != 0) {
            panda.liftComponents.incrementTarget(gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_x != 0) {
            panda.armFlipperComponent.incrementTarget(gamepad2.right_stick_x);
        }

        //------------------------------------ DRIVING & MOTOR SETS ----------------------------------//
        //HWC drive pwr and calculations
        panda.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

        //motor pid
        panda.liftComponents.moveUsingPID();
        panda.armFlipperComponent.moveUsingPID();

        //servos set
        panda.clawFlipper.setPosition(clawFlipperTarget);
        panda.clawWrist.setPosition(clawWristTarget);
        panda.claw.setPosition(clawTarget);

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