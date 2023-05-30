package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Observing TeleOp", group="Iterative Opmode")
public class ObservingTeleOp extends OpMode {
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

        //------------------------------------ GAMEPAD 2 INPUT ---------------------------------//
        /*
        this is placed AFTER cycle setting since it will override any
        power setting for stuff like intake
         */
        //arm manual increment

        if (gamepad2.left_stick_y != 0) {
            panda.backArmComponent.incrementTarget(gamepad2.left_stick_y * 70);
        } else if (gamepad2.right_stick_y != 0) {
            panda.frontArmComponent.incrementTarget(gamepad2.right_stick_y * 70);
        } else if (gamepad2.left_stick_x != 0) {
            panda.backElbowComponent.incrementTarget(gamepad2.left_stick_x * 15);
        } else if (gamepad2.right_stick_x != 0) {
            panda.frontElbowComponent.incrementTarget(gamepad2.right_stick_x * 15);
        }
        //servo manual control
        if (gamepad2.left_trigger > 0.2) { //triggers are forward, check neg
            panda.backIntakeL.setPower(-gamepad2.left_trigger);
            panda.backIntakeR.setPower(-gamepad2.left_trigger);
        } else if (gamepad2.left_bumper) {
            panda.backIntakeL.setPower(1);
            panda.backIntakeR.setPower(1);
        }
        if (gamepad2.right_trigger > 0.2) {
            panda.frontIntakeL.setPower(-gamepad2.right_trigger);
            panda.frontIntakeR.setPower(-gamepad2.right_trigger);
        } else if (gamepad2.right_bumper) {
            panda.frontIntakeL.setPower(1);
            panda.frontIntakeR.setPower(1);
        } //TODO: check negatives


        //--------------------------------------- DRIVING --------------------------------------//
        //HWC drive pwr and calculations
        panda.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

        //--------------------------------------- TELEMETRY ------------------------------------//
        telemetry.addLine();
        telemetry.addData("Loop Time w/o tmtry ", getRuntime() - currentTime);
        telemetry.addLine();
        telemetry.addData("panda State: ", robotState);
        telemetry.addData("Terry State: ", terry.getRobotState());
        telemetry.addData("Junction Height", jctHeight);
        telemetry.addData("Front Arm State: ", terry.getFrontState());
        telemetry.addData("Back Arm State: ", terry.getBackState());
        telemetry.addData("Back Scoring: ", backScoring);
        telemetry.addData("Front Has Set", terry.getFrontRun());
        telemetry.addData("Back Has Set", terry.getBackRun());
        telemetry.addData("Driver Approval", terry.getDriverApproval());
        telemetry.addData("Front Observer Approval", terry.getFrontApproval());
        telemetry.addData("Back Observer Approval", terry.getBackApproval());
        telemetry.addData("Front State Complete:", terry.frontSide.getCompletionStatus());
        telemetry.addData("Back State Complete:", terry.backSide.getCompletionStatus());
        telemetry.addLine();
        telemetry.addData("frontArm Target", panda.frontArmComponent.getTarget());
        telemetry.addData("frontArm Pos", panda.frontArm.getCurrentPosition());
        telemetry.addData("frontArm Pwr", panda.frontArm.getPower());
        telemetry.addData("frontArm Velocity", panda.frontArm.getVelocity());
        telemetry.addData("frontArm Angle", panda.frontArmComponent.getArmAngle());
        telemetry.addLine();
        telemetry.addData("frontElbow Target", panda.frontElbowComponent.getTarget());
        telemetry.addData("frontElbow Pos", panda.frontElbow.getCurrentPosition());
        telemetry.addData("frontElbow Pwr", panda.frontElbow.getPower());
        telemetry.addData("frontElbow Velocity", panda.frontElbow.getVelocity());
        telemetry.addLine();
        telemetry.addData("backArm Target", panda.backArmComponent.getTarget());
        telemetry.addData("backArm Pos", panda.backArm.getCurrentPosition());
        telemetry.addData("backArm Pwr", panda.backArm.getPower());
        telemetry.addData("backArm Velocity", panda.backArm.getVelocity());
        telemetry.addData("backArm Angle", panda.backArmComponent.getArmAngle());
        telemetry.addLine();
        telemetry.addData("backElbow Target", panda.backElbowComponent.getTarget());
        telemetry.addData("backElbow Pos", panda.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", panda.backElbow.getPower());
        telemetry.addData("backElbow Velocity", panda.backElbow.getVelocity());
        telemetry.addLine();
        telemetry.addData("Auto Cycle", autocycles);
        /*
        telemetry.addData ("front Distance", panda.frontDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData ("back Distance", panda.backDistanceSensor.getDistance(DistanceUnit.CM));
         */
        telemetry.addLine();
        telemetry.addData("Loop Time w/tmtry ", getRuntime() - currentTime);
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", panda.frontArm.getCurrentPosition(), panda.backArm.getCurrentPosition(), panda.frontElbow.getCurrentPosition(), panda.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", panda.leftFront, panda.rightFront, panda.leftRear, panda.rightRear, panda.frontArm.getPower(), panda.frontElbow.getPower());
        telemetry.update();

    }

}