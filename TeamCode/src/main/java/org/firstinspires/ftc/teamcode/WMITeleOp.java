package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="WMI TeleOp", group="Iterative Opmode")
public class WMITeleOp extends OpMode {
    //Declaring outside classes
    HWC panda;

    int jctHeight = 3;

    int liftIntakePos = 0;
    int liftLowPos = liftIntakePos;
    int liftMedPos = 1100;
    int liftHighPos = 2000;
    int armFlipperIntakePos = 0;
    int armFlipperLowPos = -1100;
    int armFlipperDeliveryPos = -2400;
    double clawFlipperIntakePos = 0;
    double clawFlipperDunkTime = .3;
    double clawFlipperReverseDunkTime = 1;
    double clawFlipperDunkCompleteTime = 2.5;
    boolean dunking = false;
    double clawWristIntakePos = 0;
    double clawWristDeliveryPos = .68;
    double clawOpenPos = 0.0;
    double clawClosedPos = 1.0;

    //targets
    double clawFlipperPwr = 0;
    double clawWristTarget = 0;
    double clawTarget = 0;

    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    double dunkStartTime;

    @Override
    public void init() {

        panda = new HWC(hardwareMap, telemetry);

        telemetry.addData("Status: ", "Initializing");
        //stop and reset and set to run w/o encoder
        panda.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panda.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panda.armFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set mode
        panda.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        panda.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        panda.armFlipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        //set arm motor directions
        panda.leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        panda.rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        panda.armFlipper.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void start() {runTime.reset();}
    @Override
    public void loop() {
        double currentTime = getRuntime(); //save time at beginning of loop

        //------------------------------------ GAMEPAD 1 INPUT ----------------------------------//

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            panda.leftLift.setPower(0);
            panda.rightLift.setPower(0);
            panda.armFlipper.setPower(0);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);

            panda.clawFlipper.setPower(0);
        }
        //reset clawflipper pwr
        clawFlipperPwr = 0;

        if (gamepad1.dpad_down) {
            jctHeight = 1;
        } else if (gamepad1.dpad_right) {
            jctHeight = 2;
        } else if (gamepad1.dpad_up) {
            jctHeight = 3;
        }

        if (gamepad1.x) { //pressing x to return to intake pos
            panda.liftComponents.setTarget(liftIntakePos);
            panda.armFlipperComponent.setTarget(armFlipperIntakePos);
            clawFlipperPwr = clawFlipperIntakePos;
            clawWristTarget = clawWristIntakePos;
            clawTarget = clawOpenPos;
        } else if (gamepad1.y) { //pressing Y to toggle claw open or close
            clawTarget = clawClosedPos;
        } else if (gamepad1.b) { //pressing B for delivery position, jct height dependent
            if (jctHeight == 1) {
                panda.armFlipperComponent.setTarget(armFlipperLowPos);
                panda.liftComponents.setTarget(liftLowPos);
                clawWristTarget = clawWristIntakePos; //not moving, just straight
            } else {
                if (jctHeight == 3) {
                    panda.liftComponents.setTarget(liftHighPos);
                } else if (jctHeight == 2) {
                    panda.liftComponents.setTarget(liftMedPos);
                }
                panda.armFlipperComponent.setTarget(armFlipperDeliveryPos);
                clawWristTarget = clawWristDeliveryPos;
            }
            clawTarget = clawClosedPos;
        } else if (gamepad1.a) {
            dunkStartTime = getRuntime();
            dunking = true;
            clawFlipperPwr = -1; //pwr claw flipper to move down
        }

        //gamepad A (dunking) functionality
        //if dunking, checks for time elapsed from greatest to least for order
        if (dunking) {
            double elapsed = getRuntime() - dunkStartTime;
            if (elapsed > clawFlipperDunkCompleteTime) {
                clawFlipperPwr = 0; //turn off claw flipper, complete
                dunking = false; //no longer dunking, reset
            } else if (elapsed > clawFlipperReverseDunkTime) {
                clawFlipperPwr = 1; //return claw
            } else if (elapsed > clawFlipperDunkTime){
                clawFlipperPwr = 0; //make claw flipper loose
                clawTarget = clawOpenPos; //open claw to drop
            }
        }


        //------------------------------------ GAMEPAD 2 INPUT ---------------------------------//
        /*
        this is placed AFTER cycle setting since it will override any
        power setting for stuff like intake
         */
        if (gamepad2.left_stick_y != 0) {
            panda.liftComponents.incrementTarget(-10* gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_x != 0) {
            panda.armFlipperComponent.incrementTarget(-10* gamepad2.right_stick_x);
        }
        if (gamepad1.dpad_right) {
            clawTarget += .05;
        } else if (gamepad1.dpad_left) {
            clawTarget -= .05;
        } else if (gamepad1.dpad_up) {
            clawWristTarget += .05;
        } else if (gamepad1.dpad_down) {
            clawWristTarget -= .05;
        } else if (gamepad1.right_bumper) {
            clawFlipperPwr = 1;
        } else if (gamepad1.left_bumper) {
            clawFlipperPwr = -1;
        }

        //------------------------------------ DRIVING & MOTOR SETS ----------------------------------//
        //HWC drive pwr and calculations
        panda.manualDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

        //motor pid
        panda.liftComponents.moveUsingPID();
        panda.armFlipperComponent.moveUsingPID();

        //clip servo range then set pwrs and positions
        clawFlipperPwr = Range.clip(clawFlipperPwr,-1, 1);
        clawWristTarget = Range.clip(clawWristTarget,0, 1);
        clawTarget = Range.clip(clawTarget,0, 1);

        panda.clawFlipper.setPower(clawFlipperPwr);
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
        telemetry.addData("Left Lift Power", panda.leftLift.getPower());
        telemetry.addData("Right Lift Power", panda.rightLift.getPower());
        telemetry.addLine();
        telemetry.addData("Claw Flipper Pos", panda.clawFlipper.getPower());
        telemetry.addData("Claw Wrist Pos", panda.clawWrist.getPosition());
        telemetry.addData("Claw Pos", panda.claw.getPosition());
        telemetry.addLine();
        telemetry.addData("Front Left Wheel Pwr", panda.leftFront.getPower());
        telemetry.addData("Front Right Wheel Pwr", panda.rightFront.getPower());
        telemetry.addData("Back Left Wheel Pwr", panda.leftBack.getPower());
        telemetry.addData("Back Right Wheel Pwr", panda.rightBack.getPower());
        telemetry.addLine();
        telemetry.addData("Front Left Wheel Pwr", panda.leftFront.getVelocity());
        telemetry.addData("Front Right Wheel Pwr", panda.rightFront.getVelocity());
        telemetry.addData("Back Left Wheel Pwr", panda.leftBack.getVelocity());
        telemetry.addData("Back Right Wheel Pwr", panda.rightBack.getVelocity());
        telemetry.addLine();
        telemetry.addData("Loop Time w/tmtry ", getRuntime() - currentTime);
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", panda.frontArm.getCurrentPosition(), panda.backArm.getCurrentPosition(), panda.frontElbow.getCurrentPosition(), panda.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", panda.leftFront, panda.rightFront, panda.leftRear, panda.rightRear, panda.frontArm.getPower(), panda.frontElbow.getPower());
        telemetry.update();

    }

}