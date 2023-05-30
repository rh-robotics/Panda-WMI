package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Desperate Auton")
public class desperateAuton extends OpMode {
    //Declaring outside classes
    HWC bronto;
    PandaBrain brain;


    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    //int representing the current scoring height
    //bool representing side of robot scoring
    int jctHeight; //0=gnd, 1=low, 2=med, 3=high
    boolean backScoring; //true = delivery from back side, false = delivery from front
    boolean autonVar = false;
    //initialize robotState and observer
    Observer terry;
    Observer.RobotStates robotState;

    @Override
    public void init() {

        bronto = new HWC(hardwareMap, telemetry);
        terry = new Observer(bronto);
        brain = new PandaBrain(bronto);

        telemetry.addData("Status: ", "Initializing");
        //stop and reset and set to run w/o encoder
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set driving motor directions
        bronto.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //set arm motor directions
        bronto.frontArm.setDirection(DcMotorSimple.Direction.FORWARD);
        bronto.frontElbow.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.backArm.setDirection(DcMotorSimple.Direction.REVERSE);
        bronto.backElbow.setDirection(DcMotorSimple.Direction.FORWARD);

        robotState = Observer.RobotStates.Rest;

        bronto.leftFront.setTargetPosition(bronto.tileTicks);
        bronto.leftRear.setTargetPosition(bronto.tileTicks);
        bronto.rightFront.setTargetPosition(bronto.tileTicks);
        bronto.rightRear.setTargetPosition(bronto.tileTicks);

        jctHeight = 3; //init to high pole
        backScoring = true; //init to back side scoring
        terry.changeScoringSide(backScoring);



        bronto.cv();

        telemetry.addData("Status: ", "Initialized");
            telemetry.addData("Status", "Scanning for Cone");
            telemetry.addData("Parking Position", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
    }


    @Override
    public void start() {runTime.reset();
        robotState = Observer.RobotStates.Drive;}
    @Override
    public void loop() {
        double currentTime = getRuntime();

        bronto.rightRear.setPower(.4);
        bronto.leftRear.setPower(0.4);
        bronto.rightFront.setPower(0.4);
        bronto.leftFront.setPower(0.4);

        if (!bronto.rightFront.isBusy() && !bronto.leftRear.isBusy()){
            robotState = Observer.RobotStates.HighJct;
            terry.giveDriverApproval();
            if (terry.getCompletion()){
                autonVar = true;
            }
        }

    if (autonVar) {
        bronto.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bronto.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotState = Observer.RobotStates.Drive;

        if (bronto.parkingZone == 2) {
            bronto.leftFront.setTargetPosition(0);
            bronto.leftRear.setTargetPosition(0);
            bronto.rightFront.setTargetPosition(0);
            bronto.rightRear.setTargetPosition(0);
        }
        } else if (bronto.parkingZone == 1) {
            bronto.leftFront.setTargetPosition(-bronto.tileTicks);
            bronto.leftRear.setTargetPosition(bronto.tileTicks);
            bronto.rightFront.setTargetPosition(bronto.tileTicks);
            bronto.rightRear.setTargetPosition(-bronto.tileTicks);
        }
        else if (bronto.parkingZone ==3){
            bronto.leftFront.setTargetPosition(bronto.tileTicks);
            bronto.leftRear.setTargetPosition(-bronto.tileTicks);
            bronto.rightFront.setTargetPosition(-bronto.tileTicks);
            bronto.rightRear.setTargetPosition(bronto.tileTicks);
        }
        else{bronto.leftFront.setTargetPosition(0);
            bronto.leftRear.setTargetPosition(0);
            bronto.rightFront.setTargetPosition(0);
            bronto.rightRear.setTargetPosition(0);
    }



        //--------------------------------------- TELEMETRY ------------------------------------//
        telemetry.addLine();
        telemetry.addData("Loop Time w/o tmtry ", getRuntime() - currentTime);
        telemetry.addLine();
        telemetry.addData("Bronto State: ", robotState);
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
        telemetry.addData("frontArm Target", bronto.frontArmComponent.getTarget());
        telemetry.addData("frontArm Pos", bronto.frontArm.getCurrentPosition());
        telemetry.addData("frontArm Pwr", bronto.frontArm.getPower());
        telemetry.addData("frontArm Velocity", bronto.frontArm.getVelocity());
        telemetry.addData("frontArm Angle", bronto.frontArmComponent.getArmAngle());
        telemetry.addLine();
        telemetry.addData("frontElbow Target", bronto.frontElbowComponent.getTarget());
        telemetry.addData("frontElbow Pos", bronto.frontElbow.getCurrentPosition());
        telemetry.addData("frontElbow Pwr", bronto.frontElbow.getPower());
        telemetry.addData("frontElbow Velocity", bronto.frontElbow.getVelocity());
        telemetry.addLine();
        telemetry.addData("backArm Target", bronto.backArmComponent.getTarget());
        telemetry.addData("backArm Pos", bronto.backArm.getCurrentPosition());
        telemetry.addData("backArm Pwr", bronto.backArm.getPower());
        telemetry.addData("backArm Velocity", bronto.backArm.getVelocity());
        telemetry.addData("backArm Angle", bronto.backArmComponent.getArmAngle());
        telemetry.addLine();
        telemetry.addData("backElbow Target", bronto.backElbowComponent.getTarget());
        telemetry.addData("backElbow Pos", bronto.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", bronto.backElbow.getPower());
        telemetry.addData("backElbow Velocity", bronto.backElbow.getVelocity());
        telemetry.addLine();

        telemetry.addLine();
        telemetry.addData("Loop Time w/tmtry ", getRuntime() - currentTime);

        telemetry.update();

    }

}