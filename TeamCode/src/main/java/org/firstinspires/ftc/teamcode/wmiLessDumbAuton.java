package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="wmiSmartAuton", group="Autonomous")
public class wmiLessDumbAuton extends LinearOpMode {

    HWC panda;

    int parkingZone = 2;

    private ElapsedTime runTime = new ElapsedTime();

    private double x = 537.7;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        panda = new HWC(hardwareMap, telemetry);

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

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        panda.cv();

        //let driver know robot is ready
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Zone", panda.sleeveDetection.getPosition());
            telemetry.addData("-----------------------", "");
            telemetry.update();
            parkingZone = panda.sleeveDetection.getPosition();
        }

        //status telemetry
        telemetry.addData("Status", "Running");
        telemetry.update();

        panda.claw.setPosition(1);
        sleep(1000);
        if (parkingZone == 1) {
            panda.rightBack.setPower(.75);
            panda.leftBack.setPower(0.75);
            panda.leftFront.setPower(0.75);
            panda.rightFront.setPower(0.75);
            sleep(1100);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);
            sleep (1000);
            panda.leftFront.setPower(-.75);
            panda.leftBack.setPower(-.75);
            panda.rightFront.setPower(.75);
            panda.rightBack.setPower(.75); //turning for 1 second
            sleep(500);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0); //stop for 1 second
            sleep(1000);
            panda.rightBack.setPower(0.75);
            panda.leftBack.setPower(0.75);
            panda.leftFront.setPower(0.75);
            panda.rightFront.setPower(0.75); //forward for 1 second
            sleep(1000);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0); //stop for 1 second
            sleep (1000);
            panda.leftFront.setPower(.75);
            panda.leftBack.setPower(.75);
            panda.rightFront.setPower(-.75);
            panda.rightBack.setPower(-.75); //turning for 1 second
            sleep(500);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0); //stop for 1 second
        } else if (parkingZone == 3) {
            sleep (15000);
            panda.rightBack.setPower(.75);
            panda.leftBack.setPower(0.75);
            panda.leftFront.setPower(0.75);
            panda.rightFront.setPower(0.75);
            sleep(1000);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);
            sleep (1000);
            panda.leftFront.setPower(.75);
            panda.leftBack.setPower(.75);
            panda.rightFront.setPower(-.75);
            panda.rightBack.setPower(-.75);
            sleep(600);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);
            sleep(1000);
            panda.rightBack.setPower(0.75);
            panda.leftBack.setPower(0.75);
            panda.leftFront.setPower(0.75);
            panda.rightFront.setPower(0.75); //forward for 1 second
            sleep(1000);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0); //stop for 1 second
            sleep (1000);
            panda.leftFront.setPower(-.75);
            panda.leftBack.setPower(-.75);
            panda.rightFront.setPower(.75);
            panda.rightBack.setPower(.75); //turning for 1 second
            sleep(400);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0); //stop for 1 second
        } else {
            panda.rightBack.setPower(.75);
            panda.leftBack.setPower(0.75);
            panda.leftFront.setPower(0.75);
            panda.rightFront.setPower(0.75);
            sleep(1200);
            panda.rightBack.setPower(0);
            panda.leftBack.setPower(0);
            panda.leftFront.setPower(0);
            panda.rightFront.setPower(0);
            sleep (1000);
        }

        }


    }
