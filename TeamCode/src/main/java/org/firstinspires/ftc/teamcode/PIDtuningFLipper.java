package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="PID Flipper Tuning Op", group="Iterative Opmode")

public class PIDtuningFLipper extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private final double TICKS_IN_DEGREES = 2786.2 * (16.0/10.0); //435 rpm has 384.5 PPR, 2786.2 for 60rpm
    String name;

    private DcMotorEx armFlipper;
    @Override
    public void init() {
        controller = new PIDController (p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWC bronto = new HWC(hardwareMap, telemetry);

        armFlipper = hardwareMap.get(DcMotorEx.class, "armFlipper");
        armFlipper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armFlipper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armFlipper.setDirection(DcMotorEx.Direction.FORWARD);

        name = "arm flipper";
//        bronto.frontArm.setDirection(DcMotorEx.Direction.FORWARD);
//        bronto.frontElbow.setDirection(DcMotorEx.Direction.REVERSE);
//        bronto.backElbow.setDirection(DcMotorEx.Direction.FORWARD);
//        bronto.backArm.setDirection(DcMotorEx.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        while (getRuntime() - currentTime < .05) {

        }
//        if (gamepad1.dpad_down) {
//            armFlipper = hardwareMap.get(DcMotorEx.class, "leftLift");
//            name = "leftLift";
//            armFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armFlipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            p = 0; i = 0; d = 0; f = 0;
//        } else if (gamepad1.dpad_up) {
//            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
//            name = "rightLift";
//            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            p = 0; i = 0; d = 0; f = 0;
//        }
        controller.setPID (p,i,d);
        int armPos = armFlipper.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        double power = pid + ff;

        armFlipper.setPower(power);

        telemetry.addData("motor: ", name);
        telemetry.addData("pos", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("power ", power);
        telemetry.addData ("p ", p);
        telemetry.addData ("i ", i);
        telemetry.addData ("d ", d);
        telemetry.addData ("f ", f);
        telemetry.addData("Loop Time", getRuntime() - currentTime);

        telemetry.update();

    }
}