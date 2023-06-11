package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@Disabled
//@TeleOp(name="PID Tuning Op", group="Iterative Opmode")

public class PIDtuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private final double TICKS_IN_DEGREES = 384.5; //435 rpm has 384.5 PPR, 2786.2 for 60rpm
    String name;

    private DcMotorEx leftLift;
    private DcMotorEx rightLift;
    @Override
    public void init() {
        controller = new PIDController (p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWC bronto = new HWC(hardwareMap, telemetry);

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);

        name = "left Lift";
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
        if (gamepad1.dpad_down) {
            leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
            name = "leftLift";
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        } else if (gamepad1.dpad_up) {
            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
            name = "rightLift";
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        }
        controller.setPID (p,i,d);
        int armPos = (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        double power = pid + ff;

        leftLift.setPower(power);
        rightLift.setPower(power);

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