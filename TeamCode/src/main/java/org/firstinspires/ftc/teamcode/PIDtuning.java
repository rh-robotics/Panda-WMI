package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="PID Tuning Op", group="Iterative Opmode")

public class PIDtuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private final double TICKS_IN_DEGREES = 2786.2/360; //435 rpm has 384.5 PPR, 2786.2 for 60rpm
    String name;

    private DcMotorEx arm_motor;
    @Override
    public void init() {
        controller = new PIDController (p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWC bronto = new HWC(hardwareMap, telemetry);

        arm_motor= hardwareMap.get(DcMotorEx.class, "backElbow");
        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        name = "back elbow";
        bronto.frontArm.setDirection(DcMotorEx.Direction.FORWARD);
        bronto.frontElbow.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.backElbow.setDirection(DcMotorEx.Direction.FORWARD);
        bronto.backArm.setDirection(DcMotorEx.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        while (getRuntime() - currentTime < .05) {

        }
        if (gamepad1.dpad_down) {
            arm_motor= hardwareMap.get(DcMotorEx.class, "backElbow");
            name = "back elbow";
            arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        } else if (gamepad1.dpad_up) {
            arm_motor= hardwareMap.get(DcMotorEx.class, "frontElbow");
            name = "front elbow";
            arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        } else if (gamepad1.dpad_right) {
            arm_motor= hardwareMap.get(DcMotorEx.class, "frontArm");
            name = "front arm";
            arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        } else if (gamepad1.dpad_left) {
            arm_motor= hardwareMap.get(DcMotorEx.class, "backArm");
            name = "back arm";
            arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            p = 0; i = 0; d = 0; f = 0;
        }
        controller.setPID (p,i,d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

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