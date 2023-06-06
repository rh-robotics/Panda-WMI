package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TwinRobotComponents {

    private final DcMotorEx motorL;
    private final DcMotorEx motorR;
    private final double ticks_per_rotation;
    private final double ticks_per_degree;
    private final double F;
    private final PIDController controller;
    private double target;

    TwinRobotComponents(DcMotorEx motorL, DcMotorEx motorR, double ticks_per_rotation, double p, double i, double d, double f) {
        this.motorL = motorL;
        this.motorR = motorR;
        this.ticks_per_rotation = ticks_per_rotation;
        this.F = f;
        ticks_per_degree = ticks_per_rotation/360.0;
        target = 0;

        controller = new PIDController (p,i,d);
    }

    public double getTicksPerDegree () {return ticks_per_degree;}

    public double getTarget() {return target;}
    public void setTarget(double newTarget) {target = newTarget;}
    public void incrementTarget(double increment) {target+=increment;}

    public void moveUsingPID() {

        controller.reset();
        motorL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int pos = (motorL.getCurrentPosition() + motorR.getCurrentPosition())/2;
        double pid = controller.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * F;
        double power = pid + ff;

        motorL.setPower(power);
        motorR.setPower(power);

    }

    public boolean motorCloseEnough(int range) {
        if ((target - range <= (motorL.getCurrentPosition() + motorR.getCurrentPosition())/2) && (target + range >= (motorL.getCurrentPosition() + motorR.getCurrentPosition())/2)) return true;
        return false;
    }
}
