package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotComponents {

    private final DcMotorEx motor;
    private final double ticks_per_rotation;
    private final double ticks_per_degree;
    private final double F;
    private final PIDController controller;
    private final double armLength = 38.4; //cm, random atm TODO: check arm lengths
    private final double elbowLength = 19.2; //cm
    private double target;

    RobotComponents (DcMotorEx motor, double ticks_per_rotation, double p, double i, double d, double f) {
        this.motor = motor;
        this.ticks_per_rotation = ticks_per_rotation;
        this.F = f;
        ticks_per_degree = ticks_per_rotation/360.0;
        target = 0;

        controller = new PIDController (p,i,d);
    }

    public double getTicksPerDegree () {return ticks_per_degree;}
    public double getArmLength () {return armLength;}
    public double getElbowLength() {return elbowLength;}

    public double getTarget() {return target;}
    public void setTarget(double newTarget) {target = newTarget;}
    public void incrementTarget(double increment) {target+=increment;
    moveUsingPID();}

    public void moveUsingPID() {

        controller.reset();
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * F;
        double power = pid + ff;

        motor.setPower(power);

    }

    public boolean motorCloseEnough(int range) {
        if ((target - range <= motor.getCurrentPosition()) && (target + range >= motor.getCurrentPosition())) return true;
        return false;
    }

    public double armTicksUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double d1 = Math.toDegrees(Math.atan2(y, x));
        double d2 = Math.toDegrees(lawOfCosines(dist, armLength, elbowLength));

        double a1 = d1+d2;
        /*
        not confident about this but should give the encoder position correctly based on angle
        divides by 4 because that gives the 0 degrees (90 degrees relative to the arm), multiplied by said value and added to it
         */
        return (ticks_per_degree * a1 + (ticks_per_rotation / 4.0));
    }

    public double armTicksUsingAngle (double a) {
        return (ticks_per_degree * a + (ticks_per_rotation / 4.0));
    }

    public double getArmAngle () {
        return (motor.getCurrentPosition()-(ticks_per_rotation / 4.0))/ticks_per_degree;
    }

    public double armAngleUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double d1 = Math.toDegrees(Math.atan2(y, x));
        double d2 = Math.toDegrees(lawOfCosines(dist, armLength, elbowLength));

        double a1 = d1+d2;

        return a1;
    }

    public double elbowTicksUsingCoords (double x, double y) {
        double dist = Math.sqrt(x*x + y*y);
        double a2 = Math.toDegrees(lawOfCosines(armLength, elbowLength, dist));
        return (ticks_per_degree * a2); //this definitely needs more math to account for arm movement, but I'm unsure on how
    }

    private double lawOfCosines (double a, double b, double c) { //return angle given side lengths using law of cosines
        return Math.acos((a*a + b*b - c*c) / (2 * a * b));
    }
}
