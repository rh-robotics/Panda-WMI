package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

@Autonomous
public class stateAutonFinal extends LinearOpMode {
    // Class members
    HWC bronto;
    Observer terry;
    PandaBrain brain;

    // Vars
    char startingPosition = 'l';

    enum StateEnum {MOVE_TO_POLE_1, STRAFE_POLE_1, TURN_POLE_1, SCORE_POLE_1, TURN_TO_PARK, STRAFE_TO_PARK, PARK}

    StateEnum state = StateEnum.MOVE_TO_POLE_1;
    double backElbowTarget, frontElbowTarget, backArmTarget, frontArmTarget;
    boolean elbowsReset = false;
    boolean parkOnly = true;
    boolean armsDoinSomethingElse = false;
    int loopCount = 0;

    String runningTrajectory = "none";

    @Override
    public void runOpMode() throws InterruptedException {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new PandaBrain(bronto);
        terry = new Observer(bronto);

        // Set Arm Targets
        backElbowTarget = bronto.backElbowAutonDrivePos;
        frontElbowTarget = bronto.frontElbowAutonDrivePos;
        backArmTarget = bronto.backArmDrivePos;
        frontArmTarget = bronto.frontArmDrivePos;

        // RIGHT Trajectories
        Trajectory RIGHT_toPole1 = bronto.drive.trajectoryBuilder(new Pose2d())
                .forward(46)
                .build();

        Trajectory RIGHT_strafeRightToPole1 = bronto.drive.trajectoryBuilder(RIGHT_toPole1.end())
                .strafeRight(8)
                .build();

        Trajectory RIGHT_strafeToPark = bronto.drive.trajectoryBuilder(RIGHT_strafeRightToPole1.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .strafeRight(8)
                .build();

        Trajectory RIGHT_forwardToPark = bronto.drive.trajectoryBuilder(RIGHT_strafeToPark.end())
                .forward(35)
                .build();

        Trajectory RIGHT_parkZone1 = bronto.drive.trajectoryBuilder(RIGHT_forwardToPark.end()) // TODO: Change Trajectory
                .strafeRight(24)
                .build();

        Trajectory RIGHT_parkZone3 = bronto.drive.trajectoryBuilder(RIGHT_forwardToPark.end()) // TODO: Change Trajectory
                .strafeLeft(24)
                .build();

        // LEFT Trajectories
        Trajectory LEFT_toPole1 = bronto.drive.trajectoryBuilder(new Pose2d())
                .forward(46)
                .build();

        Trajectory LEFT_strafeRightToPole1 = bronto.drive.trajectoryBuilder(LEFT_toPole1.end())
                .strafeLeft(8)
                .build();

        Trajectory LEFT_strafeToPark = bronto.drive.trajectoryBuilder(LEFT_strafeRightToPole1.end().plus(new Pose2d(0, 0, Math.toRadians(-180))))
                .strafeLeft(8)
                .build();

        Trajectory LEFT_forwardToPark = bronto.drive.trajectoryBuilder(LEFT_strafeToPark.end())
                .forward(35)
                .build();

        Trajectory LEFT_parkZone1 = bronto.drive.trajectoryBuilder(LEFT_forwardToPark.end()) // TODO: Change Trajectory
                .strafeRight(24)
                .build();

        Trajectory LEFT_parkZone3 = bronto.drive.trajectoryBuilder(LEFT_forwardToPark.end()) // TODO: Change Trajectory
                .strafeLeft(24)
                .build();

        // PARKONLY Trajectories
        Trajectory PARKONLY_forward = bronto.drive.trajectoryBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                .forward(12)
                .build();

        Trajectory PARKONLY_parkZone1 = bronto.drive.trajectoryBuilder(PARKONLY_forward.end())
                .strafeLeft(12)
                .build();

        Trajectory PARKONLY_parkZone3 = bronto.drive.trajectoryBuilder(LEFT_toPole1.end())
                .strafeRight(12)
                .build();

        // Init Loop
        bronto.cv();
        while (!opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Zone", bronto.sleeveDetection.getPosition());
            telemetry.addData("Starting Location", startingPosition);
            telemetry.addData("Park Only", parkOnly);
            telemetry.addData("Arms Doing Something Else", armsDoinSomethingElse);
            telemetry.addData("-----------------------", "");
            telemetry.addData("Set starting location Right", "a");
            telemetry.addData("Set starting location Left", "b");
            telemetry.addData("Set park only", "x");
            telemetry.update();

            if(gamepad1.a) {
                startingPosition = 'r';
            } else if(gamepad1.b) {
                startingPosition = 'l';
            } else if(gamepad1.x) {
                parkOnly = !parkOnly;
            }

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }

        // Running Loop
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("State", state);
            telemetry.addData("Zone", bronto.parkingZone);
            telemetry.addData("Park Only", parkOnly);
            telemetry.addData("Starting Location", startingPosition);
            telemetry.addData("Loop Number", loopCount);
            telemetry.addData("Running Trajectory", runningTrajectory);
            telemetry.addData("Arms Doing Something Else", armsDoinSomethingElse);
            telemetry.addData("isBusy", bronto.drive.isBusy());
            telemetry.update();

//            while(!gamepad1.a) {}

            bronto.drive.update();

            if (!armsDoinSomethingElse) {
                if (!elbowsReset) {
                    terry.setCycleState(Observer.RobotStates.AutonDrive);
                } else {
                    terry.setCycleState(Observer.RobotStates.Drive);
                }
            }

            if (startingPosition == 'r') {
                switch (state) {
                    case MOVE_TO_POLE_1:
                        if(!Objects.equals(runningTrajectory, "RIGHT_toPole1")) bronto.drive.followTrajectoryAsync(RIGHT_toPole1);

                        runningTrajectory = "RIGHT_toPole1";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.STRAFE_POLE_1;
                        }
                        break;
                    case STRAFE_POLE_1:
                        if(!Objects.equals(runningTrajectory, "RIGHT_strafeRightToPole1")) bronto.drive.followTrajectoryAsync(RIGHT_strafeRightToPole1);
                        runningTrajectory = "RIGHT_strafeRightToPole1";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.TURN_POLE_1;
                        }
                        break;
                    case TURN_POLE_1:
                        if(!Objects.equals(runningTrajectory, "Turning...")) bronto.drive.turnAsync(Math.toRadians(90));
                        runningTrajectory = "Turning...";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.SCORE_POLE_1;
                        }
                    case SCORE_POLE_1:
                        // Flop Out Elbows
                        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        bronto.frontElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        bronto.backElbow.setPower(1);
                        bronto.frontElbow.setPower(1);
                        sleep(1000);
                        bronto.backElbow.setPower(0);
                        bronto.frontElbow.setPower(0);
                        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        elbowsReset = true;


                        while(!gamepad1.a) {}
                        armsDoinSomethingElse = true;
                        runningTrajectory = "none";
                        terry.setCycleState(Observer.RobotStates.HighJct);
                        terry.giveDriverApproval();

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.TURN_TO_PARK;
                            armsDoinSomethingElse = false;
                        }
                        break;
                    case TURN_TO_PARK:
                        if(!Objects.equals(runningTrajectory, "Turning...")) bronto.drive.turnAsync(Math.toRadians(90));
                        runningTrajectory = "Turning...";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.STRAFE_TO_PARK;
                        }
                        break;
                    case STRAFE_TO_PARK:
                        if(!Objects.equals(runningTrajectory, "RIGHT_strafeToPark")) bronto.drive.followTrajectoryAsync(RIGHT_strafeToPark);
                        runningTrajectory = "RIGHT_strafeToPark";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.PARK;
                        }
                        break;
                    case PARK:
                        if (parkOnly) {
                            if(!Objects.equals(runningTrajectory, "PARKONLY_forward")) bronto.drive.followTrajectoryAsync(PARKONLY_forward);
                            runningTrajectory = "PARKONLY_forward";
                            bronto.drive.update();
                                if (!bronto.drive.isBusy()) {
                                switch (bronto.parkingZone) {
                                    case 1:
                                        if(!Objects.equals(runningTrajectory, "PARKONLY_parkZone1")) bronto.drive.followTrajectoryAsync(PARKONLY_parkZone1);
                                        runningTrajectory = "PARKONLY_parkZone1";
                                    case 2:
                                        break;
                                    case 3:
                                        if(!Objects.equals(runningTrajectory, "PARKONLY_parkZone3")) bronto.drive.followTrajectoryAsync(PARKONLY_parkZone3);
                                        runningTrajectory = "PARKONLY_parkZone3";
                                }
                            }
                        } else {
                            bronto.drive.update();
                            if (!bronto.drive.isBusy()) {
                                switch (bronto.parkingZone) {
                                    case 1:
                                        if(!Objects.equals(runningTrajectory, "RIGHT_parkZone1")) bronto.drive.followTrajectoryAsync(RIGHT_parkZone1);
                                        runningTrajectory = "RIGHT_parkZone1";
                                    case 2:
                                        break;
                                    case 3:
                                        if(!Objects.equals(runningTrajectory, "RIGHT_parkZone3")) bronto.drive.followTrajectoryAsync(RIGHT_parkZone3);
                                        runningTrajectory = "RIGHT_parkZone3";
                                }
                            }
                        }
                        break;
                }
            } else if (startingPosition == 'l') {
                switch (state) {
                    case MOVE_TO_POLE_1:
                        if(!Objects.equals(runningTrajectory, "LEFT_toPole1")) bronto.drive.followTrajectoryAsync(LEFT_toPole1);
                        runningTrajectory = "LEFT_toPole1";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.STRAFE_POLE_1;
                        }
                        break;
                    case STRAFE_POLE_1:
                        if(!Objects.equals(runningTrajectory, "LEFT_strafeRightToPole1")) bronto.drive.followTrajectoryAsync(LEFT_strafeRightToPole1);
                        runningTrajectory = "LEFT_strafeRightToPole1";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.TURN_POLE_1;
                        }
                        break;
                    case TURN_POLE_1:
                        if(!Objects.equals(runningTrajectory, "Turning...")) bronto.drive.turnAsync(Math.toRadians(-90));
                        runningTrajectory = "Turning...";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.SCORE_POLE_1;
                        }
                        break;
                    case SCORE_POLE_1:
                        // Flop Out Elbows
                        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        bronto.frontElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        bronto.backElbow.setPower(1);
                        bronto.frontElbow.setPower(1);
                        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        elbowsReset = true;
                        armsDoinSomethingElse = true;
                        runningTrajectory = "none";

                        terry.setCycleState(Observer.RobotStates.HighJct);
                        terry.giveDriverApproval();

                        // TODO: Run Servos to Score

                        break;
                    case TURN_TO_PARK:
                        if(!Objects.equals(runningTrajectory, "Turning...")) bronto.drive.turnAsync(Math.toRadians(-90));
                        runningTrajectory = "Turning...";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.STRAFE_TO_PARK;
                        }
                        break;
                    case STRAFE_TO_PARK:
                        if(!Objects.equals(runningTrajectory, "LEFT_strafeToPark")) bronto.drive.followTrajectoryAsync(LEFT_strafeToPark);
                        runningTrajectory = "LEFT_strafeToPark";

                        if(!bronto.drive.isBusy()) {
                            state = StateEnum.PARK;
                        }
                        break;
                    case PARK:
                        if (parkOnly) {
                            if(!Objects.equals(runningTrajectory, "PARKONLY_forward")) bronto.drive.followTrajectoryAsync(PARKONLY_forward);
                            runningTrajectory = "PARKONLY_forward";

                            if (!bronto.drive.isBusy()) {
                                switch (bronto.parkingZone) {
                                    case 1:
                                        if(!Objects.equals(runningTrajectory, "PARKONLY_parkZone1")) bronto.drive.followTrajectoryAsync(PARKONLY_parkZone1);
                                        runningTrajectory = "PARKONLY_parkZone1";
                                    case 2:
                                        break;
                                    case 3:
                                        if(!Objects.equals(runningTrajectory, "PARKONLY_parkZone3")) bronto.drive.followTrajectoryAsync(PARKONLY_parkZone3);
                                        runningTrajectory = "PARKONLY_parkZone3";
                                }
                            }
                        } else {
                            if (!bronto.drive.isBusy()) {
                                switch (bronto.parkingZone) {
                                    case 1:
                                        if(!Objects.equals(runningTrajectory, "LEFT_parkZone1")) bronto.drive.followTrajectoryAsync(LEFT_parkZone1);
                                        runningTrajectory = "LEFT_parkZone1";
                                    case 2:
                                        break;
                                    case 3:
                                        if(!Objects.equals(runningTrajectory, "LEFT_parkZone3")) bronto.drive.followTrajectoryAsync(LEFT_parkZone3);
                                        runningTrajectory = "LEFT_parkZone3";
                                }
                            }
                        }
                        break;
                }
            }
            loopCount++;
        }
    }
}