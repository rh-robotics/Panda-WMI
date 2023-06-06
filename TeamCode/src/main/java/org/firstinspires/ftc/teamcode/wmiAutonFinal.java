package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

@Autonomous
public class wmiAutonFinal extends LinearOpMode {
  // Class members
  HWC panda;
  Observer terry;
  PandaBrain brain;

  // Vars
  char startingPosition = 'l';

  enum StateEnum {MOVE_TO_POLE_1}

  StateEnum state = StateEnum.MOVE_TO_POLE_1;
  boolean elbowsReset = false;
  boolean parkOnly = true;
  int loopCount = 0;
  String runningTrajectory = "none";

  @Override
  public void runOpMode() throws InterruptedException {
    panda = new HWC(hardwareMap, telemetry);
    brain = new PandaBrain(panda);
    terry = new Observer(panda);

  
  }
}
