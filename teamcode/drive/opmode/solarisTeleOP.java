package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "solarisTeleOP", group = "teleOP")
public class solarisTeleOP extends LinearOpMode {
  private DcMotor back_right;
  private DcMotor back_left;
  private DcMotor front_left;
  private DcMotor front_right;
  private DcMotor armMotor;
  private Servo clawArm;
  private Servo autoGrab;
  private Servo claw;
  private Servo sideClaw;
  private Servo sideClawDeploy;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    back_right = hardwareMap.dcMotor.get("FrontRight");
    back_left = hardwareMap.dcMotor.get("FrontLeft");
    front_left = hardwareMap.dcMotor.get("BackLeft");
    front_right = hardwareMap.dcMotor.get("BackRight");
    armMotor = hardwareMap.dcMotor.get("armMotor");
    clawArm = hardwareMap.servo.get("clawArm");
    clawArm.setDirection(Servo.Direction.REVERSE);
    autoGrab = hardwareMap.servo.get("autoGrab");
    autoGrab.setDirection(Servo.Direction.REVERSE);
    claw = hardwareMap.servo.get("claw");
    sideClawDeploy = hardwareMap.servo.get("sideClawDeploy");
    sideClaw = hardwareMap.servo.get("sideClaw");


    // Reverse one of the drive motors.
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    //back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    //front_left.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {

        // speed that dpad controls go at; joy stick speed devided by this 
        double dpadSpeed = 0.65;
          if (gamepad1.dpad_up == true || gamepad2.dpad_up == true) {
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //if (gamepad1.dpad_up == true) {
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            back_left.setPower(dpadSpeed);
            back_right.setPower(dpadSpeed);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            front_left.setPower(dpadSpeed);
            front_right.setPower(-dpadSpeed);
          }
          if (gamepad1.dpad_down == true || gamepad2.dpad_down == true) {
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //if (gamepad1.dpad_down == true) {
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            back_left.setPower(-dpadSpeed);
            back_right.setPower(-dpadSpeed);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            front_left.setPower(-dpadSpeed);
            front_right.setPower(dpadSpeed);
          }
          if (gamepad1.dpad_right == true || gamepad2.dpad_right == true) {
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //if (gamepad1.dpad_right == true) {
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            back_left.setPower(dpadSpeed);
            back_right.setPower(-dpadSpeed);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            front_left.setPower(-dpadSpeed);
            front_right.setPower(-dpadSpeed);
          }
          if (gamepad1.dpad_left == true || gamepad2.dpad_left == true) {
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // if (gamepad1.dpad_left == true) {
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            back_left.setPower(-dpadSpeed);
            back_right.setPower(dpadSpeed);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            front_left.setPower(dpadSpeed);
            front_right.setPower(dpadSpeed);
          }
          else {
          if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
            // Right Joystick
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power. Aiden now has turning rights
            double right_x_scaled = gamepad1.right_stick_x + (gamepad2.right_stick_x / 2);
            //double right_x_scaled = gamepad1.right_stick_x;
            back_left.setPower(right_x_scaled);
            front_left.setPower(right_x_scaled);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            back_right.setPower(-right_x_scaled);
            front_right.setPower(right_x_scaled);
          } else {
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Left Joystick
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            double left_y_scaled = gamepad1.left_stick_y;
            back_left.setPower(-left_y_scaled);
            back_right.setPower(-left_y_scaled);
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            front_left.setPower(-left_y_scaled);
            front_right.setPower(left_y_scaled);
          }
        }

          // put platformClaw down
        // put platform claw up
        if (gamepad1.y) {
          autoClawUp();
        }
        if (gamepad1.a) {
          autoClawDown();
        }

        // control arm with left stick on gamepad 2 
        armMotor.setPower(-gamepad2.left_stick_y / 1);
        
        // flip claw out 
        if (gamepad2.x) {
          armOut();
        }
        // flip claw in 
        if (gamepad2.b) {
          armIn();
        }
        // grab stone 
        if (gamepad2.a) {
          closeClaw();
        }
        
        // release stone 
        if (gamepad2.y) {
          openClaw();
        }
        
        // platform grab down 
        if (gamepad2.left_stick_button) {
          autoClawDown();
        }
        if (gamepad2.right_stick_button){
          autoClawUp();
        }

        // side claw stuff
        if (gamepad1.right_bumper) {
          sideClawIn();
        }
        if (gamepad1.left_bumper) {
          sideClawOut();
        }

        if (gamepad1.left_stick_button) {
          sideClawClose();
        }

        if (gamepad1.right_stick_button) {
          sideClawOpen();
        }
        
        telemetry.addData("Left Pow", back_left.getPower());
        telemetry.addData("Left Pow", back_right.getPower());
        telemetry.addData("Left Pow", front_left.getPower());
        telemetry.addData("Right Pow", front_right.getPower());
        telemetry.update();
  }
}
}
public void armOut() {
  clawArm.setPosition(0.72);
}

public void sideClawIn() {
  sideClawDeploy.setPosition(0.3);

}
public void sideClawOut() {
  sideClawDeploy.setPosition(1);

}

public void sideClawOpen() {
  sideClaw.setPosition(0.5);
}

public void sideClawClose() {
  sideClaw.setPosition(1);
}

  public void armIn() {
    clawArm.setPosition(0.04);
  }
  
  public void autoClawDown() {
    autoGrab.setPosition(1);
    
  }
  public void autoClawUp() {
    autoGrab.setPosition(0.35);
  }

  private void closeClaw() {
    claw.setPosition(1);
  }




  private void openClaw() {
    claw.setPosition(0.65);
  }
}
