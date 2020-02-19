package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous
@Disabled
public class skystoneAutoRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private Servo autoGrab;
    private DcMotor armMotor;
    private Servo clawArm;
    private Servo claw;
    private BNO055IMU imu;
    BNO055IMU.Parameters IMU_Parameters;
    float Yaw_Angle;

    static final double     COUNTS_PER_MOTOR_REV    = 1140 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 45/35 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.6;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override

    public void runOpMode() throws InterruptedException {
        BackRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight = hardwareMap.dcMotor.get("BackRight");
        // reverse right motors
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawArm = hardwareMap.servo.get("clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // init servo position
        armOut();
        sleep(250);
        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();


        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        imu.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // detect the skystones
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        int leftvaluedetect = -1;
        int midvaluedetect = -1;
        int rightvaluedetect = -1;
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            leftvaluedetect = valLeft;
            midvaluedetect = valMid;
            rightvaluedetect = valRight;



            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }

        // drive forward slightly
        encoderDriveIMU(DRIVE_SPEED, 11, 11, 5);
        // deploy claw
        // lift arm
        liftPosition(1,14,5);
        // claw out
        armIn();
        sleep(250);
        // open claw
        openClaw();
        sleep(300);

        // skystone position left
        if (leftvaluedetect == 0) {
            // turn to left stone
            encoderStrafe(DRIVE_SPEED,12,-12,5);
            //encoderDrive(DRIVE_SPEED, -7, 7, 5);

        }
        // skystone position middle
        if (midvaluedetect == 0) {
            // drive forward to middle stone
            //encoderDrive(DRIVE_SPEED, -4, 4, 5);
            encoderStrafe(DRIVE_SPEED,6,-6,5);
        }
        // skystone position right
        if (rightvaluedetect == 0) {
            encoderStrafe(DRIVE_SPEED,-2,2,5);

        }
        // put arm down
        liftPosition(1,-16,5);

        // drive forward to stone
        encoderDriveIMU(DRIVE_SPEED,9.25,9.25,5);
        // close claw down
        closeClaw();
        // make sure claw is out
        armIn();
        sleep(1000);
        // drive backward
        encoderDriveIMU(DRIVE_SPEED,-6,-6,5);

        // turn towards skybridge
        // left pos turn
        // for previously used turning
        /*
        if (leftvaluedetect == 0) {
            encoderDrive(DRIVE_SPEED, 23.5, -23.5, 5);
        }
        if (midvaluedetect == 0) {
            encoderDrive(DRIVE_SPEED, 20.5, -20.5, 5);
        }
        if (rightvaluedetect == 0) {
            encoderDrive(DRIVE_SPEED, 18, -18, 5);

        }

         */
        // turn towards skybridge
        encoderDrive(TURN_SPEED-0.1, 16.5, -16.5, 5);


        // sprint straight to infront of platform
        if (midvaluedetect == 0) {
            encoderDriveIMU(1,57,57,5);
        }

        if (leftvaluedetect == 0) {
            encoderDriveIMU(1,65,65,5);
        }

        if (rightvaluedetect == 0) {
            encoderDriveIMU(1,52,52,5);

        }
        // make sure the stone is in the claw by opening it and closing it
        /*
        openClaw();
        sleep(200);
        closeClaw();
        sleep(200);

         */

        // raise brick
        liftPosition(1,10,5);
        // turn towards platform
        encoderDrive(TURN_SPEED,-17,17,5);
        // drive forward to platform
        encoderDrive(DRIVE_SPEED,9.5,9.5,5);
        // drop stone on platform
        // strafe to right before dropping stone
        //encoderStrafe(0.75,5.1,-5.1,5);
        // drop stone
        openClaw();
        sleep(600);


        // strafe back to correct position
        //encoderStrafe(0.75,-5.5,5.5,5);

        // move platform
        //if (midvaluedetect == 0) {
        if (true) {
            // raise lift so it does not collide with plat
            liftPosition(1,4,5);
            // put claw back in robot
            armOut();
            sleep(650);
            // put arm down
            liftPosition(0.75,-14,5);
            // drive back slightly
            encoderDrive(DRIVE_SPEED,-5,-5,5);
            // turn side claw toward build platform
            encoderDrive(TURN_SPEED,16.75,-16.75,5);
            // drive forward to not hit stone ** untesteddd
            encoderDrive(DRIVE_SPEED,8.5,8.5,5);
            // strafe into build platform
            encoderStrafe(DRIVE_SPEED,8,-8,5);
            // side claw down
            autoClawDown();
            sleep(800);
            // strafe into build zone
            encoderStrafe(1,-30,30,5); // DRIVE_SPEED-0.35
            // turn platform
            encoderDrive(TURN_SPEED,23,-23,5);

            /*
            // drive forward to straighten platform
            encoderDriveIMU(DRIVE_SPEED, 12,12,5);
            // strafe back into the wall to preserve score
            encoderStrafe(DRIVE_SPEED,-6,6,5);

             */

            autoClawUp();
            sleep(300);
            /*
            // drive to parking line
            encoderDrive(1,-25,-25,6);
            // strafe to second position
            encoderStrafe(1,15,-15,5);
            // drive into platform
            encoderDrive(1,15,15,5);
            // drive away from platform
            //encoderDrive(1,-5,-5,5);

            // drive to line
            encoderDrive(1,-20,-20,5);

            // strafe to bridge side
            encoderStrafe(1,12,-12,5);

             */

            // strafe into platform to ensure platform is in building zone
            encoderStrafe(1,12,-12,5);
            // drive forward a tinybit
            //encoderDriveIMU(DRIVE_SPEED,6.5,6.5,5);
            //
            encoderDriveIMU(0.5,-4,-4,5);
            // strafe to second position

            encoderStrafe(1,-36,36,5);
            // reverse into skybridg

            // strafe to move build platform into build zone

            /* code that was previously intended for going to get a second stone


            // drive back from platform
            encoderDrive(DRIVE_SPEED,-5.5,-5.5,5);
            // turn facing skybridge
            encoderDrive(TURN_SPEED,-16.75,16.75,5);

            // drive to middle stone positon
            encoderDrive(DRIVE_SPEED, 75,75,5);

             */

        }
        // get another stone if right position
        if (rightvaluedetect == 0) {

        }
        // move platform if left position
        if (leftvaluedetect == 0) {

        }
        //

        //
        /*
         *  Method to perfmorm a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    // use encoders to strafe
    public void encoderStrafe(double speed,
                              double a, double b,
                              double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(b * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(a * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(a * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(b * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveIMU(double speed,
                                double leftInches, double rightInches,
                                double timeoutS) {
        // current angle of the robot on start of method call
        float startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;



        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {
                // Save gyro's yaw angle
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw angle", Yaw_Angle);

                // amount the motor speed is reduced by to counteract change in angle
                double adjustAmount = 1;
                // if the robot is going in a straight line use the correction alogrithm , if turning DO NOT
                if (leftInches == rightInches) {
                    if (Yaw_Angle < startHeading - 5) {
                        // Turn left by letting right slightly overpower
                        BackRight.setPower(Math.abs(speed));
                        BackLeft.setPower(Math.abs(speed) - adjustAmount);
                        FrontLeft.setPower(Math.abs(speed) - adjustAmount);
                        FrontRight.setPower(Math.abs(speed));
                    } else if (Yaw_Angle > startHeading + 5) {
                        // Turn right by letting left slightly overpower
                        BackRight.setPower(Math.abs(speed) - adjustAmount);
                        BackLeft.setPower(Math.abs(speed));
                        FrontLeft.setPower(Math.abs(speed));
                        FrontRight.setPower(Math.abs(speed) - adjustAmount);
                    } else {
                        // Continue straight
                        BackRight.setPower(Math.abs(speed));
                        BackLeft.setPower(Math.abs(speed));
                        FrontLeft.setPower(Math.abs(speed));
                        FrontRight.setPower(Math.abs(speed));
                    }
                }
                // if the robot is turning, do not use correction algorithm
                else {
                    // simply maintain power
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }



    public void liftPosition(double speed, double inches, double timeoutS) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = armMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            armMotor.setTargetPosition(target);
            // Turn On RUN_TO_POSITION

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            armMotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d ", target);
                telemetry.addData("Path2",  "Running at %7d ",
                        armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            armMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void armOut() {
        clawArm.setPosition(0.72);

    }

    public void armIn() {
        clawArm.setPosition(0.04);

    }

    public void autoClawDown() {
        autoGrab.setPosition(1);

    }
    public void autoClawUp() {
        autoGrab.setPosition(0.2);
    }

    private void closeClaw() {
        claw.setPosition(1);
    }


    private void openClaw() {
        claw.setPosition(0.5);
    }
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}