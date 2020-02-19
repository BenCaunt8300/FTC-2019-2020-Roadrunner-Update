package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import static java.lang.Math.toRadians;


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
public class RoadRunnerBlue extends LinearOpMode {
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

    private Servo sideClaw;
    private Servo sideClawDeploy;

    BNO055IMU.Parameters IMU_Parameters;
    public DistanceSensor sensorRange;
    float Yaw_Angle;

    // angle that the robot turns when going to obtain the 1st skystone
    private final int turnAngleTo1stPos = 24;

    static final double     COUNTS_PER_MOTOR_REV    = 1140 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 45/35 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.75;
    static final boolean ARM_DEPLOY = true;
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
        sensorRange = hardwareMap.get(DistanceSensor.class, "Distance");
        sideClawDeploy = hardwareMap.servo.get("sideClawDeploy");
        sideClaw = hardwareMap.servo.get("sideClaw");

        clawArm.setPosition(0.72);
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
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);


        // detect the skystones
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        // init hook pos
        autoClawUp();
        // init side claw pos
        sideClawIn();
        sideClawClose();
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



        if (isStopRequested()) return;


        // x pos of stone line
        int stoneLineXpos = 30;
        // drive to stones


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(stoneLineXpos, 0))
                        .build());

        // turn claw towards
        drive.turnSync(toRadians(-90));

        // open the claw
        sideClawOpen();

        sideClawOut();

        // estimate robots current position
        Pose2d poseEstimate = drive.getPoseEstimate();
        if (leftvaluedetect == 0) {

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()+15 ))
                            .build());

        }
        // skystone position middle
        else if (midvaluedetect == 0) {
            // spline to middle stone
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .reverse()
                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()+6 ))
                            .build());

        }
        // skystone position right
        else if (rightvaluedetect == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()-7))
                            .build());

        }

        // strafe LEFT into stone / skystone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(10)
                        .build());


        // close the claw
        sideClawClose();
        sleep(400);
        // raise the claw
        sideClawIn();
        sleep(250);
        // strafe RIGHT away from stone / skystone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(10)
                        .build());
        // drive to platform with stone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(poseEstimate.getX(), +115))
                        .build());
        // strafe to platform
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(16)
                        .build());
        // drop stone on platform
        sideClawOut();
        sleep(500);
        sideClawOpen();
        sleep(400);

        // put the claw in the robot first
        sideClawClose();
        sideClawIn();
        sleep(450);

        // estimate robots current position
        // strafe away
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(16)
                        .build());
        sideClawClose();



        if (leftvaluedetect == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()-23))
                            .build());
        }
        // skystone position middle
        else if (midvaluedetect == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()-30))
                            .build());
        }
        // skystone position right
        else if (rightvaluedetect == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()-30))
                            .build());
        }



        // open the claw
        sideClawOpen();

        sideClawOut();
        sleep(450);



        // strafe into skystone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(15)
                        .build());




        sleep(400);
        // raise the claw
        sideClawIn();
        sleep(250);

        // strafe away from stones
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(15)
                        .build());

        // drive to platform
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(poseEstimate.getX(), poseEstimate.getY()+130))
                        .build());

        // strafe to platform
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(16)
                        .build());

        // lower the hooks
        autoClawDown();
        sleep(300);
        // drop stone on platform
        sideClawOut();
        sleep(500);
        sideClawOpen();
        sleep(300);



        /*
         *
         *
         *
         *
         *
         *  FROM SKYSTONE AUTO RED
         *
         *
         *
         *
         */
        // put claw back in robot
        sideClawClose();
        sleep(300);
        sideClawIn();



        // strafe into build zone
        // strafe away from stones
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(45)
                        .build());


        // turn platform
        drive.turnSync(toRadians(100));


        autoClawUp();
        sleep(300);


        // strafe into platform to ensure platform is in building zone
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(30)
                        .build());
        //encoderStrafe(1,12,-12,5);
        // drive forward a tinybit



        // strafe to second position
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(75)
                        .build());




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

        // drive into platform
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(60, -130))
                        .build()
        );
        // drop stone
        openClaw();
        sleep(300);


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

                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {


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

                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {
                // Save gyro's yaw angle
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw angle", Yaw_Angle);

                // amount the motor speed is reduced by to counteract change in angle
                double adjustAmount = 0.5;
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

            }

            // Stop all motion;

            armMotor.setPower(0);


        }
    }
    public void armOut() {
        if (ARM_DEPLOY) {
            clawArm.setPosition(0.69);
        }
    }

    public void armIn() {
        if (ARM_DEPLOY) {
            clawArm.setPosition(0.04);
        }

    }

    public void autoClawDown() {
        autoGrab.setPosition(1);

    }
    public void autoClawUp() {
        autoGrab.setPosition(0.3);
    }

    private void closeClaw() {
        if (ARM_DEPLOY) {
            claw.setPosition(1);
        }
    }


    private void openClaw() {
        if (ARM_DEPLOY) {
            claw.setPosition(0.5);
        }
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
        // uses the distance sensor to detect if there is anything infornt of the robot
        public boolean isObjectDetected(double distance) {
            if (distance > 1000) {
                return true;
            }
            else {
                return false;
            }

        }
    }
}