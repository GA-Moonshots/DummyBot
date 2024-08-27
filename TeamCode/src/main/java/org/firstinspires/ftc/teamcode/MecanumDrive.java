/*
 * MecanumDrive Class - FTC Robot Mecanum Drive Subsystem
 *
 * This class represents the Mecanum drive subsystem on an FTC robot.
 * It includes methods for controlling the Mecanum drive using encoders, gyroscope, and distance sensors.
 *
 * Author: Micheal + [everyone who worked on it]
 * Last Modified: 12/8/2023 10:59am
 * Version: [Self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - LinearOpMode (import from com.qualcomm.robotcore.eventloop.opmode)
 *     - MecanumDrive
 *
 * Enums:
 *   - AprilTagToAlign: LEFT, CENTER, RIGHT, NONE
 *   - HowToMove: LEFT, RIGHT, BACK, ROTATE_LEFT, ROTATE_RIGHT
 *
 * Subsystem Assets:
 *   - DcMotor leftFront
 *   - DcMotor rightFront
 *   - DcMotor leftBack
 *   - DcMotor rightBack
 *   - DistanceSensor rearDistance
 *   - DistanceSensor leftDistance
 *   - DistanceSensor rightDistance
 *   - Camera camera
 *   - IMU imu
 *   - Telemetry telemetry
 *   - double gyroTarget
 *   - double runawayRobotShield
 *   - double lastAprilTagYaw
 *   - double lastAprilTagStrafe
 *   - double fieldCentricTarget
 *   - boolean isFieldCentric
 *   - boolean isGyroLocked
 *   - boolean isTargetSet
 *
 * Methods:
 *   Constructors:
 *     - MecanumDrive(LinearOpMode opMode): Initializes the Mecanum drive subsystem with sensors and motors.
 *
 *   State Commands:
 *     - makeRobotCentric(): Switches the drive to robot-centric mode.
 *     - makeFieldCentric(): Switches the drive to field-centric mode.
 *     - toggleFieldCentric(): Toggles between field-centric and robot-centric modes.
 *     - resetFieldCentricTarget(): Resets the field-centric target based on the gyroscope reading.
 *     - postDistanceReadouts(): Displays distance sensor readings in telemetry.
 *
 *   Core Drive Commands:
 *     - drive(double forward, double strafe, double turn): Drives the robot with mecanum wheels using specified inputs.
 *     - stop(): Stops the drive from moving.
 *
 *   Autonomous Drive Commands:
 *     - autonomouslyDriveByTime(double forward, double strafe, double turn, double time): Drives autonomously for a specified time.
 *     - autonomouslyMove(double strength, double target, HowToMove side, double maxTime): Moves autonomously based on sensor feedback.
 *     - maintainStrafe(DistanceSensor sensor, double strength, double target): Feedback loop for maintaining lateral position.
 *     - maintainForward(DistanceSensor sensor, double strength, double target): Feedback loop for maintaining forward position.
 *     - maintainTurn(double strength, double target): Feedback loop for maintaining robot's orientation.
 *
 *   Friendly Autonomous Pass-Through Commands:
 *     - gotoBackDistance(double str, double target, double maxTime): Moves backward autonomously to a specified distance.
 *     - gotoRightDistance(double str, double target, double maxTime): Moves right autonomously to a specified distance.
 *     - gotoLeftDistance(double str, double target, double maxTime): Moves left autonomously to a specified distance.
 *
 *   This Year's Game Commands:
 *     - faceTheProp(double str): Turns the robot to face a prop based on sensor readings.
 *     - circleScanForProp(double str, HowToMove movement, double maxTime): Scans in a circular motion to locate a prop.
 *
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    // USEFUL ENUMS


    public enum HowToMove {
        LEFT,
        RIGHT,
        BACK,
        ROTATE,
    }

    // MOTORS
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;

    // SENSORs
    public Telemetry telemetry;

    // INSTANCE VARIABLES
    private final LinearOpMode opMode;
    private double gyroTarget = 0.0d;
    private double runawayRobotShield = -1;
    private double lastAprilTagYaw = 0.0d;
    private double lastAprilTagStrafe = 0.0d;
    public double fieldCentricTarget = 0.0d;
    private boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;


    public MecanumDrive(@NonNull LinearOpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        leftFront = opMode.hardwareMap.get(DcMotorEx.class, Constants.LEFT_FRONT_NAME);
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_FRONT_NAME);
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, Constants.LEFT_BACK_NAME);
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_BACK_NAME);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // ---- CORE DRIVE COMMANDS ----
    // ---- CORE DRIVE COMMANDS ----
    // ---- CORE DRIVE COMMANDS ----

    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    /**
     * Translates desired motion into mecanum commands
     * @param forward negative is forward
     * @param strafe lateral movement
     * @param turn positive is clockwise
     */
    public void drive(double forward, double strafe, double turn) {
        //if(isTargetSet && isGyroLocked)
        //boost = -(gyroTarget - imu.getYAngle()) / Constants.GYRO_BOOST_FACTOR;

        // forward is reversed (flight stick) and boost values should match the turn
        // the mecanum drive is a X instead of a diamond
        double leftFrontPower = -forward + strafe + turn;
        double rightFrontPower = forward + strafe + turn;
        double leftBackPower = -forward - strafe + turn;
        double rightBackPower = forward - strafe + turn;

        // if an input exceeds max, everything is proportionally reduced to keep balanced
        double powerScale = Math.max(1,
                Math.max(
                        Math.max(
                                Math.abs(leftFrontPower),
                                Math.abs(leftBackPower)
                        ),
                        Math.max(
                                Math.abs(rightFrontPower),
                                Math.abs(rightBackPower)
                        )
                )
        );
        leftFrontPower /= powerScale;
        leftBackPower /= powerScale;
        rightBackPower /= powerScale;
        rightFrontPower /= powerScale;

        // ?? if we clip the ranges below, why are we multiplying them here?
        leftFrontPower *= Constants.MOTOR_MAX_SPEED;
        leftBackPower *= Constants.MOTOR_MAX_SPEED;
        rightBackPower *= Constants.MOTOR_MAX_SPEED;
        rightFrontPower *= Constants.MOTOR_MAX_SPEED;

        if(telemetry != null)
            telemetry.addData("Motors", "(%.2f, %.2f, %.2f, %.2f)",
                    leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        drive(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );
    }

    /**
     * Clips and executes given motor speeds
     */
    protected void drive(double m1, double m2, double m3, double m4) {
        leftFront.setPower(Range.clip(m1, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
    }
}