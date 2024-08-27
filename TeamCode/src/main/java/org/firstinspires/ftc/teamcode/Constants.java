package org.firstinspires.ftc.teamcode;

public class Constants {
    // ----ARM VARIABLES----
    // ----ARM VARIABLES----
    public static final int ARM_UP_POSITION = -700;
    public static final int ARM_DOWN_POSITION = 0;
    public static final double ARM_MOTOR_STRENGTH_UP = 0.9;
    public static final double ARM_MOTOR_STRENGTH_DOWN = 0.7;
    public static final double WRIST_ON_WALL =  0.7;
    public static final double LEFT_CLAW_OPEN_POS = 0.5;
    public static final double LEFT_CLAW_CLOSED_POS = 0.7;
    public static final double RIGHT_CLAW_OPEN_POS = 0.0;
    public static final double RIGHT_CLAW_CLOSED_POS = 0.5;
    public static final double WRIST_INC = 0.01;
    public static final double WRIST_MAX = 1;
    public static final double WRIST_ON_GROUND = 0.44;
    public static final double WRIST_MIN = 0.15;
    public static final double ROLL_FLIPPED = 0.9;
    public static final double ROLL_UP = 0.15;
    public static final double ROLL_INC = 0.03;

    // -------DRIVE--------
    // -------DRIVE--------
    public static final double MOTOR_MAX_SPEED = 0.9;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 3;
    public static final double APRIL_TAG_DISTANCE_TARGET = 5;
    public static final double APRIL_TAG_PRECISION = 10;
    public static final double APRIL_TAG_MAX_SPEED = 0.3;
    public static final double GYRO_BOOST_FACTOR = 5;

    // Generalized minimum input for the robot to respond to controller input
    public static final double INPUT_THRESHOLD = 0.1;
    // Generalized minimum angle difference for the robot to respond to an autonomous movement command
    public static final double ANGLE_THRESHOLD = 3.0;
    // Generalized minimum position difference for the robot to respond to an autonomous movement command
    public static final double DISTANCE_THRESHOLD = 1;

    public static final double KP = 0.2;

    // ELEVATOR
    public static final double LOCK_OFF_POSITION = 0.0;
    public static final double LOCK_ON_POSITION = 1.0;
    public static final double ELEVATOR_MAX_SPEED = 0.9;

    // ----- HARDWARE MAP NAMES ------
    // -------- SENSOR NAMES ---------
    public static final String IMU_NAME = "imu";
    public static final String REAR_DIST_NAME = "rear_distance";
    public static final String RIGHT_DIST_NAME = "right_distance";
    public static final String LEFT_DIST_NAME = "left_distance";
    public static final String WEBCAM_NAME = "Webcam 1";
    // --------- MOTOR NAMES ---------
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";
    public static final String ARM_MOTOR_NAME = "arm";
    public static final String ELEVATOR_MOTOR_NAME = "elevator";
    // --------- SERVO NAMES ---------
    public static final String WRIST_SERVO_NAME = "wrist";
    public static final String LEFT_OPEN_SERVO_NAME = "left_open";
    public static final String RIGHT_OPEN_SERVO_NAME = "right_open";
    public static final String ROLL_SERVO_NAME = "roll";
    public static final String LAUNCHER_SERVO_NAME = "launcher";
    public static final String ELEVATOR_LEFT_SERVO_NAME = "elevLeft";
    public static final String ELEVATOR_RIGHT_SERVO_NAME = "elevRight";
}
