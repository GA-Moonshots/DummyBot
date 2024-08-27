package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Dumb")
public class Dumb extends LinearOpMode {
    // SUBSYSTEMS
    private MecanumDrive drive;

    @Override
    public void runOpMode() {

        // Init Loop (runs until stop button or start button is pressed)
        while(opModeInInit()) {

            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
        }

        waitForStart();

        // ---MAIN EXECUTION LOOP (runs until stop is pressed) --
        while(opModeIsActive()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            // telemetry.addData("UPS", 1 / (timer.seconds() - lastTime));

            // Driver 1: Responsible for drivetrain and movement
            if(opModeIsActive()) driver1Inputs();
            // Driver 2: Responsible for the subsystem attachment
            if(opModeIsActive()) driver2Inputs();

            telemetry.update();
        }
    }

    /**
     * Driver 1: Solely responsible for the control of the drivetrain;
     * this function never changes and should not be changed unless addition of a new
     * feature is required.
     */
    private void driver1Inputs() {

        double speedMod = gamepad1.left_stick_button ? 0.2 : 1; // slow mode
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // DEAD-ZONES
        if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
        if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
        if (Math.abs(turn) <= Constants.INPUT_THRESHOLD) turn = 0.0d;

        drive.drive(forward * speedMod, strafe * speedMod, turn * speedMod);


    }

    /**
     * Driver 2: responsible for any subsystem attachments we may have.
     * This function's implementation changes quickly and rapidly every year.
     */
    private void driver2Inputs() {

    }
}
