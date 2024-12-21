package org.firstinspires.ftc.teamcode.tuning;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive 24-25", group="Iterative Opmode")
public class Drivetrain extends OpMode
{
    //DONT DELETE
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private double Kg = 0.07;
    private boolean boxUp = false;
    //private PIDController pid = new PIDController(0.06, 0, 0);


    //main thingies
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;
    private DcMotor liftMotor1 = null;
    private DcMotor liftMotor2 = null;
    private Servo liftClaw = null;
    private Servo liftClawRotate_Claw = null;
    private Servo liftClawRotate_Arm = null;
    private Servo liftClawExtender = null;

    // lift encoder positions
    private int liftMotor2StartPosition = 0;
    private int liftMotor2EndPosition = 0;

    //last year's thingies
    private DcMotor intakeMotor = null;
    private Servo rightLiftServo = null;
    private Servo leftLiftServo = null;
    private Servo door = null;
    private Servo plane = null;
    private Servo pixelHolder = null;
    private int lift2Position = 0;
    private int lift1Position = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //wheels
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        rearLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        rearRight = hardwareMap.get(DcMotor.class, "rightBack");

        //lift
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        //things attached to lift
        liftClaw = hardwareMap.servo.get("liftClaw");
        liftClawRotate_Claw = hardwareMap.servo.get("liftClawRotate_Claw");
        liftClawRotate_Arm = hardwareMap.servo.get("liftClawRotate_Arm");
        liftClawExtender = hardwareMap.servo.get("liftClawExtender");

        /* 23-24 servos
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightLiftServo = hardwareMap.servo.get("rightLiftServo");
        leftLiftServo = hardwareMap.servo.get("leftLiftServo");
        door = hardwareMap.servo.get("door");
        pixelHolder = hardwareMap.servo.get("pixel");
        plane = hardwareMap.servo.get("plane");
        */


        //SETTINGS FOR MOTORS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_TO_POSITION

        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //door.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        liftMotor2StartPosition = liftMotor2.getCurrentPosition();
        liftMotor2EndPosition = -4800;  // upper limit

        telemetry.addData("lift2 Start Position", liftMotor2StartPosition);
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //GAMEPAD 1 CONTROLS - Primary: Driving, Safety: Lift(Motors)
        //START DRIVING
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        frontLeftPower = (pivot+y+x);
        rearLeftPower = (pivot+y-x);
        frontRightPower = (-pivot+y-x);
        rearRightPower = (-pivot+y+x);

        if(gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower * 0.35);
            frontRight.setPower(frontRightPower * 0.35);
            rearLeft.setPower(rearLeftPower * 0.35);
            rearRight.setPower(rearRightPower * 0.35);
        }
        else {
            frontLeft.setPower(frontLeftPower * .85);
            frontRight.setPower(frontRightPower * .85);
            rearLeft.setPower(rearLeftPower * .85);
            rearRight.setPower(rearRightPower * .85);
        }
        //END DRIVING

        /* 23-24 had gp1 control intake, but for now, not planned
        if(gamepad1.right_bumper) {
            intakeMotor.setPower(-0.7);
        }
        else if (gamepad1.x) {
            intakeMotor.setPower(-0.7);
        }
        else {
            intakeMotor.setPower(0);
        }*/

        /*START GAMEPAD 1 SAFETY LIFT CONTROLS - not crucial
        int lift1Position = liftMotor1.getCurrentPosition();
        if(gamepad1.a) {
            lift1Position = liftMotor1.getCurrentPosition();
            if (lift1Position > 1000) {
                liftMotor1.setPower(-0.75);
                liftMotor2.setPower(-0.75);
            }
            else if (lift1Position <= 1000) {
                liftMotor1.setPower(-0.5);
                liftMotor2.setPower(-0.5);
            }
            else if (lift1Position <= 500) {
                liftMotor1.setPower(-0.3);
                liftMotor2.setPower(-0.3);
            }
        }
        if (gamepad1.y) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        }
        else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
        }
        END GAMEPAD 1 SAFETY LIFT CONTROLS - not crucial*/





        //GAMEPAD 2 - Primary: Lift(Motors & Appendages)
        //START LIFT
        //targetPosition1 = liftMotor1.getCurrentPosition();
        //targetPosition2 = liftMotor2.getCurrentPosition();

        int liftMotor2Position = liftMotor2.getCurrentPosition();

        // check the limit on up and down to ensure no over running on the lift
        if (gamepad2.right_stick_y > 0.8 && liftMotor2Position < (liftMotor2StartPosition-11)) { //if stick is forward
            liftMotor1.setPower(0.8);
            liftMotor2.setPower(0.8);
        } else if (gamepad2.right_stick_y < -0.8 && liftMotor2Position > (liftMotor2EndPosition)) { //if stick is back
            liftMotor1.setPower(-0.6);
            liftMotor2.setPower(-0.6);
        }
        else {
            liftMotor1.setPower(-0.05);
            liftMotor2.setPower(-0.05);
        }
        telemetry.addData("lift1", liftMotor1.getCurrentPosition());
        telemetry.addData("lift1", liftMotor1.getPower());
        telemetry.addData("lift2", liftMotor2.getCurrentPosition());
        telemetry.addData("lift2", liftMotor2.getPower());

        /*    23-24 lift pos aware code
        if (gamepad2.right_stick_y > 0.8) {
//            liftMotor1.setPower(-1);
//            liftMotor2.setPower(-1);
            lift1Position = liftMotor1.getCurrentPosition();
            if (lift1Position < 200) {
                liftMotor1.setPower(-0.8);
                liftMotor2.setPower(-0.8);
            } else {
                liftMotor1.setPower(-1);
                liftMotor2.setPower(-1);
            }
//           else if (lift1Position <= 0) {
//                liftMotor1.setPower(0);
//                liftMotor2.setPower(0);
//            }
//            else if (lift1Position <= 50) {
//                liftMotor1.setPower(-0.35);
//                liftMotor2.setPower(-0.35);
//            }
//            else if (lift1Position <= 500) {
//                liftMotor1.setPower(-0.45);
//                liftMotor2.setPower(-0.45);
//            }
//            else if (lift1Position <= 1000) {
//                liftMotor1.setPower(-0.5);
//                liftMotor2.setPower(-0.5);
//            }
        } else if (gamepad2.right_stick_y < -0.8) {
            liftMotor1.setPower(1);
            liftMotor2.setPower(1);
        } else {
            liftMotor1.setPower(0.05);
            liftMotor2.setPower(0.05);
        }*/
        //END LIFT

        //START LIFT ATTACHMENTS CODE
        // - open claw/close claw, button (max, full close), DONE - A, B
        if (gamepad2.a) {
            liftClaw.setPosition(1); //idk positions
        } else if (gamepad2.b) {
            liftClaw.setPosition(0); //idk positions
        } //b opens, a closes

        // - rotate arm (trigger - left: rotate arm downward and hold, - right: rotate arm upward and hold)
        if (gamepad2.left_bumper) {
            liftClawRotate_Arm.setPosition(0.3); //idk positions
        } else if (gamepad2.right_bumper) {
            liftClawRotate_Arm.setPosition(1); //idk positions
        } //right rotates to floor, left rotates back up

        // - arm extender (left stick to extend/retract) WORKS, X Y
        if (gamepad2.x) {
            liftClawRotate_Claw.setPosition(1); //idk positions
        } else if (gamepad2.y) {
            liftClawRotate_Claw.setPosition(0); //idk positions
        }//x extends, y retracts

        // - rotate claw (claw itself): (bumpers, same setup as above)//DONE, WORKS FOR ROTATING, UP DOWN
        if (gamepad2.dpad_up) {
            liftClawExtender.setPosition(1); //idk positions
        } else if (gamepad2.dpad_down) {
            liftClawExtender.setPosition(0); //idk positions
        } // up - rotate to ground, down - rotate upwards

        if (gamepad2.left_stick_button) { //reset preset
            //lift min, arm retract, arm rotate down, claw rotate dpad up
            //liftMotor1.setPower(-0.6); liftMotor2.setPower(-0.6); TODO: lift min, can probably do a lift neg power for a few sec
            liftClawRotate_Claw.setPosition(0);
            liftClawRotate_Arm.setPosition(1);
            liftClawExtender.setPosition(1);
        }
        if (gamepad2.right_stick_button) { //basket extend preset
            //lift max, arm extend, arm rotate up, claw forward dpad down
            //liftMotor1.setPower(0.8); liftMotor1.setPower(0.8); TODO: lift max, can probably do a lift pos power for a few sec
            liftClawRotate_Claw.setPosition(1);
            liftClawRotate_Arm.setPosition(0.3);
            liftClawExtender.setPosition(1);
        }

        /* 23-24 lift closing/door servo code, not needed
        if (gamepad2.y) {
            rightLiftServo.setPosition(0.83);
//            leftLiftServo.setPosition(0);
        } else if (gamepad2.a) {
            rightLiftServo.setPosition(0.57);
//            leftLiftServo.setPosition(0.82);
        }

        if (gamepad2.left_bumper) {
            door.setPosition(0.7);
        } else if (gamepad2.right_bumper) {
            door.setPosition(0);
        }

        if (gamepad2.b) {
            pixelHolder.setPosition(1);
        } else if (gamepad2.x) {
            pixelHolder.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            plane.setPosition(1);
        }
        else if (gamepad2.dpad_down) {
            plane.setPosition(0.7);
        }
        else if (gamepad2.dpad_left) {
            while (true) {
                liftMotor1.setPower(-1);
                liftMotor2.setPower(-1);
            }
        }
        else if (gamepad2.dpad_right) {
            pixelHolder.setPosition(0.4);
        }*/


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), rearLeft (%.2f), frontRight (%.2f), rearRight (%.2f)", frontLeftPower, rearLeftPower, frontRightPower, rearRightPower);
        //telemetry.addData("Lift position", liftMotor1.getCurrentPosition());
        //telemetry.addData("Servo1", rightLiftServo.getPosition());
        //telemetry.addData("Servo2", leftLiftServo.getPosition());

        telemetry.update();
    }



//    public void liftToPosition(int target) {
//        int currentPosition1 = liftMotor1.getCurrentPosition();
//        int currentPosition2 = liftMotor2.getCurrentPosition();
//        while(Math.abs(currentPosition1 - target) > 6 && Math.abs(currentPosition2 - target) > 6) {
//            currentPosition1 = liftMotor1.getCurrentPosition();
//            currentPosition2 = liftMotor2.getCurrentPosition();
//            int targetPosition = target;
//            double power1 = returnPower(targetPosition, liftMotor1.getCurrentPosition());
//            double power2 = returnPower(targetPosition, liftMotor2.getCurrentPosition());
//            liftMotor1.setPower(power1);
//            liftMotor2.setPower(power2);
//            telemetry.addData("current position", currentPosition1);
//            telemetry.addData("targetPosition", targetPosition);
//            telemetry.update();
//        }
//    }



    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

