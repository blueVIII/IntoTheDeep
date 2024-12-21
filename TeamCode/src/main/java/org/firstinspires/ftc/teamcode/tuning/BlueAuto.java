package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Blue_Bucket_Auto", group = "BlueViii-auto")
public class BlueAuto extends LinearOpMode {

    int targetPosition = 1;
    int loop = 0;
    int currentPosition;
    boolean direction = true; //true == up
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);
        Pose2d initPos = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPos);

        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();
        // Start autonomous sequence
        while (opModeIsActive()) {
            telemetry.addData("Lift1 Position", drive.liftMotor1.getCurrentPosition());
            telemetry.addData("Lift2 Position", drive.liftMotor2.getCurrentPosition());
            telemetry.addData("Current Position", currentPosition);
            telemetry.update();
            Actions.runBlocking(
                drive.actionBuilder(initPos)
                    .lineToX(10)
                    // .afterDisp(1, setTargetPosition(1000))
                    .build()
            );
            setTargetPosition(1000);
            liftUpdate(drive);
        }
    }
    public void liftUpdate(MecanumDrive drive) {
        currentPosition = drive.liftMotor1.getCurrentPosition();
        if (targetPosition == 0){
        }
        else if (currentPosition < targetPosition && direction) {
            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
            drive.liftMotor1.setPower(power);
            drive.liftMotor2.setPower(power);

        } else if (currentPosition > targetPosition && direction) {
            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
            drive.liftMotor1.setPower(power);
            drive.liftMotor2.setPower(power);
        }
        else if (currentPosition+10 > targetPosition && direction){
            drive.liftMotor1.setPower(0.05);
            drive.liftMotor2.setPower(0.05);
        }
        else if (currentPosition+10 < targetPosition && direction){
            drive.liftMotor1.setPower(0.05);
            drive.liftMotor2.setPower(0.05);
        }
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    // lift control for getting power for motors
    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        return (error * 0.03) + (derivative * 0.0002) + 0.05;
    }
}