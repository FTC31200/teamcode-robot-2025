package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Autonomous", group = "Linear Opmode")
public class AutonomousRun extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotorEx shooterMotor;
    private DcMotor coreHEX;
    private CRServo servo;
    PIDController pid = new PIDController(.005, 0, 0, .02
            , telemetry);

    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        coreHEX = hardwareMap.get(DcMotor.class, "coreHEX");
        servo = hardwareMap.get(CRServo.class, "servo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        coreHEX.setDirection(DcMotor.Direction.REVERSE);
        servo.setDirection(CRServo.Direction.REVERSE);

        coreHEX.setPower(-1.0);
        servo.setPower(1.0);

        pid.setTolerance(145.0);

        waitForStart();

        if (opModeIsActive()) {

            leftMotor.setPower(-0.5);
            rightMotor.setPower(-0.5);
            sleep(1000);

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(500);

            shooterMotor.setPower(1.0);
            servo.setPower(1.0);
            for (int i = 0; i < 3; i++) {
                telemetry.addLine(String.format("How many shoots: %d", i + 1));
                telemetry.update();
                shoot();
                waitForBall();
                sleep(1000                                                                                                                                                                                                                                                                                                                                                                                                                                                                             );
            }
        }
        stop();
    }

    void shoot() {
        while (!pid.isAtTargetPoint) {
            telemetry.addData("ShooterSpeed", shooterMotor.getVelocity());
            telemetry.update();
            double power = pid.calculate(1800, shooterMotor.getVelocity());
            shooterMotor.setPower(power);
        }
        sleep(500);
        coreHEX.setPower(1.0);
    }

    void waitForBall() {
        while (true) {
            telemetry.addData("ShooterSpeed", shooterMotor.getVelocity());
            telemetry.update();
            pid.calculate(1800, shooterMotor.getVelocity()); // to update the ERROR, so update the isAtTargetPoint
            if (shooterMotor.getVelocity() < 1100) break;
        }
        sleep(100);
        coreHEX.setPower(-1.0);
    }
}