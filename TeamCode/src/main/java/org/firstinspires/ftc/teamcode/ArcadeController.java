
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;

@TeleOp
public class ArcadeController extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor shooterMotor;
    private DcMotor coreHEX;
    private CRServo servo;
    private boolean canSpinShooter = false;

    private double shooterSpeed = 0f;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "motorEsquerda");
        rightMotor = hardwareMap.get(DcMotor.class, "motorDireita");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        coreHEX = hardwareMap.get(DcMotor.class, "coreHEX");
        servo = hardwareMap.get(CRServo.class, "servo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        coreHEX.setDirection(DcMotor.Direction.REVERSE);
        servo.setPower(-1.0);
    }
    @Override
    public void loop() {
        // foram feitas reduções de velocidade
        double drive = -gamepad1.left_stick_y * .72;
        double turn = gamepad1.right_stick_x * .85;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0 * .85f) {
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (gamepad1.dpadDownWasPressed()) canSpinShooter = !canSpinShooter;  // se pressionar D-Pad down, can spin shooter inverte o valor (true -> false, false -> true;
        if (gamepad1.dpadLeftWasPressed()) shooterSpeed -= 0.05; // aumenta um pouco a velocidade do shooter (D-Pad left)
        if (gamepad1.dpadRightWasPressed()) shooterSpeed += 0.05; // diminue um pouco a velocidade do shooter (D-Pad right)

        // liga ou desliga o shooter
        if (canSpinShooter) shooterMotor.setPower(shooterSpeed);
        else {
            shooterSpeed = 0.0;
            shooterMotor.setPower(shooterSpeed);
        }


        // limite de velocidade para o shooter (min: 0; max: 1.0)
        if (shooterSpeed < 0.0) shooterSpeed = 1.0;
        else if (shooterSpeed > 1.0) shooterSpeed = 1.0;

        // ação de atirar (inverte a direção do core hex para levar uma bola para o shooter)
        if (gamepad1.right_trigger > 0.05f) coreHEX.setPower(1.0);
        else coreHEX.setPower(-1.0);

        telemetry.addData("Shooter speed", String.format(Locale.ENGLISH, "%f", shooterSpeed * 100));
        telemetry.update();
    }
}
