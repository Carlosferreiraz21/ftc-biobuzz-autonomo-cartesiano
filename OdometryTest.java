package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * OdometryTest: calibra a odometria e MEDE os pontos da arena para o autônomo.
 *
 * ROTEIRO DE CALIBRAÇÃO (nesta ordem):
 *  1. Sentido dos motores: stick esquerdo para frente -> robô anda reto para frente.
 *     Errado? Ajuste LEFT_DIRECTION / RIGHT_DIRECTION no TankDrive.
 *  2. Encoders: empurre o robô para frente. "Ticks" dos DOIS lados devem AUMENTAR.
 *     Ficou em 0? Cabo do encoder solto ou na porta errada.
 *  3. IMU: gire o robô à mão no sentido anti-horário (visto de cima). θ deve AUMENTAR.
 *     Diminuiu ou ficou estranho? Ajuste HUB_LOGO / HUB_USB no TankDrive.
 *  4. Distância: marque 48 in com trena no chão, ande reto e compare com Y/X.
 *     DISTANCE_SCALE = 48 / valor_medido.
 *  5. Teste dos movimentos automáticos (segure o botão):
 *     Y = anda 24 in para frente (driveTo)    B = gira para +90° (turnTo)    X = gira para 0°
 *     Ajuste DRIVE_KP / TURN_KP se oscilar (diminua) ou parar antes (aumente).
 *
 * MEDIR OS PONTOS DO AUTÔNOMO ("ensinar" o caminho):
 *  - Coloque o robô na largada e ajuste START_* abaixo para os mesmos valores do autônomo.
 *  - Aperte A para zerar a pose na largada.
 *  - Dirija até o ponto de tiro / LOADING ZONE e anote X e Y da telemetria.
 *  - Copie para SHOOT_X/Y e PARK_X/Y do StarterBotAutoCartesian.
 */
@TeleOp(name = "OdometryTest", group = "Test")
public class OdometryTest extends OpMode {

    // Use os MESMOS valores de largada do autônomo.
    static final double START_X = -60, START_Y = 12, START_HEADING_DEG = 0;

    private TankDrive drive;
    private boolean moving = false;
    private double targetX, targetY;

    @Override
    public void init() {
        drive = new TankDrive(hardwareMap);
        telemetry.addLine("Coloque o robô na largada e aperte START.");
    }

    @Override
    public void start() {
        drive.setPose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    @Override
    public void loop() {
        drive.update();

        if (gamepad1.a) {
            drive.setPose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        }

        if (gamepad1.y) {
            // Marca o alvo só no momento em que o botão é apertado.
            if (!moving) {
                targetX = drive.getX() + 24 * Math.cos(drive.getHeading());
                targetY = drive.getY() + 24 * Math.sin(drive.getHeading());
                drive.newMove();
                moving = true;
            }
            drive.driveTo(targetX, targetY, 0.6, false);
        } else if (gamepad1.b) {
            drive.turnTo(Math.toRadians(90), 0.5);
        } else if (gamepad1.x) {
            drive.turnTo(0, 0.5);
        } else {
            moving = false;
            // Arcade: stick esquerdo Y = frente/trás, stick direito X = giro.
            double forward = -gamepad1.left_stick_y;   // no gamepad, para cima é negativo
            double turn = -gamepad1.right_stick_x;     // direita = horário = turn negativo
            drive.setDrivePower(forward * 0.7, turn * 0.5);
        }

        telemetry.addData("X (in)", "%.2f", drive.getX());
        telemetry.addData("Y (in)", "%.2f", drive.getY());
        telemetry.addData("θ (graus)", "%.1f", drive.getHeadingDeg());
        telemetry.addData("Ticks", "esq=%d  dir=%d",
                drive.leftDrive.getCurrentPosition(), drive.rightDrive.getCurrentPosition());
        telemetry.addLine("A = zera na largada | Y = 24 in | B = 90° | X = 0°");
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
