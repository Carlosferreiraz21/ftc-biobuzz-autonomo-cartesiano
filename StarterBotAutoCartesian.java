package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autônomo BIOBUZZ 2026-27 com navegação por PLANO CARTESIANO (encoder + IMU).
 *
 * Pontos do AUTO (Competition Manual, Tabela 10-2):
 *   LEAVE     3 pts  -> no fim do AUTO, robô não encosta na parede
 *   PARK      5 pts  -> no fim do AUTO, robô ao menos parcialmente na LOADING ZONE
 *   HIVE TIP 20 pts  -> lançar POLLEN na CELL voltada para cima até a HIVE tombar
 * Cada robô começa com 4 POLLEN pré-carregados. AUTO dura 30 s.
 *
 * Sequência:
 *   1. DRIVE_TO_SHOOT  vai até o ponto de lançamento (lançador já acelerando)
 *   2. AIM             gira para mirar na HIVE
 *   3. SHOOT           alimenta o moinho SÓ quando o lançador está na velocidade
 *   4. DRIVE_TO_PARK   vai até a LOADING ZONE  (LEAVE + PARK)
 *   5. DONE
 *   Segurança: passou de PARK_DEADLINE_S, larga o que estiver fazendo e vai estacionar.
 *
 * ANTES DE COMPETIR: meça os pontos com o OdometryTest (veja instruções lá) e
 * substitua os valores marcados com "MEDIR". Os valores padrão são SEGUROS: o robô
 * atira de onde larga e anda 24 in para frente para sair da parede.
 */
@Autonomous(name = "StarterBotAutoCartesian", group = "StarterBot")
public class StarterBotAutoCartesian extends OpMode {

    // ===================== PONTOS DA ARENA (ALIANÇA VERMELHA) =====================
    // Polegadas, origem no centro da arena, θ em graus (0 = +X, anti-horário positivo).
    // A aliança azul é calculada automaticamente pelo espelho (mirrorY / mirrorHeading).
    static final double START_X = -60, START_Y = 12, START_HEADING_DEG = 0;   // MEDIR
    static final double SHOOT_X = -60, SHOOT_Y = 12;                          // MEDIR (padrão: atira da largada)
    static final double HIVE_X  =  60, HIVE_Y  = 12;                          // MEDIR (alvo da mira)
    static final double PARK_X  = -36, PARK_Y  = 12;                          // MEDIR (dentro da LOADING ZONE)
    static final boolean PARK_IN_REVERSE = false; // true = estaciona de ré (economiza a meia-volta)

    // Se o lançador atira pela TRASEIRA do robô, use 180. Se pela frente, 0.
    static final double LAUNCHER_FACING_DEG = 0;                              // CONFERIR

    // ===================== VELOCIDADES =====================
    static final double DRIVE_MAX_POWER = 0.8;   // pode subir: o controle P freia sozinho perto do alvo
    static final double TURN_MAX_POWER = 0.6;

    // ===================== LANÇADOR =====================
    static final double LAUNCHER_TARGET_VELOCITY = 285;   // ticks/s (valor que vocês já usam)
    // Histerese do moinho: COMEÇA a alimentar acima de START e só PAUSA abaixo de STOP.
    // Cada POLLEN lançado freia o shooter um pouco; a faixa entre os dois evita que o
    // moinho fique parando e voltando ("engasgando") a cada disparo.
    static final double FEED_START_VELOCITY = 270;        // ~95% do alvo
    static final double FEED_STOP_VELOCITY = 240;         // ~85% do alvo
    static final double FEED_SECONDS = 5.0;               // tempo ALIMENTANDO (só conta com o lançador na velocidade)
    static final boolean INTAKE_HELPS_FEED = false;       // true = intake empurra o POLLEN durante o tiro
    static final double INTAKE_POWER = 1.0;

    // ===================== TEMPOS DE SEGURANÇA =====================
    static final double PARK_DEADLINE_S = 24.0;   // depois disso vai direto estacionar
    static final double DRIVE_TIMEOUT_S = 5.0;    // nenhum trecho pode travar o autônomo
    static final double AIM_TIMEOUT_S = 2.0;
    static final double SHOOT_TIMEOUT_S = 9.0;

    // ===================== ESTADO =====================
    private enum Step { DRIVE_TO_SHOOT, AIM, SHOOT, DRIVE_TO_PARK, DONE }
    private Step step;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    private TankDrive drive;
    private DcMotorEx launcher;
    private CRServo windmillServo;
    private DcMotor intake;   // null se não existir na configuração

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private double feedSeconds = 0;
    private boolean feeding = false;   // moinho alimentando agora? (histerese)

    @Override
    public void init() {
        drive = new TankDrive(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setPower(0);

        intake = hardwareMap.tryGet(DcMotor.class, "intake");
        if (intake != null) intake.setPower(0);

        telemetry.addData("Status", "Inicializado");
        telemetry.addData("Intake", (intake != null) ? "encontrado" : "NÃO encontrado (confira o nome)");
    }

    @Override
    public void init_loop() {
        windmillServo.setPower(0);

        if (gamepad1.b) alliance = Alliance.RED;
        else if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Aliança (X = azul, B = vermelha)", alliance);
        telemetry.addData("Largada", "x=%.1f  y=%.1f  θ=%.0f°",
                START_X, my(START_Y), Math.toDegrees(mh(Math.toRadians(START_HEADING_DEG))));
        telemetry.addLine("Robô PARADO na posição de largada antes do START.");
    }

    @Override
    public void start() {
        // A pose é definida no start (e não no init) para pegar a aliança escolhida.
        drive.setPose(START_X, my(START_Y), mh(Math.toRadians(START_HEADING_DEG)));
        autoTimer.reset();
        goTo(Step.DRIVE_TO_SHOOT);
    }

    @Override
    public void loop() {
        drive.update();   // SEMPRE primeiro: atualiza x, y, θ

        // Segurança de tempo: garante os 8 pts de LEAVE + PARK.
        if (autoTimer.seconds() > PARK_DEADLINE_S && step.ordinal() < Step.DRIVE_TO_PARK.ordinal()) {
            stopShooter();
            goTo(Step.DRIVE_TO_PARK);
        }

        switch (step) {
            case DRIVE_TO_SHOOT:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);   // acelera enquanto anda
                if (drive.driveTo(SHOOT_X, my(SHOOT_Y), DRIVE_MAX_POWER, false)
                        || stepTimer.seconds() > DRIVE_TIMEOUT_S) {
                    goTo(Step.AIM);
                }
                break;

            case AIM:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (drive.turnToFace(HIVE_X, my(HIVE_Y), Math.toRadians(LAUNCHER_FACING_DEG), TURN_MAX_POWER)
                        || stepTimer.seconds() > AIM_TIMEOUT_S) {
                    drive.stop();
                    feedSeconds = 0;
                    feeding = false;
                    feedTimer.reset();
                    goTo(Step.SHOOT);
                }
                break;

            case SHOOT:
                drive.stop();
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                double velocity = launcher.getVelocity();

                // Só alimenta com o lançador na velocidade: cada POLLEN sai com a mesma força.
                if (!feeding && velocity >= FEED_START_VELOCITY) feeding = true;
                else if (feeding && velocity < FEED_STOP_VELOCITY) feeding = false;

                windmillServo.setPower(feeding ? 1 : 0);
                if (intake != null && INTAKE_HELPS_FEED) intake.setPower(feeding ? INTAKE_POWER : 0);

                if (feeding) feedSeconds += feedTimer.seconds();
                feedTimer.reset();

                if (feedSeconds >= FEED_SECONDS || stepTimer.seconds() > SHOOT_TIMEOUT_S) {
                    stopShooter();
                    goTo(Step.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                if (drive.driveTo(PARK_X, my(PARK_Y), DRIVE_MAX_POWER, PARK_IN_REVERSE)
                        || stepTimer.seconds() > DRIVE_TIMEOUT_S) {
                    drive.stop();
                    goTo(Step.DONE);
                }
                break;

            case DONE:
                drive.stop();
                stopShooter();
                break;
        }

        telemetry.addData("Etapa", step);
        telemetry.addData("Pose", "x=%.1f in  y=%.1f in  θ=%.1f°", drive.getX(), drive.getY(), drive.getHeadingDeg());
        telemetry.addData("Lançador", "%.0f / %.0f ticks/s", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Tempo", "auto %.1f s | etapa %.1f s", autoTimer.seconds(), stepTimer.seconds());
        telemetry.addData("Aliança", alliance);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        stopShooter();
    }

    // ===================== AUXILIARES =====================

    /** Troca de etapa: SEMPRE zera o timer da etapa e prepara o próximo movimento. */
    private void goTo(Step next) {
        step = next;
        stepTimer.reset();
        drive.newMove();
    }

    private void stopShooter() {
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        if (intake != null) intake.setPower(0);
    }

    // Espelho da aliança azul: inverte Y e o ângulo (reflexão no eixo X).
    // CONFERIR no desenho da arena (Figura 10-1) se a BIOBUZZ é espelhada assim.
    private double my(double redY) {
        return (alliance == Alliance.RED) ? redY : -redY;
    }

    private double mh(double redHeadingRad) {
        return (alliance == Alliance.RED) ? redHeadingRad : -redHeadingRad;
    }
}
