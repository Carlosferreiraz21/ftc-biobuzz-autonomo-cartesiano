package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * TankDrive: tração tanque (skid-steer) do kit bot goBILDA com LOCALIZAÇÃO NO PLANO CARTESIANO.
 *
 * Como o robô sabe onde está (odometria / dead reckoning):
 *   - Encoders dos dois motores de tração -> quanto o robô andou.
 *   - IMU do Control Hub                  -> para onde o robô aponta (θ).
 *   - A cada loop:  d = (Δesq + Δdir) / 2
 *                   x += d · cos(θ)
 *                   y += d · sin(θ)
 *
 * Convenções (padrão FTC):
 *   - Unidade: POLEGADAS (o manual e o CAD da arena usam polegadas).
 *   - Origem (0, 0) no CENTRO da arena (144 x 144 in, 36 tiles).
 *   - θ = 0 aponta para +X; θ cresce no sentido ANTI-HORÁRIO (visto de cima). Em radianos.
 *   - "Frente" do robô = sentido em que ele anda com potência POSITIVA.
 *
 * Esta classe NÃO trava o programa (sem while/sleep): driveTo() e turnTo() devem ser
 * chamados a cada loop() e retornam true quando o movimento terminou.
 */
public class TankDrive {

    // ===================== CONSTANTES FÍSICAS =====================
    // Motor goBILDA Yellow Jacket 312 RPM (19.2:1) e roda de 96 mm, iguais ao StarterBot.
    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;
    public static final double TICKS_PER_IN = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    // Calibração de distância: empurre o robô 48 in medidos com trena e veja quanto a
    // telemetria marcou. DISTANCE_SCALE = real / medido (ex.: 48 / 46.5 = 1.032).
    public static double DISTANCE_SCALE = 1.0;

    // Sentido dos motores. Teste no OdometryTest: stick para frente -> robô anda reto
    // para frente e X/Y mudam no sentido certo. Se andar para trás, troque os DOIS.
    // Se girar em vez de andar reto, troque SÓ UM deles.
    public static DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.REVERSE;

    // Montagem do Control Hub no robô (para a IMU). Ajuste para o SEU robô:
    // para onde aponta o logo REV e para onde apontam as portas USB.
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ===================== GANHOS DE CONTROLE =====================
    // Controle proporcional (P): potência = ganho × erro. Quanto mais perto, mais devagar.
    public static double DRIVE_KP = 0.06;          // potência por polegada de erro
    public static double HEADING_KP = 1.0;         // correção de rumo enquanto anda (por radiano)
    public static double TURN_KP = 0.9;            // giro no lugar (por radiano)
    public static double MIN_DRIVE_POWER = 0.10;   // abaixo disso o robô não vence o atrito
    public static double MIN_TURN_POWER = 0.12;
    public static double MAX_ACCEL = 2.0;          // rampa: quanto a potência pode subir por segundo
    public static double TURN_FIRST_ANGLE = Math.toRadians(45); // erro de rumo acima disso: gira antes de andar
    public static double NO_HEADING_FIX_DIST_IN = 4.0; // perto do alvo, para de corrigir rumo (evita "rodar" no ponto)
    public static double NEAR_LATERAL_TOLERANCE_IN = 2.0; // perto do alvo, desvio lateral aceito (tanque não anda de lado)

    public static double POSITION_TOLERANCE_IN = 1.0;
    public static double HEADING_TOLERANCE_RAD = Math.toRadians(2);
    public static double SETTLE_SECONDS = 0.15;    // tempo dentro da tolerância para considerar "chegou"

    // ===================== HARDWARE =====================
    public final DcMotor leftDrive;
    public final DcMotor rightDrive;
    private final IMU imu;

    // ===================== ESTADO DA ODOMETRIA =====================
    private double x = 0, y = 0, heading = 0;   // pose atual (in, in, rad)
    private double headingOffset = 0;           // pose.heading = yaw da IMU + offset
    private int lastLeftTicks = 0, lastRightTicks = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private double loopDt = 0.02;
    private double lastForwardCmd = 0;

    public TankDrive(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        imu = hardwareMap.get(IMU.class, "imu");

        leftDrive.setDirection(LEFT_DIRECTION);
        rightDrive.setDirection(RIGHT_DIRECTION);

        // Zera os encoders e usa RUN_WITHOUT_ENCODER: a potência vai DIRETO para o motor,
        // sem limite. Os encoders continuam contando normalmente para a odometria.
        // (RUN_USING_ENCODER limitava a velocidade máxima das rodas.)
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        imu.resetYaw();

        setPose(0, 0, 0);
    }

    // ===================== POSE =====================

    /** Define onde o robô está agora (use no início com a posição de largada). */
    public void setPose(double newX, double newY, double newHeadingRad) {
        x = newX;
        y = newY;
        heading = AngleUnit.normalizeRadians(newHeadingRad);
        headingOffset = heading - rawYaw();
        lastLeftTicks = leftDrive.getCurrentPosition();
        lastRightTicks = rightDrive.getCurrentPosition();
        loopTimer.reset();
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getHeadingDeg() { return Math.toDegrees(heading); }

    private double rawYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** CHAME UMA VEZ POR LOOP, antes de qualquer movimento. Atualiza x, y, θ. */
    public void update() {
        loopDt = Range.clip(loopTimer.seconds(), 0.001, 0.1);
        loopTimer.reset();

        int leftTicks = leftDrive.getCurrentPosition();
        int rightTicks = rightDrive.getCurrentPosition();
        double dLeft = (leftTicks - lastLeftTicks) / TICKS_PER_IN * DISTANCE_SCALE;
        double dRight = (rightTicks - lastRightTicks) / TICKS_PER_IN * DISTANCE_SCALE;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;

        double newHeading = AngleUnit.normalizeRadians(rawYaw() + headingOffset);
        // Usa o ângulo MÉDIO do trecho: mais preciso em curvas do que usar só o novo.
        double avgHeading = heading + AngleUnit.normalizeRadians(newHeading - heading) / 2.0;

        double d = (dLeft + dRight) / 2.0;
        x += d * Math.cos(avgHeading);
        y += d * Math.sin(avgHeading);
        heading = newHeading;
    }

    // ===================== MOVIMENTOS =====================

    /**
     * Potência "humana": forward > 0 anda para frente, turn > 0 gira anti-horário.
     * Se alguma roda passar de 1, escala as duas para manter a proporção.
     */
    public void setDrivePower(double forward, double turn) {
        double left = forward - turn;
        double right = forward + turn;
        double max = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        leftDrive.setPower(left / max);
        rightDrive.setPower(right / max);
    }

    public void stop() {
        setDrivePower(0, 0);
        lastForwardCmd = 0;
    }

    /**
     * Vai até o ponto (tx, ty) do plano cartesiano.
     * @param maxPower  potência máxima (0 a 1)
     * @param reverse   true = chega de ré (útil para não precisar dar meia-volta)
     * @return true quando chegou e estabilizou
     */
    public boolean driveTo(double tx, double ty, double maxPower, boolean reverse) {
        double dx = tx - x;
        double dy = ty - y;
        double dist = Math.hypot(dx, dy);

        // Posição do alvo vista PELO ROBÔ:
        //   along   = quanto o alvo está à frente (negativo = ficou para trás)
        //   lateral = quanto o alvo está para o lado (positivo = à esquerda)
        double along = dx * Math.cos(heading) + dy * Math.sin(heading);
        double lateral = -dx * Math.sin(heading) + dy * Math.cos(heading);
        boolean near = dist < NO_HEADING_FIX_DIST_IN;

        // "Chegou": dentro da tolerância, OU perto do alvo com a distância ao longo da
        // frente acertada e um desvio lateral pequeno (o tanque não anda de lado).
        boolean arrived = dist < POSITION_TOLERANCE_IN
                || (near && Math.abs(along) < POSITION_TOLERANCE_IN
                         && Math.abs(lateral) < NEAR_LATERAL_TOLERANCE_IN);
        if (arrived) {
            stop();
            return settled(true);
        }
        settled(false);

        double forward = Range.clip(DRIVE_KP * along, -maxPower, maxPower);
        if (Math.abs(forward) < MIN_DRIVE_POWER) forward = Math.signum(along) * MIN_DRIVE_POWER;

        double turn = 0;
        if (near && Math.abs(lateral) < NEAR_LATERAL_TOLERANCE_IN) {
            // Perto e quase alinhado: só acerta a distância, SEM corrigir o rumo.
            // Corrigir rumo a poucas polegadas do alvo faz o robô rodar em volta do ponto.
        } else {
            // Ângulo para onde a FRENTE (ou a TRASEIRA, se for de ré) deve apontar.
            double angleToTarget = Math.atan2(dy, dx);
            if (reverse) angleToTarget = AngleUnit.normalizeRadians(angleToTarget + Math.PI);
            double headingError = AngleUnit.normalizeRadians(angleToTarget - heading);

            // Tanque não anda de lado: se estiver muito torto, primeiro gira no lugar.
            if (Math.abs(headingError) > TURN_FIRST_ANGLE) forward = 0;

            turn = Range.clip(HEADING_KP * headingError, -maxPower, maxPower);
            if (forward == 0 && Math.abs(turn) < MIN_TURN_POWER) {
                turn = Math.signum(headingError) * MIN_TURN_POWER;
            }
        }

        setDrivePower(rampForward(forward), turn);
        return false;
    }

    /**
     * Gira no lugar até o ângulo ABSOLUTO targetRad (em relação à arena, não ao robô).
     * @return true quando chegou e estabilizou
     */
    public boolean turnTo(double targetRad, double maxPower) {
        double error = AngleUnit.normalizeRadians(targetRad - heading);

        if (Math.abs(error) < HEADING_TOLERANCE_RAD) {
            stop();
            return settled(true);
        }
        settled(false);

        double turn = Range.clip(TURN_KP * error, -maxPower, maxPower);
        if (Math.abs(turn) < MIN_TURN_POWER) turn = Math.signum(error) * MIN_TURN_POWER;

        lastForwardCmd = 0;
        setDrivePower(0, turn);
        return false;
    }

    /** Gira até apontar para o ponto (tx, ty). offsetRad = PI se o mecanismo fica atrás. */
    public boolean turnToFace(double tx, double ty, double offsetRad, double maxPower) {
        return turnTo(angleTo(tx, ty) + offsetRad, maxPower);
    }

    public double angleTo(double tx, double ty) {
        return Math.atan2(ty - y, tx - x);
    }

    public double distanceTo(double tx, double ty) {
        return Math.hypot(tx - x, ty - y);
    }

    /** Chame ao começar um movimento novo (o goTo() do autônomo já faz isso). */
    public void newMove() {
        settleTimer.reset();
        lastForwardCmd = 0;
    }

    // Rampa de aceleração: a potência só pode CRESCER MAX_ACCEL por segundo.
    // Evita patinar na arrancada (o que estragaria a odometria). Frear é livre.
    private double rampForward(double target) {
        double cmd = target;
        if (Math.abs(target) > Math.abs(lastForwardCmd) || Math.signum(target) != Math.signum(lastForwardCmd)) {
            double maxStep = MAX_ACCEL * loopDt;
            cmd = Range.clip(target, lastForwardCmd - maxStep, lastForwardCmd + maxStep);
        }
        lastForwardCmd = cmd;
        return cmd;
    }

    // Só considera "chegou" depois de SETTLE_SECONDS seguidos dentro da tolerância.
    private boolean settled(boolean inTolerance) {
        if (!inTolerance) {
            settleTimer.reset();
            return false;
        }
        return settleTimer.seconds() >= SETTLE_SECONDS;
    }
}
