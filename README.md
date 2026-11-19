# Autônomo cartesiano — FTC BIOBUZZ 2026-27

Autônomo para o kit bot goBILDA (StarterBot, tração tanque) que navega por **coordenadas da arena**, usando apenas o hardware do kit: os **encoders** dos motores de tração e a **IMU** do Control Hub. Não precisa de nenhum sensor extra.

## Arquivos

| Arquivo | O que é | Responsabilidade |
| --- | --- | --- |
| `TankDrive.java` | Classe comum (não é OpMode) | Motores de tração, IMU, odometria (`update()`) e movimentos (`driveTo`, `turnTo`, `turnToFace`) |
| `StarterBotAutoCartesian.java` | Autônomo | Sequência da partida: pontos da arena, lançador e seguranças de tempo |
| `OdometryTest.java` | TeleOp de teste | Calibrar os sensores, testar os movimentos e medir os pontos da arena |

### Instalação (OnBot Java)

1. Conecte o computador ao Wi-Fi do Control Hub e abra `http://192.168.43.1:8080`, aba **OnBot Java**.
2. Faça upload dos três `.java` para `org/firstinspires/ftc/teamcode`.
3. Clique em **Build Everything** (chave inglesa) e espere a mensagem `Build SUCCEEDED!`.
4. Nomes esperados na configuração do robô: `left_drive`, `right_drive`, `launcher`, `intake` (motores); `windmill` (CR servo); `imu`.

## O que mudou em relação ao autônomo por tempo

| | Antes (`StarterBotAuto2`) | Agora (`StarterBotAutoCartesian`) |
| --- | --- | --- |
| Como decide parar | Quando passam X segundos | Quando chega ao ponto pedido |
| Sensores usados | Nenhum | Encoders + IMU do Control Hub |
| Sabe onde está? | Não | Sim: x, y e θ a cada ciclo |
| Bateria fraca | Anda menos e erra o ponto | Velocidade controlada (`RUN_USING_ENCODER`) |
| Velocidade | 0,5 | Até 0,8, freando sozinho perto do alvo |
| Lançador | O moinho liga após 3 s, na velocidade ou não | O moinho só alimenta com o lançador na velocidade |
| Esquecer o reset do timer | Aconteceu 3 vezes | Impossível: `goTo()` sempre zera |

## Como o robô sabe onde está (odometria)

O robô não "vê" a arena. Ele **calcula** a posição somando pequenos movimentos, ciclo a ciclo (*dead reckoning*). Para isso usa três coisas:

1. **Ponto de partida conhecido:** a pose da largada é definida no `start()`.
2. **Encoders:** 537,7 ticks por volta numa roda de 96 mm dão cerca de 45,3 ticks por polegada.
3. **IMU:** o giroscópio do Control Hub mede o ângulo real, sem depender das rodas, que patinam nos giros.

A cada ciclo (cerca de 50 por segundo), a função `update()` faz:

```
d = (Δesq + Δdir) / 2          // quanto o centro do robô andou
x = x + d · cos(θ)
y = y + d · sin(θ)              // θ = ângulo médio do trecho, lido da IMU
```

**Exemplo:** o robô está em (−60, 12) a 30°, e as rodas andaram 1,0 e 1,2 in. Então d = 1,1, x = −59,05 e y = 12,55.

## Convenções do plano cartesiano

- Arena de 144 × 144 in, com a **origem no centro**. A unidade é a **polegada**.
- θ = 0° aponta para +X e cresce no sentido **anti-horário**. No código, o ângulo é guardado em radianos.
- "Frente" do robô = o sentido em que ele anda com potência positiva.
- Os pontos são escritos para a **aliança vermelha**. A azul é calculada por espelho (inverte Y e θ, com `my()` e `mh()`). Confira na Figura 10-1 do manual se a arena é simétrica desse jeito.

## Como o robô vai até um ponto

- **`driveTo(x, y)`:** calcula a distância (`Math.hypot`) e a direção (`Math.atan2`) até o alvo. Se o erro de rumo passa de 45°, primeiro gira no lugar. Depois anda corrigindo o rumo e para a menos de 1 in do alvo. Com `reverse = true`, chega de ré.
- **`turnTo(θ)`:** gira até um ângulo **absoluto** da arena. `turnToFace(x, y)` gira para apontar a um ponto.
- **Controle proporcional (P):** potência = K_P × erro, então o robô desacelera sozinho conforme se aproxima.
- **Potência mínima:** vence o atrito, para o robô não parar antes do alvo.
- **Rampa de aceleração:** evita que a roda patine na arrancada.

## Sequência do autônomo

```
DRIVE_TO_SHOOT → AIM → SHOOT → DRIVE_TO_PARK → DONE
   (lançador        (mira     (HIVE TIP     (LEAVE + PARK
    acelerando)      na HIVE)  20 pts)       3 + 5 pts)

Saída de emergência: passou de 24 s antes do PARK → desliga o lançador e vai estacionar.
```

- `drive.update()` é sempre a primeira linha do `loop()`.
- Toda etapa tem um tempo máximo, então o autônomo nunca trava.
- Toda troca de etapa passa por `goTo()`, que zera o timer da etapa.
- Com os pontos padrão (que são seguros), o autônomo termina em cerca de 8 s. Sobra tempo para um segundo ciclo.

Pontuação do AUTO na BIOBUZZ (Competition Manual, Tabela 10-2):

| Ação | Pontos |
| --- | --- |
| LEAVE (o robô termina sem encostar na parede) | 3 |
| PARK (o robô termina pelo menos parcialmente na LOADING ZONE) | 5 |
| HIVE TIP (lançar POLLEN na CELL de cima até a HIVE tombar) | 20 |

## Calibração (rode o `OdometryTest` antes de competir)

1. **Sentido dos motores:** com o stick para frente, o robô deve andar reto para frente. Ajuste `LEFT_DIRECTION` / `RIGHT_DIRECTION`.
2. **Encoders:** ao empurrar o robô para frente, os ticks dos **dois** lados devem aumentar.
3. **IMU:** ao girar o robô no sentido anti-horário, θ deve aumentar. Ajuste `HUB_LOGO` / `HUB_USB` se não aumentar.
4. **Distância:** ande 48 in medidos com trena e calcule `DISTANCE_SCALE = 48 / valor medido`.
5. **Movimentos:** segure **Y** (anda 24 in), **B** (gira para 90°) ou **X** (gira para 0°). Se oscilar, diminua `DRIVE_KP` / `TURN_KP`.
6. **Pontos:** aperte **A** com o robô na largada, dirija até o ponto de tiro e até a LOADING ZONE e copie X e Y para o autônomo.

## Limitações

A odometria é uma estimativa: cada ciclo soma um erro pequeno, e esses erros se acumulam. As principais fontes de erro são roda patinando, diâmetro real da roda diferente do nominal, robô mal posicionado na largada e batidas em outros robôs. Na simulação, com 3% de patinagem, o robô parou a 0,7–3,2 in do alvo. Para melhorar além disso, seriam necessárias rodas de odometria (dead wheels) ou correção por AprilTags.
