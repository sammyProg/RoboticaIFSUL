// ============================================================================
// Objetivo geral do codigo
// ----------------------------------------------------------------------------
// Este programa implementa o controle de 3 juntas de um braco robotico usando:
// - IHM Nextion (3 sliders) para definir posicao alvo das juntas (em graus)
// - 3 motores de passo via drivers DRV8825 (STEP/DIR/EN) usando AccelStepper
// - 3 sensores magneticos AS5600 (via I2C) para medir os angulos reais
// - Multiplexador I2C PCA9548A para compartilhar o barramento I2C entre sensores
// - Calculo de cinematica direta via matrizes 4x4 (DH) para obter x,y,z
// - Envio para o Nextion dos angulos (t0..t5) e coordenadas (t6..t8)
//
// Fluxo logico principal:
// 1) setup: inicializa Serial, Nextion, I2C, sensores, drivers e motores
// 2) setup: le sliders uma vez e sincroniza posicao atual dos motores (evita tranco)
// 3) loop: le sliders -> converte para passos -> move motores para alvos
// 4) loop: le sensores -> corrige offsets/normaliza -> mostra no display
// 5) loop: converte graus->rad -> monta A1..A6 -> multiplica -> extrai x,y,z
// 6) loop: envia x,y,z ao display
// ============================================================================

#include <AccelStepper.h>
#include <Wire.h>
#include <AS5600.h>
#include <Nextion.h>
#include <math.h>

// Endereco do multiplexador I2C PCA9548A
#define PCA9548A_ADDRESS 0x70

// Constante PI (evita depender de definicoes externas)
#define PI 3.1415926535897932384626433832795

// Flags usadas para "ignorar" o primeiro comando de movimento em algumas juntas
// Ideia: na primeira iteracao, apenas marca como inicializado e nao move
bool init1_done = false;
bool init2_done = false;
bool init3_done = false;

// Variaveis para leitura inicial dos sliders (evitar movimento no boot)
uint32_t val00 = 0, val11 = 180, val22 = 180;

// Targets iniciais em passos (posicao alvo equivalente aos sliders no boot)
long target11 = 0, target22 = 0, target33 = 0;

// ---------------------------------------------------------------------------
// 1) Declaracao dos sensores AS5600
// ---------------------------------------------------------------------------
AS5600 as5600_1;
AS5600 as5600_2;
AS5600 as5600_3;

// ---------------------------------------------------------------------------
// 2) Declaracao do Nextion: sliders (h0,h1,h2) e textos (t0..t8)
// ---------------------------------------------------------------------------
// Sliders: representam comando de posicao (graus) para cada junta motorizada
NexSlider h0 = NexSlider(0, 1, "h0");
NexSlider h1 = NexSlider(0, 3, "h1");
NexSlider h2 = NexSlider(0, 6, "h2");

// Lista de elementos touch que podem gerar eventos (na pratica aqui so le valor)
NexTouch *nrx_listen_list[] = {
  &h0, &h1, &h2,
  NULL
};

// Textos no Nextion:
// t0..t5: angulos das 6 articulacoes (3 reais + 3 fixas)
// t6..t8: coordenadas x,y,z do efetuador final
NexText t0 = NexText(0, 2, "t0");
NexText t1 = NexText(0, 4, "t1");
NexText t2 = NexText(0, 5, "t2");
NexText t3 = NexText(0, 7, "t3");
NexText t4 = NexText(0, 8, "t4");
NexText t5 = NexText(0, 9, "t5");
NexText t6 = NexText(0, 11, "t6");
NexText t7 = NexText(0, 10, "t7");
NexText t8 = NexText(0, 12, "t8");

// ---------------------------------------------------------------------------
// 3) Declaracao dos motores (DRV8825 + AccelStepper)
// ---------------------------------------------------------------------------
// Pinos ENABLE de cada driver (LOW habilita, conforme seu codigo)
#define PIN_EN1 56
#define PIN_STEP1 57
#define PIN_DIR1 58

#define PIN_EN2 59
#define PIN_STEP2 60
#define PIN_DIR2 61

#define PIN_EN3 62
#define PIN_STEP3 63
#define PIN_DIR3 64

// AccelStepper em modo DRIVER: so precisa STEP e DIR
AccelStepper stepper1(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_STEP3, PIN_DIR3);

// ---------------------------------------------------------------------------
// 4) Selecao de canal no PCA9548A
// ---------------------------------------------------------------------------
// O PCA9548A permite selecionar qual "ramo" I2C fica ativo.
// Assim, varios AS5600 podem ter o mesmo endereco e ainda assim funcionar.
void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;                 // PCA9548A tem 8 canais: 0..7
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);                // habilita apenas o canal desejado
  Wire.endTransmission();
}

// ---------------------------------------------------------------------------
// 5) Conversao do AS5600 (0..4095) para graus (0..360)
// ---------------------------------------------------------------------------
// O AS5600 fornece 12 bits de angulo: 0 a 4095 (teorico).
// Mapeia linearmente para graus.
float mapAngleTo360(uint16_t rawAngle) {
  return (rawAngle * 360.0) / 4095.0;
}

// ---------------------------------------------------------------------------
// 6) Multiplicacao de matrizes 4x4 (cinematica direta)
// ---------------------------------------------------------------------------
// Multiplica a*b e armazena em result.
// Usa temp para evitar sobrescrever caso result seja o mesmo ponteiro.
void multiplyMatrices(double a[4][4], double b[4][4], double result[4][4]) {
  double temp[4][4];

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      temp[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        temp[i][j] += a[i][k] * b[k][j];
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      result[i][j] = temp[i][j];
    }
  }
}

void setup() {

  // Serial para debug/monitoramento (nao esta sendo usado no loop)
  Serial.begin(9600);

  // Inicializa comunicacao com Nextion
  nexInit();

  // -----------------------
  // I2C para os AS5600
  // -----------------------
  Wire.begin();                             // inicia I2C como master
  as5600_1.begin(AS5600_DEFAULT_ADDRESS);    // instancia sensor 1
  as5600_2.begin(AS5600_DEFAULT_ADDRESS);    // instancia sensor 2
  as5600_3.begin(AS5600_DEFAULT_ADDRESS);    // instancia sensor 3
  selectI2CChannel(0);                       // seleciona canal 0 como padrao

  // -----------------------
  // Configuracao DRV8825
  // -----------------------
  pinMode(PIN_EN1, OUTPUT);
  digitalWrite(PIN_EN1, LOW);               // habilita driver 1

  pinMode(PIN_EN2, OUTPUT);
  digitalWrite(PIN_EN2, LOW);               // habilita driver 2

  pinMode(PIN_EN3, OUTPUT);
  digitalWrite(PIN_EN3, LOW);               // habilita driver 3

  // Ajuste de limites de velocidade e aceleracao (unidade: passos/seg e passos/seg^2)
  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(100.0);

  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(500.0);

  stepper3.setMaxSpeed(500.0);
  stepper3.setAcceleration(100.0);

  // =======================================================
  // Leitura inicial dos sliders para evitar movimento no boot
  // -------------------------------------------------------
  // Problema tipico: ao ligar, o motor esta em uma posicao, mas o software
  // "acha" que esta em 0. Ao ler o slider e setar como posicao atual, voce
  // sincroniza referencia e evita um deslocamento brusco.
  // =======================================================

  h0.getValue(&val00);
  h1.getValue(&val11);
  h2.getValue(&val22);

  // Conversao graus -> passos
  // Observacao: usa sinal negativo. Isso define convencao de sentido do motor.
  // As escalas (25600, 102400, 25600) representam passos por volta (inclui microstepping e reducoes).
  target11 = -map(val00, 0, 360, 0, 25600);
  target22 = -map(val11, 0, 360, 0, 102400);
  target33 = -map(val22, 0, 360, 0, 25600);

  // Sincroniza posicao atual interna do AccelStepper com o valor lido do slider
  // Assim, a primeira referencia nao gera movimento inesperado.
  stepper1.setCurrentPosition(target11);
  stepper2.setCurrentPosition(target22);
  stepper3.setCurrentPosition(target33);
}

void loop() {

  // -------------------------------------------------------------------------
  // 1) Leitura dos sliders (comandos do usuario)
  // -------------------------------------------------------------------------
  uint32_t val0, val1, val2;
  h0.getValue(&val0);
  h1.getValue(&val1);
  h2.getValue(&val2);

  // -------------------------------------------------------------------------
  // 2) Conversao de graus (0..360) para passos (0..N)
  // -------------------------------------------------------------------------
  // N define quantos passos correspondem a uma volta completa da junta.
  // Sinal negativo inverte o sentido do movimento.
  long target1 = -map(val0, 0, 360, 0, 25600);
  long target2 = -map(val1, 0, 360, 0, 102400);
  long target3 = -map(val2, 0, 360, 0, 25600);

  // -------------------------------------------------------------------------
  // 3) Movimento dos motores ate a nova posicao
  // -------------------------------------------------------------------------
  // runToNewPosition() e bloqueante: ele so retorna quando atingir o alvo.
  // Consequencia: enquanto um motor esta indo ao alvo, o resto do loop "para".
  // Isso reduz responsividade de leitura de sensores e atualizacao do display.

  // Junta 1: aqui nao ha logica de ignorar o primeiro movimento, ele sempre executa
  stepper1.runToNewPosition(target1);

  // Junta 2: ignora o primeiro comando (marca init2_done e nao move na 1a vez)
  if (!init2_done) {
    init2_done = true;
  } else {
    stepper2.runToNewPosition(target2);
  }

  // Junta 3: idem
  if (!init3_done) {
    init3_done = true;
  } else {
    stepper3.runToNewPosition(target3);
  }

  // -------------------------------------------------------------------------
  // 4) Leitura dos sensores (AS5600) via multiplexador PCA9548A
  // -------------------------------------------------------------------------
  // Sensor 1: possui tratamento de "voltas" para obter angulo absoluto
  // (detecao de rollover 359->0 e 0->359).

  selectI2CChannel(0);
  uint16_t raw1 = as5600_1.readAngle();

  // Converte para graus (nota: aqui voce usa 4096.0 no denominador)
  float ang_motor = (float)raw1 * 360.0 / 4096.0;

  // Variaveis estaticas persistem entre iteracoes do loop
  static float last_ang = ang_motor;
  static long voltas = 0;

  // Diferenca entre amostras
  float diff = ang_motor - last_ang;

  // Detecao de rollover:
  // - Se diff > 180: provavelmente saiu de perto de 0 e foi para perto de 360 (na amostra anterior)
  // - Se diff < -180: provavelmente saiu de perto de 360 e foi para perto de 0
  if (diff > 180) voltas--;
  else if (diff < -180) voltas++;

  last_ang = ang_motor;

  // Angulo total do motor considerando voltas completas
  float ang_motor_total = ang_motor + voltas * 360.0;

  // Converte angulo do motor para angulo da junta considerando reducao 4:1
  float ang_junta = ang_motor_total / 4.0;

  // Aplica offset mecanico (zero fisico) da junta
  ang_junta -= 62.4;

  // Normaliza para 0..360 usando resto (fmod)
  ang_junta = fmod(ang_junta, 360.0);
  if (ang_junta < 0) ang_junta += 360.0;

  // Define convencao de sinal para angle1 (inversao)
  float angle1 = -ang_junta;

  // Normaliza novamente para 0..360
  angle1 = fmod(angle1, 360.0);
  if (angle1 < 0) {
    angle1 += 360.0;
  }

  // Sensor 2: leitura simples, com offset e normalizacao
  selectI2CChannel(1);
  uint16_t raw2 = as5600_2.readAngle();
  float angle2 = mapAngleTo360(raw2) - 278.2;

  if (angle2 < 0) {
    angle2 += 360.0;
  }

  // Sensor 3: leitura simples, offset e normalizacao via fmod
  selectI2CChannel(2);
  uint16_t raw3 = as5600_3.readAngle();
  float angle3 = mapAngleTo360(raw3) - 290.5;

  angle3 = fmod(angle3, 360.0);
  if (angle3 < 0) {
    angle3 += 360.0;
  }

  // -------------------------------------------------------------------------
  // 5) Angulos fixos para juntas nao sensorizadas (juntas 4..6)
  // -------------------------------------------------------------------------
  float angle4 = 0.0;
  float angle5 = 0.0;
  float angle6 = 0.0;

  // -------------------------------------------------------------------------
  // 6) Envia angulos ao Nextion (t0..t5)
  // -------------------------------------------------------------------------
  t0.setText(String(angle1, 1).c_str());
  t1.setText(String(angle2, 1).c_str());
  t2.setText(String(angle3, 1).c_str());
  t3.setText(String(angle4, 1).c_str());
  t4.setText(String(angle5, 1).c_str());
  t5.setText(String(angle6, 1).c_str());

  // -------------------------------------------------------------------------
  // 7) Converte graus para radianos (necessario para sin/cos)
  // -------------------------------------------------------------------------
  float thetas_rad[6] = {
    angle1 * PI / 180.0,
    angle2 * PI / 180.0,
    angle3 * PI / 180.0,
    angle4 * PI / 180.0,
    angle5 * PI / 180.0,
    angle6 * PI / 180.0
  };

  // -------------------------------------------------------------------------
  // 8) Montagem das matrizes A1..A6 (transformacoes homogeneas DH)
  // -------------------------------------------------------------------------
  // Cada Ai representa a transformacao do elo i-1 para o elo i.
  // Voce ja embutiu offsets (ex: theta2 - PI/2, theta6 + PI) e parametros a/d/alpha.

  double A1[4][4] = {
    {cos(thetas_rad[0]), -sin(thetas_rad[0])*cos(-PI/2),  sin(thetas_rad[0])*sin(-PI/2),  0*cos(thetas_rad[0])},
    {sin(thetas_rad[0]),  cos(thetas_rad[0])*cos(-PI/2), -cos(thetas_rad[0])*sin(-PI/2),  0*sin(thetas_rad[0])},
    {0,                   sin(-PI/2),                   cos(-PI/2),                     0.3991},
    {0,                   0,                            0,                              1}
  };

  double A2[4][4] = {
    {cos(thetas_rad[1]-PI/2), -sin(thetas_rad[1]-PI/2)*cos(0), sin(thetas_rad[1]-PI/2)*sin(0), 0.448*cos(thetas_rad[1]-PI/2)},
    {sin(thetas_rad[1]-PI/2),  cos(thetas_rad[1]-PI/2)*cos(0), -cos(thetas_rad[1]-PI/2)*sin(0), 0.448*sin(thetas_rad[1]-PI/2)},
    {0,                        sin(0),                       cos(0),                       0},
    {0,                        0,                            0,                            1}
  };

  double A3[4][4] = {
    {cos(thetas_rad[2]), -sin(thetas_rad[2])*cos(-PI/2),  sin(thetas_rad[2])*sin(-PI/2), 0.042*cos(thetas_rad[2])},
    {sin(thetas_rad[2]),  cos(thetas_rad[2])*cos(-PI/2), -cos(thetas_rad[2])*sin(-PI/2), 0.042*sin(thetas_rad[2])},
    {0,                   sin(-PI/2),                   cos(-PI/2),                     0},
    {0,                   0,                            0,                              1}
  };

  double A4[4][4] = {
    {cos(thetas_rad[3]), -sin(thetas_rad[3])*cos(PI/2),  sin(thetas_rad[3])*sin(PI/2),  0*cos(thetas_rad[3])},
    {sin(thetas_rad[3]),  cos(thetas_rad[3])*cos(PI/2), -cos(thetas_rad[3])*sin(PI/2),  0*sin(thetas_rad[3])},
    {0,                   sin(PI/2),                    cos(PI/2),                    0.451},
    {0,                   0,                             0,                             1}
  };

  double A5[4][4] = {
    {cos(thetas_rad[4]), -sin(thetas_rad[4])*cos(-PI/2),  sin(thetas_rad[4])*sin(-PI/2),  0*cos(thetas_rad[4])},
    {sin(thetas_rad[4]),  cos(thetas_rad[4])*cos(-PI/2), -cos(thetas_rad[4])*sin(-PI/2),  0*sin(thetas_rad[4])},
    {0,                   sin(-PI/2),                    cos(-PI/2),                     0},
    {0,                   0,                             0,                              1}
  };

  double A6[4][4] = {
    {cos(thetas_rad[5]+PI), -sin(thetas_rad[5]+PI)*cos(0),  sin(thetas_rad[5]+PI)*sin(0), 0*cos(thetas_rad[5]+PI)},
    {sin(thetas_rad[5]+PI),  cos(thetas_rad[5]+PI)*cos(0), -cos(thetas_rad[5]+PI)*sin(0), 0*sin(thetas_rad[5]+PI)},
    {0,                      sin(0),                        cos(0),                       0.082},
    {0,                      0,                             0,                             1}
  };

  // -------------------------------------------------------------------------
  // 9) Cinematica direta: T = A1*A2*A3*A4*A5*A6
  // -------------------------------------------------------------------------
  double T[4][4];
  multiplyMatrices(A1, A2, T);
  multiplyMatrices(T, A3, T);
  multiplyMatrices(T, A4, T);
  multiplyMatrices(T, A5, T);
  multiplyMatrices(T, A6, T);

  // -------------------------------------------------------------------------
  // 10) Extrai posicao do efetuador (x,y,z) da ultima coluna da matriz T
  // -------------------------------------------------------------------------
  float x = T[0][3];
  float y = T[1][3];
  float z = T[2][3];

  // -------------------------------------------------------------------------
  // 11) Envia x,y,z ao Nextion (t6..t8)
  // -------------------------------------------------------------------------
  t6.setText(String(x, 3).c_str());
  t7.setText(String(y, 3).c_str());
  t8.setText(String(z, 3).c_str());
}
