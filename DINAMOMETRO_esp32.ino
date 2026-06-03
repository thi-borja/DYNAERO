// ============================================================
//  DINAMÔMETRO DE MOTOR E HÉLICE - RC AIRCRAFT
//  Microcontrolador: ESP32
//  Sensores: 2x Célula de Carga PE-130 20kg + 2x HX711
//
//  PROTOCOLO SERIAL (recebido do Python):
//    TARE          → zera ambas as células
//    TARE_E        → zera apenas empuxo
//    TARE_T        → zera apenas torque
//    CAL_E:<fator>:<raw_vazio> → define fator e zero do empuxo
//    CAL_T:<fator>:<raw_vazio> → define fator e zero da força de torque
//    CAL3_E:<raw_vazio>:<raw1>:<N1>:<raw2>:<N2>:<raw3>:<N3> → calibracao multiponto do empuxo
//    CAL3_T:<raw_vazio>:<raw1>:<N1>:<raw2>:<N2>:<raw3>:<N3> → calibracao multiponto da forca de torque
//    RAW_E         → envia leitura raw do empuxo (para calibração)
//    RAW_T         → envia leitura raw do torque (para calibração)
//    RESET         → reinicia o ESP32
// ============================================================

#include "HX711.h"
#include <math.h>

// ------ CÉLULA DE CARGA 1: EMPUXO ------
// DT  → RX2 = GPIO 16
// SCK → TX2 = GPIO 17
#define PINO_DT  16
#define PINO_SCK 17

// ------ CÉLULA DE CARGA 2: TORQUE ------
// DT  → D4 = GPIO 4
// SCK → D5 = GPIO 5
#define PINO_DT_TORQUE  4
#define PINO_SCK_TORQUE 5

// ------ INSTÂNCIAS HX711 ------
HX711 balanca;        // empuxo
HX711 balanca_torque; // torque

// ------ FATORES DE CALIBRAÇÃO ------
// Unidade dos fatores: contagens RAW por Newton.
// Depois de calibrado, get_units() retorna força em N.
float fator_calibracao        = -1000.0; // empuxo
float fator_calibracao_torque = -1000.0; // força na célula do torque

// ------ CALIBRACAO MULTIPONTO ------
// Usa zero + 3 pesos conhecidos para corrigir a leitura por interpolacao.
const byte PONTOS_CALIBRACAO = 4;

struct CalibracaoMultiponto {
  bool ativa;
  long raw_zero;
  float delta[PONTOS_CALIBRACAO];
  float forca[PONTOS_CALIBRACAO];
};

CalibracaoMultiponto cal_empuxo = {false, 0, {0, 0, 0, 0}, {0, 0, 0, 0}};
CalibracaoMultiponto cal_torque = {false, 0, {0, 0, 0, 0}, {0, 0, 0, 0}};

// ------ PARAMETRO DO BRACO DE TORQUE ------
// Altere este valor para a distancia entre o eixo do motor e o ponto
// onde a celula de carga de torque aplica/le a forca.
const float BRACO_TORQUE_MM = 62.0;
const float BRACO_TORQUE_M  = BRACO_TORQUE_MM / 1000.0;

// ------ TAXA DE ENVIO SERIAL ------
// Limita o fluxo de DATA para nao atrasar respostas RAW/CAL na serial.
const unsigned long INTERVALO_ENVIO_MS = 60;
const byte AMOSTRAS_RAW = 20;
const byte AMOSTRAS_TARA = 20;
const byte AMOSTRAS_LEITURA = 2;

// ------ VARIÁVEL DE TEMPO ------
unsigned long tempo_inicio = 0;
unsigned long ultimo_envio = 0;

// ============================================================
long lerRawEstavel(HX711 &sensor) {
  return sensor.read_average(AMOSTRAS_RAW);
}

void zerarSensor(HX711 &sensor, CalibracaoMultiponto &cal) {
  long offset = sensor.read_average(AMOSTRAS_TARA);
  sensor.set_offset(offset);
  if (cal.ativa) {
    cal.raw_zero = offset;
  }
}

float interpolarSegmento(float x, float x0, float y0, float x1, float y1) {
  float dx = x1 - x0;
  if (fabs(dx) < 0.001) {
    return y0;
  }
  return y0 + (x - x0) * (y1 - y0) / dx;
}

void ordenarPontos(CalibracaoMultiponto &cal) {
  for (byte i = 0; i < PONTOS_CALIBRACAO - 1; i++) {
    for (byte j = i + 1; j < PONTOS_CALIBRACAO; j++) {
      if (cal.delta[j] < cal.delta[i]) {
        float delta_tmp = cal.delta[i];
        float forca_tmp = cal.forca[i];
        cal.delta[i] = cal.delta[j];
        cal.forca[i] = cal.forca[j];
        cal.delta[j] = delta_tmp;
        cal.forca[j] = forca_tmp;
      }
    }
  }
}

float calcularForcaMultiponto(long raw, const CalibracaoMultiponto &cal) {
  float delta = (float)(raw - cal.raw_zero);

  if (delta <= cal.delta[0]) {
    return interpolarSegmento(delta, cal.delta[0], cal.forca[0], cal.delta[1], cal.forca[1]);
  }

  for (byte i = 0; i < PONTOS_CALIBRACAO - 1; i++) {
    if (delta <= cal.delta[i + 1]) {
      return interpolarSegmento(delta, cal.delta[i], cal.forca[i], cal.delta[i + 1], cal.forca[i + 1]);
    }
  }

  return interpolarSegmento(
    delta,
    cal.delta[PONTOS_CALIBRACAO - 2],
    cal.forca[PONTOS_CALIBRACAO - 2],
    cal.delta[PONTOS_CALIBRACAO - 1],
    cal.forca[PONTOS_CALIBRACAO - 1]
  );
}

float lerForcaCalibrada(HX711 &sensor, const CalibracaoMultiponto &cal) {
  if (!cal.ativa) {
    return sensor.get_units(AMOSTRAS_LEITURA);
  }

  long raw = sensor.read_average(AMOSTRAS_LEITURA);
  return calcularForcaMultiponto(raw, cal);
}

bool lerCamposCal3(String payload, long &raw_zero, long raw_pesos[3], float forcas[3]) {
  String campos[7];
  int inicio = 0;

  for (byte i = 0; i < 7; i++) {
    int fim = payload.indexOf(':', inicio);
    if (fim < 0 && i < 6) {
      return false;
    }

    campos[i] = (fim < 0) ? payload.substring(inicio) : payload.substring(inicio, fim);
    campos[i].trim();
    if (campos[i].length() == 0) {
      return false;
    }

    inicio = fim + 1;
  }

  raw_zero = campos[0].toInt();
  for (byte i = 0; i < 3; i++) {
    raw_pesos[i] = campos[1 + i * 2].toInt();
    forcas[i] = campos[2 + i * 2].toFloat();
  }
  return true;
}

void aplicarCalibracaoMultiponto(
  HX711 &sensor,
  CalibracaoMultiponto &cal,
  String payload,
  const char *nome
) {
  long raw_zero;
  long raw_pesos[3];
  float forcas[3];

  if (!lerCamposCal3(payload, raw_zero, raw_pesos, forcas)) {
    Serial.print("STATUS:Formato CAL3 invalido para ");
    Serial.println(nome);
    return;
  }

  CalibracaoMultiponto nova = {true, raw_zero, {0, 0, 0, 0}, {0, 0, 0, 0}};
  nova.delta[0] = 0;
  nova.forca[0] = 0;

  for (byte i = 0; i < 3; i++) {
    if (forcas[i] == 0) {
      Serial.print("STATUS:Forca de referencia invalida para ");
      Serial.println(nome);
      return;
    }

    float delta = (float)(raw_pesos[i] - raw_zero);
    if (delta == 0) {
      Serial.print("STATUS:RAW de referencia igual ao zero em ");
      Serial.println(nome);
      return;
    }

    for (byte j = i + 1; j < 3; j++) {
      if (raw_pesos[i] == raw_pesos[j] || fabs(forcas[i] - forcas[j]) < 0.0001) {
        Serial.print("STATUS:Pontos repetidos na calibracao ");
        Serial.println(nome);
        return;
      }
    }

    nova.delta[i + 1] = delta;
    nova.forca[i + 1] = forcas[i];
  }

  ordenarPontos(nova);
  for (byte i = 0; i < PONTOS_CALIBRACAO - 1; i++) {
    if (fabs(nova.delta[i + 1] - nova.delta[i]) < 0.001) {
      Serial.print("STATUS:Pontos RAW invalidos na calibracao ");
      Serial.println(nome);
      return;
    }
  }

  cal = nova;
  sensor.set_offset(raw_zero);

  Serial.print("STATUS:Calibracao 3 pontos aplicada em ");
  Serial.println(nome);
}

void aplicarCalibracao(HX711 &sensor, float &fator, CalibracaoMultiponto &cal, String payload, const char *nome) {
  int separador = payload.indexOf(':');
  float novo_fator = payload.toFloat();

  if (novo_fator == 0) {
    Serial.print("STATUS:Fator invalido para ");
    Serial.println(nome);
    return;
  }

  fator = novo_fator;
  cal.ativa = false;
  sensor.set_scale(fator);

  if (separador >= 0) {
    long offset = payload.substring(separador + 1).toInt();
    sensor.set_offset(offset);
    Serial.print("STATUS:Zero ");
    Serial.print(nome);
    Serial.print(" definido pelo RAW vazio: ");
    Serial.println(offset);
  }

  Serial.print("STATUS:Fator ");
  Serial.print(nome);
  Serial.print(" definido (raw/N): ");
  Serial.println(fator, 6);
}

// ============================================================
void processarComando(String cmd) {
  cmd.trim();

  if (cmd == "TARE") {
    zerarSensor(balanca, cal_empuxo);
    zerarSensor(balanca_torque, cal_torque);
    Serial.println("STATUS:Tara realizada em ambas as celulas");

  } else if (cmd == "TARE_E") {
    zerarSensor(balanca, cal_empuxo);
    Serial.println("STATUS:Tara do empuxo realizada");

  } else if (cmd == "TARE_T") {
    zerarSensor(balanca_torque, cal_torque);
    Serial.println("STATUS:Tara do torque realizada");

  } else if (cmd.startsWith("CAL3_E:")) {
    aplicarCalibracaoMultiponto(balanca, cal_empuxo, cmd.substring(7), "empuxo");

  } else if (cmd.startsWith("CAL3_T:")) {
    aplicarCalibracaoMultiponto(balanca_torque, cal_torque, cmd.substring(7), "torque");

  } else if (cmd.startsWith("CAL_E:")) {
    aplicarCalibracao(balanca, fator_calibracao, cal_empuxo, cmd.substring(6), "empuxo");

  } else if (cmd.startsWith("CAL_T:")) {
    aplicarCalibracao(balanca_torque, fator_calibracao_torque, cal_torque, cmd.substring(6), "torque");

  } else if (cmd == "RAW_E") {
    long raw = lerRawEstavel(balanca);
    Serial.print("RAW_E:");
    Serial.println(raw);
    Serial.flush();

  } else if (cmd == "RAW_T") {
    long raw = lerRawEstavel(balanca_torque);
    Serial.print("RAW_T:");
    Serial.println(raw);
    Serial.flush();

  } else if (cmd == "RESET") {
    Serial.println("STATUS:Reiniciando ESP32...");
    Serial.flush();
    delay(100);
    ESP.restart();

  } else if (cmd.length() > 0) {
    Serial.print("STATUS:Comando desconhecido: ");
    Serial.println(cmd);
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  balanca.begin(PINO_DT, PINO_SCK);
  balanca_torque.begin(PINO_DT_TORQUE, PINO_SCK_TORQUE);

  balanca.set_gain(128);
  balanca_torque.set_gain(128);

  Serial.println("STATUS:Iniciando balanca...");
  delay(3000);

  balanca.set_scale(fator_calibracao);
  zerarSensor(balanca, cal_empuxo);

  balanca_torque.set_scale(fator_calibracao_torque);
  zerarSensor(balanca_torque, cal_torque);

  Serial.println("STATUS:Tara concluida.");
  Serial.print("STATUS:Braco torque: ");
  Serial.print(BRACO_TORQUE_MM, 1);
  Serial.println(" mm");
  Serial.println("HEADER:tempo_s,empuxo_N,torque_N_m");

  tempo_inicio = millis();
}

// ============================================================
void loop() {
  // Verifica comandos recebidos do Python
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processarComando(cmd);
  }

  unsigned long agora = millis();
  if (agora - ultimo_envio < INTERVALO_ENVIO_MS) {
    return;
  }
  ultimo_envio = agora;

  float empuxo_n       = lerForcaCalibrada(balanca, cal_empuxo);
  float forca_torque_n = lerForcaCalibrada(balanca_torque, cal_torque);
  float torque_nm      = forca_torque_n * BRACO_TORQUE_M;
  float tempo_s        = (agora - tempo_inicio) / 1000.0;

  // Saída CSV para o Python
  Serial.print("DATA:");
  Serial.print(tempo_s, 3);
  Serial.print(",");
  Serial.print(empuxo_n, 4);
  Serial.print(",");
  Serial.println(torque_nm, 5);
}
