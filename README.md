# Dinamômetro para Conjuntos Moto-Propulsores de Aeronaves RC

Repositório do software desenvolvido como parte do Trabalho de Conclusão de Curso em Engenharia Mecânica — FEI.

O sistema realiza a medição simultânea de **empuxo** e **torque** de conjuntos moto-propulsores (motor brushless + hélice) para aeronaves de asas fixas controladas por rádio, com aplicação direta na seleção de configurações para a competição SAE Brasil AeroDesign.

---

## Arquivos

| Arquivo | Descrição |
|---|---|
| `DINAMOMETRO_esp32.ino` | Firmware do microcontrolador ESP32 — leitura dos HX711, calibração, cálculo de torque e comunicação serial |
| `dinamometro_interface.py` | Interface gráfica em Python — visualização em tempo real, calibração, gravação e exportação dos dados |

---

## Hardware necessário

- ESP32 (qualquer variante com pelo menos 4 GPIOs disponíveis)
- 2× Célula de carga PE-130 20 kgf
- 2× Módulo HX711
- Fonte de alimentação 5 V para o circuito de aquisição
- ESC e motor brushless sob teste
- Cabo USB para comunicação serial entre ESP32 e computador

**Pinagem do ESP32:**

| Canal | Pino DT (dados) | Pino SCK (clock) |
|---|---|---|
| Empuxo | GPIO 16 | GPIO 17 |
| Torque | GPIO 4 | GPIO 5 |

---

## Dependências

### Firmware (Arduino IDE)

- Biblioteca **HX711** — instale pela Library Manager do Arduino IDE  
  (buscar por `HX711 by Bogdan Necula`)

### Interface gráfica (Python ≥ 3.8)

```bash
pip install pyserial matplotlib
```

O módulo `tkinter` já acompanha a instalação padrão do Python no Windows. Se necessário instalar separadamente:

```bash
sudo apt install python3-tk
```

---

## Como usar

### 1. Firmware

1. Abra `DINAMOMETRO_esp32.ino` no Arduino IDE.
2. Se necessário, ajuste o valor de `BRACO_TORQUE_MM` para a distância medida na sua montagem entre o eixo do motor e o ponto de aplicação de força na célula de carga de torque.
3. Selecione a placa **ESP32 Dev Module** e a porta correta.
4. Faça o upload. O ESP32 iniciará a transmissão automaticamente ao ser conectado ao computador.

### 2. Interface gráfica

```bash
python dinamometro_interface.py
```

1. Selecione a porta serial do ESP32 e clique em **Conectar**.
2. Na aba **Calibração**, execute a tara e o procedimento de calibração por três pontos para cada canal (empuxo e torque) antes de iniciar os ensaios.
3. Informe a densidade do ar local (kg/m³) no campo correspondente — o fator de correção é aplicado automaticamente a todas as leituras exibidas e gravadas.
4. Na aba **Exportar**, preencha as informações do ensaio (motor, hélice, local, temperatura), clique em **Start** para iniciar a gravação e em **Stop** ao final. Use **Salvar CSV + PNG** para exportar os resultados.

---

## Protocolo serial

A comunicação entre a interface e o ESP32 opera a **115 200 bps**. Os comandos enviados pelo computador e as respostas do ESP32 são todos em texto ASCII terminado em `\n`.

**Comandos enviados pela interface:**

| Comando | Ação |
|---|---|
| `TARE` | Zera ambas as células |
| `TARE_E` | Zera apenas o canal de empuxo |
| `TARE_T` | Zera apenas o canal de torque |
| `RAW_E` | Solicita leitura bruta do canal de empuxo (para calibração) |
| `RAW_T` | Solicita leitura bruta do canal de torque (para calibração) |
| `CAL3_E:<params>` | Aplica calibração por três pontos no canal de empuxo |
| `CAL3_T:<params>` | Aplica calibração por três pontos no canal de torque |
| `RESET` | Reinicia o ESP32 |

**Dados transmitidos pelo ESP32:**

```
DATA:<tempo_s>,<empuxo_N>,<torque_N_m>
STATUS:<mensagem>
RAW_E:<valor>
RAW_T:<valor>
```

---

## Calibração

O sistema utiliza calibração por **três pontos com interpolação linear por segmentos**, executada pela interface sem necessidade de alterar o firmware.

**Procedimento:**
1. Com a célula descarregada, colete o RAW vazio.
2. Aplique três massas de referência conhecidas sequencialmente (os valores padrão sugeridos são 1 kgf, 2 kgf e 3 kgf, ou seja, 9,807 N, 19,614 N e 29,421 N).
3. Para cada massa, insira o valor de força e clique em **Coletar**.
4. Clique em **Aplicar calibração 3 pontos**.

Os parâmetros ficam armazenados no ESP32 durante toda a sessão e devem ser reestabelecidos após qualquer reinicialização.

---

## Correção por densidade do ar

Todos os valores de empuxo exibidos e gravados são corrigidos pela razão entre a densidade do ar local e a densidade padrão ao nível do mar (1,225 kg/m³):

```
T_corrigido = (ρ_local / 1,225) × T_medido
```

Informe a densidade local no campo da interface antes de iniciar o ensaio. O fator de correção resultante é exibido em tempo real.

---

## Formato de exportação

Ao exportar, dois arquivos são gerados simultaneamente:

- **`ensaio_YYYYMMDD_HHMMSS.csv`** — colunas: `tempo_s`, `empuxo_N`, `torque_N_m`
- **`ensaio_YYYYMMDD_HHMMSS_graficos.png`** — gráficos de empuxo e torque em função do tempo, com identificação do ensaio

O arquivo CSV contém apenas os pontos capturados entre os acionamentos de **Start** e **Stop**, com o tempo referenciado ao instante do Start.

## Autor

Fernando Borge de Souza Barbosa

Lucas Silva Perez Siqueira

Thiago Borja Gracindo

João Guilherme Marun Dias
