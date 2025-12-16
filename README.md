# STM32 Temperature Control with FreeRTOS

Este repositório contém o desenvolvimento de um **sistema embarcado de controle de temperatura**
implementado em um microcontrolador **STM32F401RE**, utilizando o **FreeRTOS** para gerenciamento
de tarefas em tempo real.

O projeto foi desenvolvido como **trabalho acadêmico** no curso de **Engenharia de Controle e Automação**
e envolve desde a modelagem do sistema até a implementação prática em hardware.

---

##  Visão Geral do Projeto

O sistema realiza:

- Leitura de temperatura via **sensor LM35**
- Aquisição de dados através do **ADC do STM32**
- Filtragem do sinal usando **filtro de média móvel**
- Controle em malha fechada utilizando **controlador PID**
- Atuação por meio de **PWM** aplicado a um atuador
- Execução concorrente das tarefas utilizando **FreeRTOS**

---

##  Hardware Utilizado

- Microcontrolador: **STM32F401RE (Nucleo)**
- Sensor de temperatura: **LM35**
- Atuador: controle via **PWM (TIM3)**
- Interface de comunicação: **UART (USART2)**

---

##  Arquitetura de Software

O sistema foi estruturado utilizando múltiplas *tasks* do FreeRTOS:

- **Task de Leitura do ADC**
  - Realiza a leitura periódica do sensor LM35
  - Converte a leitura para temperatura em °C

- **Task de Filtro (Média Móvel)**
  - Aplica um filtro de média móvel com N = 100 amostras
  - Reduz ruído de medição
  - Envia o valor filtrado para o controlador

- **Task de Controle PID**
  - Implementa um controlador PID discreto
  - Calcula o sinal de controle com base no erro
  - Aplica saturação e anti-windup
  - Atualiza o duty cycle do PWM

A comunicação entre as tarefas é feita por meio de **filas (queues)** do FreeRTOS.

---

##  Filtro de Média Móvel

O filtro de média móvel foi adotado para reduzir ruídos provenientes do sensor e do ADC.
O valor filtrado é obtido a partir da média das últimas **100 amostras**, garantindo maior
estabilidade na ação de controle.

---

##  Controle PID

O controlador PID foi implementado, considerando:

- Termo proporcional (P)
- Termo integral (I) com **anti-windup**
- Termo derivativo (D)
- Saturação do sinal de controle

O sinal de saída do controlador atua diretamente no **PWM** responsável pelo atuador térmico.

---

## 📁 Estrutura do Repositório

```text
.
├── Core/
│   ├── Inc/
│   └── Src/
│       └── main.c
├── Drivers/
├── docs/
│   └── artigo_projeto.pdf
├── STM32CubeMX/
├── README.md
