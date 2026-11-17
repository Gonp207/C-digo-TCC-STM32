Analisador de Qualidade de Energia Trifásico de Baixo Custo (STM32F407VGT6)

> Trabalho de Conclusão de Curso (TCC) - Engenharia Elétrica
>
> Instituição: Centro Universitário Ingá (UNINGÁ) - 2026
>
> Título: "Análise da Distorção Harmônica de um Inversor Fotovoltaico em Rede de Baixa Tensão por Meio de um Analisador de Energia de Baixo Custo"

---

 1. Visão Geral

Este repositório contém o firmware embarcado desenvolvido em linguagem C para um analisador de energia trifásico de baixo custo baseado no microcontrolador STM32F407VGT6 (placa *STM32F4 Discovery*). O instrumento foi projetado segundo os critérios de amostragem e janelamento da norma IEC 61000-4-7 e aplicado na medição e avaliação da distorção harmônica da corrente injetada por um inversor fotovoltaico conectado à rede (*on-grid*), confrontando os resultados com os limites estabelecidos pela ABNT NBR 16149 e pelo PRODIST (Módulo 8 - ANEEL).

Principais Características

* Aquisição simultânea de 6 canais (3 tensões e 3 correntes) utilizando três ADCs de 12 bits operando em modo triplo simultâneo.
* Sobreamostragem de 8x (~61,44 kHz) com filtragem digital FIR passa-baixa de 181 coeficientes e decimação para 128 amostras por ciclo (7.680 Hz em 60 Hz).
* Sincronismo dinâmico com a rede via algoritmo SOGI-PLL (*Second-Order Generalized Integrator - Phase-Locked Loop*), ajustando em tempo real a base de tempo do temporizador `TIM8` para manter o erro de sincronismo menor ou igual a 0,03%.
* Análise espectral até a 50a ordem harmônica por meio de soma coerente de 12 ciclos (200 ms), FFT de 128 pontos, agregação em 5 janelas consecutivas (IEC 61000-4-30) e compensação matemática da atenuação do filtro RC de entrada.
* Registro autônomo em cartão SD via interface USB OTG FS (conector CN5) operando como *USB Host Mass Storage Class* com sistema de arquivos FatFs (`BRUTO.CSV` e `HARMON.CSV`).

---

 2. Arquitetura de Hardware

| Subsistema | Componente / Especificação | Função no Sistema |
| :--- | :--- | :--- |
| Microcontrolador | STM32F407VGT6 (ARM Cortex-M4F @ 168 MHz, 1 MB Flash, 196 KB RAM) | Aquisição tripla simultânea, filtragem FIR, SOGI-PLL, FFT e controle USB Host. |
| Sensores de Tensão (3x) | ZMPT101B + condicionador LM358 (VCC = 5 V, offset ~2,39 V) | Medição de tensões fase-neutro (127 V nominais). Ganho nominal de projeto GV = 515 V/V (resolução de 0,377 V). |
| Sensores de Corrente (3x) | QNHCK2-16 (Efeito Hall, malha aberta, 150 A nominais, 3,3 V, offset 1,65 V) | Medição de correntes CA/CC. Sensibilidade de 0,625 V na corrente nominal, ganho GI = 240 A/V (resolução de 176 mA). |
| Filtro Anti-Aliasing | 6 filtros passa-baixa RC de 1a ordem (R = 1,6 kOhm, C = 10 nF) | Frequência de corte fc ~ 9.947 Hz; fornece carga ao capacitor interno do ADC e atenua componentes de alta frequência. |
| Armazenamento | Leitor de cartão SD via USB OTG FS (conector CN5) | Gravação contínua dos indicadores no domínio do tempo e espectro harmônico em formato CSV. |
| Interface Local | LEDs *onboard* da placa STM32F4 Discovery | Indicação visual de estados de operação, travamento da PLL e diagnóstico de falhas de aquisição/escrita. |

---

 3. Cadeia de Aquisição e Processamento Digital de Sinais

```text
[Rede 127/220 V, 60 Hz]
         |
         v
[Sensores: 3x ZMPT101B (V) e 3x QNHCK2-16 (I)]
         |
         v
[Filtro RC Anti-Aliasing (fc ~ 9,95 kHz)]
         |
         v
[ADC1, ADC2, ADC3 - 12 bits, Triplo Simultaneo]
  ^
  | (Disparo via TIM8 a 8 x 128 x f ~ 61,44 kHz)
         |
         v
[DMA Circular - 2 metades de 768 amostras]
         |
         v
[Filtro FIR Digital (181 coef., 0 a 3 kHz) + Decimacao /8]
         |
         +---> [SOGI-PLL (Fase A)] -> Reprograma TIM8 (erro <= 0,03%)
         |
         v
[Calculos Temporais e Espectrais (5 janelas de 200 ms)]
  - Vrms, Irms, P, S, FP e Fator de Crista
  - FFT de 128 pontos + Compensacao do filtro RC (2a a 50a)
         |
         v
[USB Host MSC + FatFs -> BRUTO.CSV e HARMON.CSV]
```

Parâmetros Normativos e de Amostragem (IEC 61000-4-7)

| Parâmetro | Valor Adotado | Observação Técnica |
| :--- | :--- | :--- |
| Frequência fundamental (f1) | 60 Hz | Monitorada continuamente pelo algoritmo SOGI-PLL. |
| Taxa de sobreamostragem (fADC) | 61.440 Hz (8 x 128 x f1) | Clock do ADC em 21 MHz; 144 ciclos de amostragem (7,43 us por conversão). |
| Filtro digital FIR | 181 coeficientes | Banda passante de 0 a 3.000 Hz (ondulação de 0,08%); rejeição de 97 dB a partir de 4.680 Hz. |
| Taxa após decimação (fs) | 7.680 Hz (128 amostras/ciclo) | Armazenamento interno em escala de 14 bits. |
| Janela de observação (Tj) | 200 ms (12 ciclos) | 1.536 amostras por canal em cada janela; resolução espectral de 5 Hz. |
| Bloco de captura completo | 5 janelas (60 ciclos ~ 1 s) | Total de 46.080 amostras nos 6 canais por captura (~1,2 s entre registros). |
| Faixa harmônica registrada | 2a a 50a ordem (120 Hz a 3.000 Hz) | Agregação quadrática nas 5 janelas conforme IEC 61000-4-30. |

---

 4. Estrutura do Repositório

O projeto foi configurado no STM32CubeMX (v6.18.1) e desenvolvido no STM32CubeIDE (v2.2):

```text
+-- Core/
|   +-- Inc/
|   |   +-- adc.h                   # Definicoes e prototipos dos ADCs 1, 2 e 3
|   |   +-- dma.h                   # Configuracoes dos fluxos DMA
|   |   +-- gpio.h                  # Mapeamento de pinos e LEDs de diagnostico
|   |   +-- main.h                  # Constantes globais e parametros do sistema
|   |   +-- stm32f4xx_hal_conf.h    # Habilitacao dos modulos da biblioteca HAL
|   |   +-- stm32f4xx_it.h          # Cabecalhos das rotinas de interrupcao
|   |   \-- tim.h                   # Prototipos do temporizador TIM8 (trigger ADC)
|   \-- Src/
|       +-- adc.c                   # Configuracao do modo triplo simultaneo e canais
|       +-- dma.c                   # Inicializacao do DMA circular (ping-pong)
|       +-- gpio.c                  # Inicializacao das portas digitais
|       +-- main.c                  # Loop principal, FIR, SOGI-PLL, FFT e FatFs
|       +-- stm32f4xx_hal_msp.c     # Inicializacoes de baixo nivel (MSP)
|       +-- stm32f4xx_it.c          # Tratamento de interrupcoes (DMA e USB)
|       +-- syscalls.c              # Chamadas de sistema da biblioteca padrao C
|       +-- sysmem.c                # Gerenciamento de memoria heap
|       +-- system_stm32f4xx.c      # Arvore de clock (HSE 8 MHz -> PLL 168 MHz)
|       \-- tim.c                   # Configuracao da base de tempo do TIM8
+-- Drivers/                        # Bibliotecas CMSIS e STM32F4xx HAL Driver
+-- Middlewares/                    # Pilha USB Host Library e sistema de arquivos FatFs
\-- *.ioc                           # Arquivo de configuracao de hardware (CubeMX)
```

---

 5. Formato dos Arquivos de Saída (Cartão SD)

A cada ciclo de operação, o firmware grava dois arquivos CSV na raiz do dispositivo USB/SD:

1. `BRUTO.CSV` *(1 linha por captura, a cada ~1,2 s)*:
   * *Timestamp* relativo desde a energização;
   * Valores eficazes (`Vrms`, `Irms`), máximo, mínimo e médio retificado por fase;
   * Fator de crista, potência ativa (`P`), potência aparente (`S`) e fator de potência (`FP`).

2. `HARMON.CSV` *(1 linha a cada intervalo agregado de 5 s)*:
   * Valores médio, mínimo e máximo das capturas compreendidas no intervalo de 5 s;
   * Por canal: amplitude eficaz da componente fundamental, Distorção Harmônica Total (DHT) e magnitude de cada ordem harmônica da 2a à 50a.

---

 6. Validação e Principais Resultados

 6.1. Validação por Simulação (PSIM 2026 + Bloco C)

As rotinas em linguagem C do firmware foram embarcadas em um bloco C no PSIM 2026 junto ao modelo completo do inversor fotovoltaico (ponte completa IGBT, PWM unipolar de 20 kHz, filtro LCL e controle PR em 60 Hz) e da cadeia de condicionamento:

* Erro na potência ativa: menor que 0,03% (0,024% em rede senoidal; 0,026% em rede com distorção de fundo);
* Erro na corrente eficaz (Irms): menor que 0,01%;
* Diferença na DHT de corrente: até 0,013 ponto percentual em relação aos valores de referência do simulador;
* Ordens harmônicas (2a a 13a): diferença máxima de 3,5 mA, dentro dos limites da Classe I da IEC 61000-4-7.

 6.2. Ensaio Experimental em Campo

O protótipo foi ensaiado na saída de um inversor *on-grid* SAJ R5-4K-S2-15 (4 kW, 220 V, 18,2 A nominais) conectado entre duas fases de uma rede 127/220 V (60 Hz):

* Corrente eficaz medida: 17,97 A (Fase A) e 18,25 A (Fase B), com erro máximo de 1,3% frente ao multímetro de referência (*Fluke 107*);
* DHT de corrente medida: 3,04% (Fase A) e 3,02% (Fase B), atendendo ao limite de menor que 5,0% exigido pela ABNT NBR 16149;
* Comportamento espectral: predominância da 3a (1,71%) e 5a (1,74%) ordens harmônicas, com todas as componentes individuais dentro dos limites normativos.

---

 7. Como Compilar e Executar

1. Pré-requisitos:
   * STM32CubeIDE (versão 2.2 ou superior);
   * STM32CubeMX (versão 6.18.1 ou superior, caso deseje alterar pinagem ou árvore de clock);
   * Placa STM32F4 Discovery (`STM32F407VGT6`) e cabo Mini-USB (programação/depuração via ST-LINK/V2 integrado).

2. Importação do Projeto:
   * Clone este repositório:

     ```bash
     git clone https://github.com/Gonp207/C-digo-TCC-STM32.git
     ```

   * No STM32CubeIDE, acesse `File > Open Projects from File System...`, selecione a pasta clonada e clique em Finish.

3. Compilação e Gravação:
   * Compile o projeto (`Project > Build All` ou `Ctrl + B`).
   * Conecte um pendrive ou leitor de cartão SD formatado em FAT32 à porta USB OTG FS (conector CN5) da placa Discovery.
   * Inicie a gravação no microcontrolador (`Run > Run` ou `F11`).

---

 8. Limitações Conhecidas e Trabalhos Futuros

* Calibração individual dos canais de tensão: A utilização de um ganho fixo (515 V/V) sem calibração individual dos *trimpots* dos módulos ZMPT101B ocasionou desvio de -10,2% na tensão eficaz de linha durante o ensaio prático (sem afetar os valores percentuais de DHT). Recomenda-se rotina de calibração por canal.
* Deslocamento de fase experimental: Observou-se defasagem de aproximadamente 31 graus entre tensão de linha e corrente no ensaio prático (ausente na simulação com o mesmo firmware), restringindo a validação experimental de potência ativa e fator de potência.
* Resolução em baixas correntes: O sensor de 150 A oferece degrau de quantização de 176 mA, limitando a precisão direta em harmônicas de ordem elevada de inversores de menor porte.
* Melhorias sugeridas: Substituição do LM358 por amplificador *rail-to-rail* alimentado em 3,0 V, inclusão de módulo RTC para estampa de tempo absoluta e aquisições contínuas sem intervalo entre capturas.

---

 9. Autores e Orientação

Autores (Discentes - Engenharia Elétrica, UNINGÁ):

* Leonardo de Souza Gonçalves - `gonp207@outlook.com`
* Luiz Felipe Sanches Santana - `ra191159-24@uninga.edu.br`
* Pedro Henrique Machado de Oliveira - `pedroomachado95@gmail.com`

Orientação (Docentes - Engenharia Elétrica, UNINGÁ):

* Prof. Adelan de P. N. Martens - `prof.adelanmartens@uninga.edu.br`
* Prof.a Lilian Felipe da Silva Tupan - `prof.liliantupan@uninga.edu.br`