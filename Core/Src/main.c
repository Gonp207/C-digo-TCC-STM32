/* USER CODE BEGIN Header */
/**
  *****************************************************************************
  * @file    main.c
  * @brief   Analisador de harmonicos trifasico 220 V / 60 Hz - IEC 61000
  *          STM32F407VGT6-DISCOVERY (MB997) - STM32CubeIDE
  ******************************************************************************
  *
  * FLUXO (medicao continua: uma captura de 60 ciclos a cada ~1,1 s)
  *  1. Aquisicao : o TIM3 dispara o ADC1 (mestre) + ADC2 + ADC3 em modo triplo
  *                 simultaneo. Rank 1 = PA1/PA2/PA3 (IA/IB/IC) ao mesmo tempo;
  *                 rank 2 = PC1/PC2/PC3 (VA/VB/VC) ao mesmo tempo. O DMA (modo 1)
  *                 grava 6 amostras por gatilho: 7680 por canal = 46080 no total.
  *  2. SOGI-PLL  : mede a frequencia na tensao da fase A e reprograma o TIM3
  *                 para fs = 128*f -> 128 amostras por ciclo (janela sincronizada;
  *                 a IEC 61000-4-7 admite erro de +-0,03 %, senao recaptura).
  *  3. FFT       : 60 ciclos = 5 janelas de 12 ciclos (IEC 61000-4-7 @ 60 Hz).
  *                 Os 12 ciclos de cada janela sao somados (media coerente) e
  *                 passam por uma FFT radix-2 de 128 pontos -> ordens 0..64.
  *                 Isso e matematicamente IGUAL a DFT de 12 ciclos da norma
  *                 avaliada nas frequencias harmonicas. V e I de cada fase vao
  *                 juntos numa unica FFT complexa (z = v + j*i).
  *  4. Agregacao : raiz da media dos quadrados das 5 janelas (IEC 61000-4-30).
  *  5. Registro  : no pendrive / leitor SD do CN5.
  *     BRUTO.CSV  - uma linha por captura: valores brutos, fundamentais,
  *                  angulos de fase (relativos a VA), potencias, tensoes de
  *                  linha, corrente de neutro e desequilibrio.
  *     HARMON.CSV - uma linha por intervalo de registro (padrao 5000 ms,
  *                  ver INTERVALO_REGISTRO_MS / Registro_DefinirIntervalo),
  *                  no mesmo leiaute de colunas do analisador P55
  *                  (medio/min/max de todas as capturas do intervalo).
  *                  Harmonicas h2..h50 em valor RMS (V ou A).
  *     Data/Hora : relogio proprio que comeca em 00:00:00 (dia 1) na primeira
  *                  captura apos ligar, como se o equipamento fosse ligado a
  *                  meia-noite. Data = numero do dia (1, 2, 3...).
  *
  * TABELA UNICA: TABELA_SENO[129] = sen(2*pi*k/128), k = 0..128. O cosseno e o
  *  proprio seno defasado 90 graus (indice + 32). FFT, PLL e correcoes de fase
  *  usam somente ela; atan2f e usada apenas para o angulo gravado no BRUTO.CSV.
  *
  * CONFIGURACAO ESPERADA NO CubeMX (o codigo confere e acusa erro nos LEDs)
  *  (o projetomx.ioc que acompanha este arquivo ja esta assim)
  *  ADC1 : Triple regular simultaneous mode only | DMA access mode 1
  *         Clock prescaler PCLK2/4 (21 MHz), Scan ON, Continuous OFF,
  *         DMA Continuous Requests ON
  *         External trigger: Timer 3 Trigger Out event, borda de subida
  *         2 conversoes: Rank1 = IN1 (PA1), Rank2 = IN11 (PC1)
  *  ADC2 : 2 conversoes: IN2 (PA2), IN12 (PC2)
  *  ADC3 : 2 conversoes: IN3 (PA3), IN13 (PC3)
  *         Sampling time 480 ciclos em todos (47 us por gatilho, periodo 130 us)
  *         -> se inverter os ranks (tensao primeiro) o codigo se adapta sozinho
  *  DMA  : ADC1 -> DMA2 Stream0, Normal, Half Word / Half Word
  *  TIM3 : clock interno, TRGO = Update Event (PSC e ARR sao definidos aqui)
  *  USB  : USB_OTG_FS Host_Only (PA11/PA12 = conector CN5), classe MSC,
  *         Drive_VBUS_FS = PC0; FATFS: USB Disk. (O OTG_HS em PB14/PB15 NAO
  *         esta ligado ao CN5 da Discovery.)
  *  Project Manager: Minimum Heap Size 0x1000, Minimum Stack Size 0x1000
  *
  * LEDs (GPIOD)
  *  Laranja piscando ............... aguardando pendrive / leitor de cartao
  *  Verde>Laranja>Vermelho>Azul .... barra de progresso da aquisicao (25..100 %)
  *  Os 4 acesos .................... processando (SOGI-PLL + FFT)
  *  Azul ........................... gravando no cartao
  *  Vermelho pisca 3x .............. falha de gravacao (remonta o cartao)
  *  Vermelho+Laranja piscam 3x ..... falha de aquisicao (timeout/overrun)
  *  Codigo binario fixo ............ erro de configuracao (ver ERRO_CFG_*)
  *  Os 4 piscando rapido ........... Error_Handler (falha de inicializacao HAL)
  *
  * SOBREAMOSTRAGEM x8 (SOBREAMOSTRAGEM = 1, ver CONFIGURACAO)
  *  O filtro RC da placa (corte em 9.947 Hz) fica acima da frequencia de
  *  Nyquist de 3.840 Hz e nao evita o recobrimento. Com a opcao ligada:
  *   - o ADC amostra a 8 x 128 x f (61,4 kHz por canal), disparado pelo TIM8
  *     (168 MHz: erro maximo de sincronismo de 0,018 %, contra 0,037 % que o
  *     TIM3 de 84 MHz daria nessa taxa), com 144 ciclos de amostragem;
  *   - o DMA trabalha em modo circular com duas metades de 16 x 8 quadros;
  *     a cada metade, a interrupcao passa cada canal por um filtro FIR
  *     passa-baixa de 181 coeficientes (0 a 3.000 Hz = ordem 50, ondulacao
  *     de 0,08 %; rejeicao de 97 dB a partir de 4.680 Hz = ordem 78) e
  *     guarda 1 de cada 8 amostras, multiplicada por 4 (14 bits), no adc_buf;
  *   - o adc_buf continua com 7680 amostras por canal (128 por ciclo) e todo
  *     o processamento (PLL, FFT, potencias, registro) fica igual.
  *  Nada muda no CubeMX: gatilho, tempo de amostragem e modo do DMA sao
  *  reprogramados em tempo de execucao (OS_ConfigurarADC). Com
  *  SOBREAMOSTRAGEM = 0 o firmware volta exatamente ao comportamento anterior.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Estatisticas brutas de um canal, em unidades fisicas (V ou A) */
typedef struct
{
  float   rms;        /* valor eficaz (true RMS da componente CA)            */
  float   rms_min;    /* menor RMS entre as 5 janelas de 12 ciclos           */
  float   rms_max;    /* maior RMS entre as 5 janelas de 12 ciclos           */
  float   amp_media;  /* amplitude media: media de (max-min)/2 de cada ciclo */
  float   max;        /* valor instantaneo maximo                            */
  float   min;        /* valor instantaneo minimo                            */
  float   retif;      /* valor medio retificado: media de |x|                */
  float   fc;         /* fator de crista: pico / RMS                         */
  float   offset_v;   /* media da tensao no pino do ADC (V)                  */
  uint8_t saturou;    /* 1 = alguma amostra encostou em 0 ou 4095            */
} Estat_t;

/* Potencias por fase */
typedef struct
{
  float p;            /* potencia ativa (W) - media de v*i                   */
  float s;            /* potencia aparente Vrms*Irms (VA)                    */
  float fp;           /* fator de potencia P/S                               */
  float q1;           /* potencia reativa da fundamental (var)               */
  float fpd;          /* fator de deslocamento cos(phi1)                     */
  float p1;           /* potencia ativa da fundamental (W)                   */
  float n;            /* potencia nao ativa sqrt(S^2-P^2), sinal de Q1 (var) */
  float phi1;         /* angulo V1 - I1 da fundamental (graus; + = indutivo) */
} Potencia_t;

/* Indicadores harmonicos e verificacao de norma de um canal */
typedef struct
{
  float   h1;         /* RMS da fundamental (V ou A)                         */
  float   thd40;      /* THD (%) com h = 2..40                               */
  float   thd63;      /* THD (%) com h = 2..63                               */
  float   pwhd;       /* PWHD (%) com h = 14..40                             */
  float   hrms40;     /* RMS do conteudo harmonico h = 2..40 (V ou A)        */
  float   pior_pct;   /* maior (valor/limite)*100 entre as ordens            */
  uint8_t pior_h;     /* ordem do pior caso                                  */
  uint8_t excede;     /* quantas ordens passaram do limite                   */
  uint8_t conforme;   /* 1 = atende a norma aplicada                         */
  uint8_t valido;     /* 1 = fundamental acima do minimo                     */
} ResultIEC_t;

/* SOGI-PLL monofasico. Fase em "passos de tabela": 0..128 = 0..2*pi */
typedef struct
{
  float u1, u2;       /* entradas anteriores                                 */
  float a1, a2;       /* saidas v'  (alfa) anteriores                        */
  float b1, b2;       /* saidas qv' (beta) anteriores                        */
  float fase;         /* fase estimada [0,128)                               */
  float f;            /* frequencia estimada (Hz)                            */
  float integ;        /* integrador do PI (Hz)                               */
} SogiPll_t;

typedef enum
{
  EST_AGUARDA_MIDIA = 0,
  EST_INICIA_AQUISICAO,
  EST_AQUISICAO,
  EST_PROCESSA,
  EST_GRAVA
} Estado_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ============================= CONFIGURACAO ============================== */

/* --- INTERVALO ENTRE REGISTROS DO HARMON.CSV  >>> ALTERE AQUI <<< ----------
 * Tempo entre duas linhas do HARMON.CSV, em milissegundos (padrao 5000 ms).
 * Durante o intervalo o analisador mede sem parar (uma captura de 60 ciclos a
 * cada ~1,1 s) e, ao fim dele, grava uma linha com o valor medio, minimo e
 * maximo de todas as capturas, como o P55. Em tempo de execucao o valor pode
 * ser trocado com Registro_DefinirIntervalo(ms) (chamada no inicio do main).
 *   5000u = 5 s (padrao)   60000u = 1 min   300000u = 5 min   600000u = 10 min */
#define INTERVALO_REGISTRO_MS 5000u
#define INTERVALO_MIN_MS      1000u        /* limites aceitos pela funcao      */
#define INTERVALO_MAX_MS      86400000u    /* 24 h                             */

/* --- Rede e amostragem ---------------------------------------------------- */
#define F_NOMINAL_HZ          60.0f
#define N_FFT                 128u     /* amostras por ciclo = pontos da FFT   */
#define H_MAX                 64u      /* ordens 0..64 (64 = Nyquist)          */
#define CICLOS_JANELA         12u      /* IEC 61000-4-7: 12 ciclos em 60 Hz    */
#define JANELAS               5u       /* 5 x 12 = 60 ciclos por captura       */
#define N_CANAIS              6u
#define AMOSTRAS_CANAL        (N_FFT * CICLOS_JANELA * JANELAS)    /* 7680    */
#define ADC_BUF_LEN           (AMOSTRAS_CANAL * N_CANAIS)          /* 46080   */

/* --- Sobreamostragem com filtro anti-aliasing digital  >>> ALTERE AQUI <<< ---
 * 1 = ADC a 8 x 128 x f, filtro FIR passa-baixa e decimacao por 8 (ver o
 *     cabecalho). 0 = aquisicao original (TIM3, 480 ciclos, sem filtro).      */
#define SOBREAMOSTRAGEM       1

#if SOBREAMOSTRAGEM
#define OS_FATOR              8u       /* fator de sobreamostragem             */
#define FIR_N                 181u     /* coeficientes do FIR (impar, simetrico)*/
#define FIR_MEIO              ((FIR_N - 1u) / 2u)                 /* 90      */
#define OS_ESCALA             4.0f     /* saida do FIR x4 -> 14 bits no adc_buf */
#define OS_BLOCO              16u      /* amostras decimadas por meia-transf.:
                                          interrupcao a cada 2,1 ms, < 1 ms de
                                          calculo (nao atrasa o SysTick)      */
#define OS_MEIA_LEN           (OS_BLOCO * OS_FATOR * N_CANAIS)    /* 768     */
#define OS_DMA_LEN            (2u * OS_MEIA_LEN)                  /* 1536    */
#define OS_SMP                ADC_SAMPLETIME_144CYCLES  /* 2x(144+12) ciclos = 14,9 us
                                                           < 16,3 us entre gatilhos */
/* Linhas de atraso do FIR na CCM RAM (64 KB em 0x10000000, nao usada pelo
 * projeto e inacessivel ao DMA, mas a CPU acessa sem espera): 8,7 KB que nao
 * pesam na SRAM. Se o seu .ld colocar algo na CCM, mude para 0 (vai para a
 * SRAM comum).                                                              */
#define OS_LINHA_NA_CCM       1
#endif

/* --- Front-end analogico -------------------------------------------------- */
#define VDDA_V                3.00f    /* Discovery MB997: VDD = VDDA = 3,0 V  */
#if SOBREAMOSTRAGEM
#define ADC_FUNDO_ESCALA      (4096.0f * OS_ESCALA)   /* contagens no adc_buf  */
#else
#define ADC_FUNDO_ESCALA      4096.0f
#endif

/* ZMPT101B: volts na rede por volt no pino do ADC.  >>> CALIBRAR <<<
 * Ganho_novo = Ganho_atual * V_multimetro / V_rms registrado no BRUTO.CSV
 * (valor negativo inverte a polaridade do sensor)                           */
/* Calibrado em 15/11/2026 com 234 V eficazes medidos com multimetro entre
 * as fases (ensaio do inversor de 14/11). Com 515 nos dois canais, a coleta
 * de 11,6 min registrou VA = 112,55 V e VB = 125,84 V (media); os ganhos
 * abaixo deixam as duas fases iguais e o maior VAB da coleta em 234 V
 * (tensao de fase equivalente de 132,2 V):
 *   VA: 515 * 132,2 / 112,55 = 605      VB: 515 * 132,2 / 125,84 = 541
 * Valem para a posicao dos trimpots DAQUELA coleta (eles foram mexidos depois,
 * na coleta 4). Antes de uma nova coleta: voltar os trimpots a esse ganho
 * ou calibrar de novo, e depois NAO mexer mais nos trimpots.               */
#define GANHO_VA              605.0f   /* modulo em 5 V                      */
#define GANHO_VB              541.0f
#define GANHO_VC              250.0f   /* fase C sem modulo ligado          */

/* QNHCK2-16: saida 1,65 V +- 0,625 V em +-I_PN (alimentacao 3,3 V)            */
#define I_PN_A                150.0f   /* corrente nominal do seu modelo       */
#define I_SAIDA_PN_V          0.625f   /* excursao da saida em I_PN            */
#define I_ESPIRAS             1.0f     /* voltas do condutor dentro do nucleo  */
#define GANHO_IA              (I_PN_A / I_SAIDA_PN_V / I_ESPIRAS)  /* A por V  */
#define GANHO_IB              GANHO_IA
#define GANHO_IC              GANHO_IA

/* Atraso de fase proprio de CADA sensor em 60 Hz (graus; + = atrasa,
 * - = adianta). Um valor por canal, porque cada modulo ZMPT101B tem o seu
 * trimpot e a defasagem do modulo muda com a posicao dele.
 * Como medir (com o trimpot na posicao de uso):
 *  - carga puramente resistiva entre as fases A e B (FP = 1): a corrente tem
 *    de sair em fase com VAB; ou
 *  - osciloscopio com um canal na tensao da rede (ponta isolada/diferencial)
 *    e outro na saida do modulo: atraso da saida em relacao a rede.
 * NAO usar o ensaio do inversor para achar este valor (seria supor o FP dele).
 * Onde entram: angulos do BRUTO.CSV, P1, Q1, FPD, phi1 e, pela fundamental,
 * a potencia ativa P (e com ela FP, N, totais, energia e demanda).
 * Com todos em 0 o resultado e identico ao da versao anterior.             */
#define ATRASO_VA_GRAUS       0.0f     /* modulo ZMPT101B da fase A            */
#define ATRASO_VB_GRAUS       0.0f     /* modulo ZMPT101B da fase B            */
#define ATRASO_VC_GRAUS       0.0f     /* modulo ZMPT101B da fase C            */
#define ATRASO_IA_GRAUS       0.0f     /* sensor de corrente QNHCK2-16 (A)     */
#define ATRASO_IB_GRAUS       0.0f     /* sensor de corrente QNHCK2-16 (B)     */
#define ATRASO_IC_GRAUS       0.0f     /* sensor de corrente QNHCK2-16 (C)     */

/* Filtro anti-aliasing RC de 1a ordem (R = 1,6 k; C = 10 nF -> fc = 9947 Hz).
 * Cada harmonica e corrigida por |H(f)| = 1/sqrt(1+(f/fc)^2). 0 = desliga.  */
#define AAF_FC_HZ             9947.0f

/* --- Limiares -------------------------------------------------------------- */
#define V_MIN_VALIDO          10.0f    /* V rms da fundamental p/ calcular %   */
#define I_MIN_VALIDO          0.5f     /* A rms da fundamental p/ calcular %   */

/* --- SOGI-PLL -------------------------------------------------------------- */
#define SOGI_K                1.41421356f  /* amortecimento do SOGI (raiz de 2) */
#define PLL_FN_HZ             15.0f        /* frequencia natural da malha      */
#define PLL_ZETA              0.7071f      /* amortecimento da malha           */
#define PLL_KP_HZ             (2.0f * PLL_ZETA * PLL_FN_HZ)       /* Hz/rad     */
#define PLL_KI_HZ             (DOIS_PI * PLL_FN_HZ * PLL_FN_HZ)   /* Hz/(rad.s) */
#define PLL_DESVIO_MAX_HZ     5.0f         /* faixa de captura 55..65 Hz       */
#define PLL_CICLOS_AQUEC      16u          /* ciclos p/ convergir (descartados)*/
#define PLL_AMP_MIN_PICO      30.0f        /* V de pico minimos p/ travar      */
#define PLL_ERRO_MAX_RAD      0.10f        /* erro de fase RMS max. travado    */

/* --- IEC -------------------------------------------------------------------- */
#define IEC_ERRO_SINC_MAX     0.0003f  /* +-0,03 % (IEC 61000-4-7)              */
#define IEC_ORDEM_THD         40u      /* ordem maxima do THD (IEC 61000-3-2/12) */
#define IEC_PWHD_INICIO       14u      /* PWHD: h = 14..40                      */
#define IEC_THD_V_LIMITE_PCT  8.0f     /* IEC 61000-2-2: THD de tensao <= 8 %   */
#define MAX_TENTATIVAS_SINC   2u       /* recapturas para acertar o sincronismo */

/* --- Registro --------------------------------------------------------------- */
#define CSV_SEP               ';'
#define CSV_DEC               ','      /* ',' abre direto no Excel pt-BR        */
#define ARQ_BRUTO             "BRUTO.CSV"
#define ARQ_HARM              "HARMON.CSV"
#define MEDIR_SEM_PENDRIVE    0        /* 1 = mede mesmo sem pendrive (debug)   */
#define TIMEOUT_AQUISICAO_MS  3000u
#define BRUTO_TODAS_CAPTURAS  1        /* 1 = BRUTO com todas as capturas;
                                          0 = so a 1a captura de cada intervalo
                                          (arquivo menor em ensaios longos)    */
#define ARQUIVO_NOVO_AO_LIGAR 1        /* 1 = ao ligar, renomeia HARMON.CSV e
                                          BRUTO.CSV existentes p/ HARM_nnn.CSV e
                                          BRUT_nnn.CSV (cada ensaio em arquivo
                                          proprio, relogio de novo em 00:00:00) */
#define Q_HARMON_NAO_ATIVA    0        /* reativa no HARMON.CSV:
                                          0 = Q1 da fundamental (IEEE 1459);
                                          1 = N = raiz(S^2 - P^2) com o sinal de
                                              Q1 (inclui a distorcao, como em
                                              analisadores comerciais)          */
#define H_DHT_MAX             50u      /* DHT e harmonicas do HARMON: 2..50     */
#define DEMANDA_MIN           15u      /* janela movel da demanda (minutos)     */

/* --- LEDs da Discovery (GPIOD) ---------------------------------------------- */
#define LED_VERDE             GPIO_PIN_12
#define LED_LARANJA           GPIO_PIN_13
#define LED_VERMELHO          GPIO_PIN_14
#define LED_AZUL              GPIO_PIN_15
#define LED_TODOS             (LED_VERDE | LED_LARANJA | LED_VERMELHO | LED_AZUL)

/* --- Erros de configuracao (mostrados em binario: PD12 = bit0 ... PD15 = bit3) */
#define ERRO_CFG_MULTIMODO    1u   /* ADC nao esta em triplo simultaneo regular */
#define ERRO_CFG_DMA_MODO     2u   /* DMA access mode diferente de 1            */
#define ERRO_CFG_DMA_LARGURA  3u   /* DMA nao esta em Half Word / Half Word     */
#define ERRO_CFG_N_CONV       4u   /* cada ADC deve ter 2 conversoes            */
#define ERRO_CFG_CANAIS       5u   /* canais diferentes de PA1..PA3 / PC1..PC3  */
#define ERRO_CFG_GATILHO      6u   /* gatilho do ADC1 diferente de TIM3 TRGO    */
                                   /* (TIM8 TRGO com SOBREAMOSTRAGEM = 1)       */

/* --- Constantes ---------------------------------------------------------------- */
#define DOIS_PI               6.28318531f
#define N_TAB                 128u     /* pontos por periodo da tabela seno     */
#define TAB_90GRAUS           32u      /* 90 graus = 128/4 posicoes             */

enum { SIG_VA = 0, SIG_VB, SIG_VC, SIG_IA, SIG_IB, SIG_IC };

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Acesso por indice inteiro (usado na FFT): cosseno = seno defasado 90 graus */
#define TAB_SEN(i)            (TABELA_SENO[(i) & (N_TAB - 1u)])
#define TAB_COS(i)            (TABELA_SENO[((i) + TAB_90GRAUS) & (N_TAB - 1u)])

/* Letras acentuadas em UTF-8 para o cabecalho do HARMON.CSV (o fonte segue
 * em ASCII). Use sempre como literal separado: "Tens" U_AT "o".            */
#define U_AT                  "\xC3\xA3"   /* a com til          */
#define U_EA                  "\xC3\xA9"   /* e agudo            */
#define U_IA                  "\xC3\xAD"   /* i agudo            */
#define U_AA                  "\xC3\xA1"   /* a agudo            */
#define U_EC                  "\xC3\xAA"   /* e circunflexo      */
#define U_OC                  "\xC3\xB4"   /* o circunflexo      */
#define U_ORD                 "\xC2\xAA"   /* a ordinal          */

#define FIM                   ((const char *)0)   /* fim da lista de Nome() */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

_Static_assert(N_FFT == N_TAB, "A FFT usa a tabela seno diretamente: ambas com 128 pontos");
_Static_assert(ADC_BUF_LEN <= 65535u, "O contador NDTR do DMA tem 16 bits");
_Static_assert(H_DHT_MAX < H_MAX, "H_DHT_MAX deve ficar abaixo de Nyquist");

/* Atraso de fase de cada sensor, na ordem dos sinais SIG_VA..SIG_IC
 * (valores definidos em ATRASO_*_GRAUS) */
static const float atraso_sensor_graus[N_CANAIS] =
{
  ATRASO_VA_GRAUS, ATRASO_VB_GRAUS, ATRASO_VC_GRAUS,
  ATRASO_IA_GRAUS, ATRASO_IB_GRAUS, ATRASO_IC_GRAUS
};

/* ---- Tipos que dependem das constantes acima -------------------------------- */

/* Grandezas derivadas de uma captura (fasores, angulos, linha, neutro...) */
typedef struct
{
  float   fre[N_CANAIS];     /* fasor fundamental corrigido (RMS): real         */
  float   fim[N_CANAIS];     /*                                    imaginario   */
  float   ang[N_CANAIS];     /* angulo da fundamental relativo a VA (graus)     */
  float   dht[N_CANAIS];     /* DHT (%) com h = 2..H_DHT_MAX                    */
  float   v_linha[3];        /* VAB, VBC, VCA true RMS (V)                      */
  float   i_neutro;          /* |ia + ib + ic| true RMS (A) - neutro calculado  */
  float   seq_v[3];          /* |V0|, |V1|, |V2| (V)                            */
  float   seq_i[3];          /* |I0|, |I1|, |I2| (A)                            */
  float   deseq_v;           /* V2/V1 (%)                                       */
  float   deseq_i;           /* I2/I1 (%)                                       */
  float   p_tri;             /* soma das potencias ativas (W)                   */
  float   q_tri;             /* soma das Q1 (var)                               */
  float   n_tri;             /* soma das N (var)                                */
  float   s_tri;             /* soma das aparentes Vrms*Irms (VA)               */
  float   fp_tri;            /* P_tri / S_tri                                   */
  float   fs;                /* taxa de amostragem usada nesta captura (Hz)     */
  uint8_t ang_ok[N_CANAIS];  /* 1 = canal e referencia (VA) com sinal          */
  uint8_t fp_ok[4];          /* 1 = FP valido (A, B, C, TRI)                    */
  uint8_t deseq_v_ok;
  uint8_t deseq_i_ok;
} Captura_t;

/* Acumulador de uma grandeza ao longo do intervalo de registro.
 * Zerado com memset: n = 0 indica que min/max ainda nao foram iniciados.  */
typedef struct
{
  double   soma;             /* soma (media aritmetica) ou soma dos quadrados   */
  float    min;
  float    max;
  uint32_t n;
} Agg_t;

typedef struct               /* fator de potencia: min/max com o tipo L/C       */
{
  float    min;
  float    max;
  char     tmin;
  char     tmax;
  uint32_t n;
} AggFP_t;

/* Tudo o que entra numa linha do HARMON.CSV. Indice 0..2 = fases A, B, C;
 * indice 3 = trifasico. Potencias em W/var/VA.                            */
typedef struct
{
  uint32_t n;                          /* capturas no intervalo               */
  uint64_t idx;                        /* numero do intervalo (0, 1, 2...)    */
  uint32_t intervalo_ms;               /* duracao usada neste intervalo       */
  uint64_t t_ult;                      /* instante da ultima captura (ms)     */
  Agg_t    rms[N_CANAIS];              /* quadratica; min/max de 12 ciclos    */
  Agg_t    f;                          /* frequencia                          */
  Agg_t    pc[4], pi[4], pt[4];        /* P consumida, injetada e total       */
  Agg_t    qic[4], qii[4], qcc[4], qci[4], qt[4]; /* Q por quadrante e total  */
  Agg_t    sa[4];                      /* aparente aritmetica Vrms*Irms       */
  AggFP_t  fp[4];
  Agg_t    deseq_v, deseq_i, i_n;      /* quadraticas                         */
  double   vl2[3];                     /* soma dos quadrados VAB, VBC, VCA    */
  double   h2[N_CANAIS][H_DHT_MAX + 1u]; /* soma dos quadrados de cada ordem  */
} Acc_t;

/* Demanda: media movel de DEMANDA_MIN minutos da potencia ativa consumida,
 * em baldes de 1 minuto (A, B, C, trifasico).                              */
typedef struct
{
  uint32_t minuto[DEMANDA_MIN];
  uint32_t n[DEMANDA_MIN];
  double   soma[DEMANDA_MIN][4];
  uint8_t  usado[DEMANDA_MIN];
} Demanda_t;

/* -----------------------------------------------------------------------------
 * TABELA SENO UNICA: sen(2*pi*k/128), k = 0..128
 * 128 pontos por periodo + ponto de guarda (k = 128 -> 2*pi) para interpolacao.
 * Cosseno: mesma tabela defasada 90 graus (indice + 32).
 * --------------------------------------------------------------------------- */
static const float TABELA_SENO[N_TAB + 1u] =
{
   0.000000000f,  0.049067674f,  0.098017140f,  0.146730474f,  0.195090322f,  0.242980180f,  0.290284677f,  0.336889853f,   /*   0..  7 */
   0.382683432f,  0.427555093f,  0.471396737f,  0.514102744f,  0.555570233f,  0.595699304f,  0.634393284f,  0.671558955f,   /*   8.. 15 */
   0.707106781f,  0.740951125f,  0.773010453f,  0.803207531f,  0.831469612f,  0.857728610f,  0.881921264f,  0.903989293f,   /*  16.. 23 */
   0.923879533f,  0.941544065f,  0.956940336f,  0.970031253f,  0.980785280f,  0.989176510f,  0.995184727f,  0.998795456f,   /*  24.. 31 */
   1.000000000f,  0.998795456f,  0.995184727f,  0.989176510f,  0.980785280f,  0.970031253f,  0.956940336f,  0.941544065f,   /*  32.. 39 */
   0.923879533f,  0.903989293f,  0.881921264f,  0.857728610f,  0.831469612f,  0.803207531f,  0.773010453f,  0.740951125f,   /*  40.. 47 */
   0.707106781f,  0.671558955f,  0.634393284f,  0.595699304f,  0.555570233f,  0.514102744f,  0.471396737f,  0.427555093f,   /*  48.. 55 */
   0.382683432f,  0.336889853f,  0.290284677f,  0.242980180f,  0.195090322f,  0.146730474f,  0.098017140f,  0.049067674f,   /*  56.. 63 */
   0.000000000f, -0.049067674f, -0.098017140f, -0.146730474f, -0.195090322f, -0.242980180f, -0.290284677f, -0.336889853f,   /*  64.. 71 */
  -0.382683432f, -0.427555093f, -0.471396737f, -0.514102744f, -0.555570233f, -0.595699304f, -0.634393284f, -0.671558955f,   /*  72.. 79 */
  -0.707106781f, -0.740951125f, -0.773010453f, -0.803207531f, -0.831469612f, -0.857728610f, -0.881921264f, -0.903989293f,   /*  80.. 87 */
  -0.923879533f, -0.941544065f, -0.956940336f, -0.970031253f, -0.980785280f, -0.989176510f, -0.995184727f, -0.998795456f,   /*  88.. 95 */
  -1.000000000f, -0.998795456f, -0.995184727f, -0.989176510f, -0.980785280f, -0.970031253f, -0.956940336f, -0.941544065f,   /*  96..103 */
  -0.923879533f, -0.903989293f, -0.881921264f, -0.857728610f, -0.831469612f, -0.803207531f, -0.773010453f, -0.740951125f,   /* 104..111 */
  -0.707106781f, -0.671558955f, -0.634393284f, -0.595699304f, -0.555570233f, -0.514102744f, -0.471396737f, -0.427555093f,   /* 112..119 */
  -0.382683432f, -0.336889853f, -0.290284677f, -0.242980180f, -0.195090322f, -0.146730474f, -0.098017140f, -0.049067674f,   /* 120..127 */
   0.000000000f                                                                                                              /* 128 (guarda) */
};

/* Buffer do DMA: quadros de 6 amostras [rank1: ADC1 ADC2 ADC3 | rank2: ADC1 ADC2 ADC3]
 * = [IA IB IC VA VB VC] na configuracao padrao (ordem real e autodetectada)  */
static uint16_t adc_buf[ADC_BUF_LEN] __attribute__((aligned(4)));

#if SOBREAMOSTRAGEM
/* buffer circular do DMA: duas metades de OS_BLOCO x OS_FATOR quadros       */
static uint16_t os_dma[OS_DMA_LEN] __attribute__((aligned(4)));
#if OS_LINHA_NA_CCM
#define OS_LINHA  ((float (*)[2u * FIR_N])0x10000000u)
#else
static float os_linha_sram[N_CANAIS][2u * FIR_N];
#define OS_LINHA  os_linha_sram
#endif
static uint32_t          os_pos = 0u;       /* posicao de escrita na linha     */
static uint32_t          os_n = 0u;         /* quadros brutos recebidos        */
static volatile uint32_t os_saidas = 0u;    /* amostras decimadas no adc_buf   */
static volatile uint8_t  os_sat[N_CANAIS];  /* amostra bruta em 0 ou 4095      */
static volatile uint32_t os_atrasos = 0u;   /* capturas descartadas porque a
                                               interrupcao nao acompanhou o DMA
                                               (ver no Live Expressions)       */

/* FIR passa-baixa de fase linear, 181 coeficientes, projetado (Parks-McClellan)
 * para 8 x 128 x 60 Hz = 61.440 Hz: banda passante 0..3.000 Hz (ordem 50),
 * ondulacao 0,08 %; banda de rejeicao a partir de 4.680 Hz (ordem 78), 97 dB.
 * Ganho 1 em 60 Hz. Como as bordas acompanham f (o TIM8 segue o PLL), elas
 * ficam sempre nas ordens 50 e 78. Simetrico: guarda-se c[0..90].          */
static const float FIR_C[FIR_MEIO + 1u] = {
  +1.217349305e-05f, +1.326823916e-05f, +1.727398268e-05f, +1.872807317e-05f, +1.567152616e-05f, +6.319748543e-06f,
  -1.044973006e-05f, -3.456781093e-05f, -6.436600324e-05f, -9.635557968e-05f, -1.253502834e-04f, -1.449592454e-04f,
  -1.485198082e-04f, -1.303190894e-04f, -8.697733330e-05f, -1.873250733e-05f, +6.957927580e-05f, +1.682435217e-04f,
  +2.632257651e-04f, +3.376660990e-04f, +3.741990623e-04f, +3.580517969e-04f, +2.801412395e-04f, +1.400112438e-04f,
  -5.226906798e-05f, -2.756775202e-04f, -4.996125203e-04f, -6.873079642e-04f, -8.009151941e-04f, -8.077918351e-04f,
  -6.869967969e-04f, -4.349959261e-04f, -6.935852278e-05f, +3.704916494e-04f, +8.261322271e-04f, +1.226673664e-03f,
  +1.498501805e-03f, +1.577036028e-03f, +1.418850147e-03f, +1.012151654e-03f, +3.836253415e-04f, -4.001023147e-04f,
  -1.237337076e-03f, -2.002722990e-03f, -2.564173433e-03f, -2.803301385e-03f, -2.636519253e-03f, -2.033369149e-03f,
  -1.028772058e-03f, +2.736697447e-04f, +1.708982386e-03f, +3.068979052e-03f, +4.129715885e-03f, +4.685014870e-03f,
  +4.581453212e-03f, +3.749415926e-03f, +2.224786051e-03f, +1.567013281e-04f, -2.201529344e-03f, -4.518949467e-03f,
  -6.428840961e-03f, -7.581257326e-03f, -7.699344262e-03f, -6.631198561e-03f, -4.388817818e-03f, -1.166766457e-03f,
  +2.664596330e-03f, +6.593140448e-03f, +1.002425779e-02f, +1.235946328e-02f, +1.308497973e-02f, +1.185861503e-02f,
  +8.582394997e-03f, +3.449327572e-03f, -3.044747613e-03f, -1.012912691e-02f, -1.682586014e-02f, -2.205140574e-02f,
  -2.474170214e-02f, -2.398613630e-02f, -1.915376428e-02f, -9.995138000e-03f, +3.294695341e-03f, +2.006217591e-02f,
  +3.923454706e-02f, +5.941027663e-02f, +7.899176187e-02f, +9.634591742e-02f, +1.099743131e-01f, +1.186729099e-01f,
  +1.216623255e-01f
};
#endif

/* Mapeamento sinal -> posicao no quadro, rank (0/1) e ganho */
static uint8_t  pos_sinal[N_CANAIS];
static uint8_t  rank_sinal[N_CANAIS];
static float    ganho_sinal[N_CANAIS];     /* unidade fisica por contagem   */
static float    atraso_rank_s = 0.0f;      /* tempo entre amostras rank1/2 */

/* Aquisicao */
static volatile uint8_t acq_concluida = 0u;
static volatile uint8_t acq_erro = 0u;
static uint32_t t_inicio_acq = 0u;         /* HAL_GetTick (timeout)         */
static uint64_t t_inicio_acq64 = 0u;       /* Tempo_ms (registro)           */
static float    fs_uso = 7680.0f;          /* taxa realmente programada     */

/* Relogio do registro: 64 bits a partir do HAL_GetTick, zerado na 1a captura */
static uint32_t intervalo_ms = INTERVALO_REGISTRO_MS;
static uint32_t tick_ult = 0u;
static uint32_t tick_voltas = 0u;
static uint64_t t_zero_ms = 0u;
static uint8_t  relogio_iniciado = 0u;

/* Resultados da ultima medicao (use no "Live Expressions" do CubeIDE) */
static float        media_cnt[N_CANAIS];           /* offset em contagens    */
static Estat_t      estat[N_CANAIS];
static Potencia_t   pot[3];
static float        harm[N_CANAIS][H_MAX + 1u];    /* RMS de cada ordem h    */
static ResultIEC_t  iec[N_CANAIS];
static float        f_medida = F_NOMINAL_HZ;       /* frequencia (SOGI-PLL)  */
static float        erro_sinc = 0.0f;              /* |fs/(128*f) - 1|       */
static uint8_t      pll_travado = 0u;
static uint8_t      sinc_ok = 0u;
static uint32_t     seq_medicao = 0u;
static uint64_t     t_medicao_ms = 0u;             /* inicio, no relogio     */
static uint32_t     recapturas = 0u;               /* descartadas p/ sinc.   */
static Captura_t    cap;

/* Registro do HARMON.CSV */
static Acc_t        acc;
static Demanda_t    dem;

/* Trabalho */
static float        fft_re[N_FFT];
static float        fft_im[N_FFT];
static float        fas_re[N_CANAIS];              /* soma dos fasores h1    */
static float        fas_im[N_CANAIS];              /* das 5 janelas          */
static SogiPll_t    pll;

/* Aplicacao */
static Estado_t     estado = EST_AGUARDA_MIDIA;
static uint8_t      midia_ok = 0u;
static uint32_t     t_montagem = 0u;
static uint32_t     tentativas_sinc = 0u;
static uint32_t     erro_config = 0u;
static uint8_t      sessao_iniciada = 0u;

/* Escrita bufferizada nos arquivos */
static char         wbuf[512];
static UINT         wlen = 0u;
static FIL         *wfil = NULL;
static FRESULT      wres = FR_OK;

/* Colunas do CSV: a mesma funcao escreve o cabecalho ou os valores */
static uint8_t      col_cab = 0u;                  /* 1 = cabecalho          */
static uint8_t      col_prim = 1u;                 /* 1 = primeira coluna    */
static char         col_nome[96];

extern ApplicationTypeDef Appli_state;     /* definido em usb_host.c       */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* 0) intervalo de registro */
void Registro_DefinirIntervalo(uint32_t ms);
uint32_t Registro_Intervalo(void);
/* 1) tabela */
static inline float Seno_Tab(float pos);
static inline float Cosseno_Tab(float pos);
static float Fase_Normaliza(float pos);
/* 2) FFT */
static void FFT128(float *re, float *im);
/* 3) estatisticas */
static void Estatisticas_Calcular(void);
/* 4) SOGI-PLL */
static void SogiPll_Reinicia(SogiPll_t *s, uint8_t manter_freq);
static float SogiPll_Passo(SogiPll_t *s, float u, float ts, float *amp);
static void Pll_Processar(void);
/* 5) harmonicos e grandezas derivadas */
static void Harmonicos_Calcular(void);
static void Sequencias(const float *re, const float *im, float *mag);
static void Grandezas_Calcular(void);
/* 6) IEC */
static float Limite_IEC61000_2_2_pct(uint32_t h);
static float Limite_IEC61000_3_2A_A(uint32_t h);
static void IEC_Registra(ResultIEC_t *r, uint32_t h, float razao);
static void IEC_Avaliar(void);
/* 7) CSV */
static void W_Inicia(FIL *f);
static void W_Descarrega(void);
static void W_Char(char ch);
static void W_Texto(const char *t);
static void W_Sep(void);
static void W_Fim(void);
static void W_Uint(uint32_t v);
static void W_Uint64(uint64_t v);
static void W_Float(float v, uint32_t dec);
static void W_Dois(uint32_t v);
static void W_Hora(uint64_t t_ms);
static uint32_t Dia(uint64_t t_ms);
static void Txt_Uint(char *dst, uint32_t v);
static void Col_Inicio(uint8_t cabecalho);
static void Nome(const char *s, ...);
static void Col_Abre(void);
static void ColF(float v, uint32_t dec);
static void ColFs(uint8_t ok, float v, uint32_t dec);
static void ColU(uint32_t v);
static void ColCh(uint8_t ok, char ch);
static void ColVazio(void);
static void Bruto_Colunas(void);
static void Bruto_Cabecalho(void);
static void Bruto_Linha(void);
static void Harm_Colunas(void);
static void Harm_Cabecalho(void);
static void Harm_Linha(void);
/* 7b) agregacao do intervalo */
static uint64_t Tempo_ms(void);
static void Agg_Soma(Agg_t *a, float x);
static void Agg_SomaQ(Agg_t *a, float x, float xmin, float xmax);
static float Agg_Valor(const Agg_t *a, uint32_t e, uint8_t quadratica);
static void AggFP_Soma(AggFP_t *a, float fp, char tipo);
static void Acc_Adicionar(void);
static void Demanda_Adicionar(uint64_t t_ms, const float *pc);
static float Demanda_Valor(uint32_t i, uint64_t t_ms);
/* 8) hardware */
static uint32_t Config_Verificar(void);
static float TIM3_ProgramarTaxa(float fs) __attribute__((unused));
static HAL_StatusTypeDef Aquisicao_Iniciar(void);
static uint8_t Aquisicao_Encerrar(void);
static void LED_Mascara(uint16_t acesos);
static void LED_Barra(uint32_t n);
static void LED_Piscar(uint16_t leds, uint32_t vezes, uint32_t periodo_ms);
static void Espera_ms(uint32_t ms);
static void Erro_Config_Laco(uint32_t codigo);
/* 9) cartao */
static void Midia_Servico(void);
static uint8_t Caminho(char *dst, size_t tam, const char *nome);
static void Arquivo_Arquivar(const char *nome, const char *prefixo);
static FRESULT Log_Anexar(const char *nome, void (*cabecalho)(void), void (*linhas)(void));
static uint8_t Registro_Gravar(void);
/* 10) aplicacao */
static void App_Iniciar(void);
static void App_Tarefa(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =============================================================================
 *  0) INTERVALO ENTRE REGISTROS DO HARMON.CSV  >>> ALTERE AQUI <<<
 *     Registro_DefinirIntervalo(ms): define o tempo entre duas linhas do
 *     HARMON.CSV (padrao INTERVALO_REGISTRO_MS = 5000 ms; aceita de
 *     INTERVALO_MIN_MS a INTERVALO_MAX_MS). Pode ser chamada a qualquer
 *     momento: o intervalo em andamento e gravado e o novo vale a partir da
 *     captura seguinte. Ex.: Registro_DefinirIntervalo(300000u) -> 5 min.
 * ============================================================================= */
void Registro_DefinirIntervalo(uint32_t ms)
{
  if (ms < INTERVALO_MIN_MS) { ms = INTERVALO_MIN_MS; }
  if (ms > INTERVALO_MAX_MS) { ms = INTERVALO_MAX_MS; }
  intervalo_ms = ms;
}

uint32_t Registro_Intervalo(void)
{
  return intervalo_ms;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Registro_DefinirIntervalo(INTERVALO_REGISTRO_MS);   /* ms entre registros */
  App_Iniciar();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    App_Tarefa();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSE 8 MHz -> 168 MHz; USB 48 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function (mestre do modo triplo)
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function (escravo)
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function (escravo)
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function (gatilho do ADC1 via TRGO)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10936;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level (PC0 = OTG_FS_PowerSwitchOn da Discovery) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 (LEDs) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* =============================================================================
 *  1) SENO / COSSENO PELA TABELA UNICA
 *     Fase em "passos de tabela": 0..128 corresponde a 0..2*pi.
 *     Fases continuas (PLL, correcoes) usam interpolacao linear entre pontos
 *     vizinhos (erro maximo ~3e-4); a FFT usa indices inteiros (exatos).
 * ============================================================================= */
static inline float Seno_Tab(float pos)
{
  uint32_t i = (uint32_t)pos;                     /* pos em [0,128)            */
  const float fr = pos - (float)i;
  i &= (N_TAB - 1u);
  return TABELA_SENO[i] + fr * (TABELA_SENO[i + 1u] - TABELA_SENO[i]);
}

static inline float Cosseno_Tab(float pos)
{
  float q = pos + (float)TAB_90GRAUS;             /* cos(x) = sen(x + 90 graus) */
  if (q >= (float)N_TAB)
  {
    q -= (float)N_TAB;
  }
  return Seno_Tab(q);
}

static float Fase_Normaliza(float pos)
{
  while (pos < 0.0f)          { pos += (float)N_TAB; }
  while (pos >= (float)N_TAB) { pos -= (float)N_TAB; }
  return pos;
}

/* =============================================================================
 *  2) FFT RADIX-2 DE 128 PONTOS (in-place, decimacao no tempo)
 *     X[k] = soma x[n] * e^(-j*2*pi*k*n/128)
 *     Fator de giro W = cos(2*pi*i/128) - j*sen(2*pi*i/128), lido da TABELA_SENO.
 *     A reordenacao bit-reversa e calculada em laco (sem tabela extra).
 * ============================================================================= */
static void FFT128(float *re, float *im)
{
  uint32_t i, j = 0u, k, m;

  for (i = 0u; i < (N_FFT - 1u); i++)            /* reordenacao bit-reversa   */
  {
    if (i < j)
    {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
    k = N_FFT >> 1;
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  for (m = 2u; m <= N_FFT; m <<= 1)              /* 7 estagios de borboletas  */
  {
    const uint32_t meio  = m >> 1;
    const uint32_t passo = N_FFT / m;            /* salto na tabela           */
    for (k = 0u; k < meio; k++)
    {
      const uint32_t idx = k * passo;
      const float wr =  TAB_COS(idx);
      const float wi = -TAB_SEN(idx);
      for (i = k; i < N_FFT; i += m)
      {
        const uint32_t l = i + meio;
        const float tr = wr * re[l] - wi * im[l];
        const float ti = wr * im[l] + wi * re[l];
        re[l] = re[i] - tr;
        im[l] = im[i] - ti;
        re[i] += tr;
        im[i] += ti;
      }
    }
  }
}

/* =============================================================================
 *  3) VALORES BRUTOS E POTENCIA (dominio do tempo, 60 ciclos inteiros)
 *     Somas inteiras exatas (uint64) -> media e RMS sem erro de arredondamento.
 *     Tambem calcula:
 *      - o RMS de cada janela de 12 ciclos (min/max do HARMON.CSV, como os
 *        valores de 10/12 ciclos da IEC 61000-4-30);
 *      - as tensoes de linha e a corrente de neutro em true RMS, pelos
 *        produtos cruzados de canais amostrados no mesmo instante (mesmo rank).
 * ============================================================================= */
static void Estatisticas_Calcular(void)
{
  const uint32_t n_ciclos = AMOSTRAS_CANAL / N_FFT;          /* 60           */
  const uint32_t n_jan    = N_FFT * CICLOS_JANELA;           /* 1536         */
  const double   nn       = (double)AMOSTRAS_CANAL;
  static uint32_t somaj[JANELAS][N_CANAIS];                  /* por janela   */
  static uint64_t soma2j[JANELAS][N_CANAIS];
  uint32_t soma[N_CANAIS], amp[N_CANAIS];
  uint64_t soma2[N_CANAIS];
  uint64_t sx[6];                    /* VA.VB VB.VC VC.VA IA.IB IB.IC IC.IA  */
  uint32_t px[6][2];                 /* posicao de cada par no quadro        */
  double   var_s[N_CANAIS];          /* variancia por sinal (contagens^2)    */
  uint16_t gmin[N_CANAIS], gmax[N_CANAIS];
  float    m[N_CANAIS], ret[N_CANAIS];
  int64_t  svi[3], svil[3];
  uint32_t kv[3], ki[3], lag[3];
  float    mu[3];
  const uint16_t *q;
  uint32_t c, r, k, p, n, w;

  for (k = 0u; k < N_CANAIS; k++)
  {
    amp[k] = 0u;  ret[k] = 0.0f;
    gmin[k] = 0xFFFFu;  gmax[k] = 0u;
  }
  memset(somaj, 0, sizeof(somaj));
  memset(soma2j, 0, sizeof(soma2j));
  for (k = 0u; k < 3u; k++)
  {
    px[k][0]      = pos_sinal[SIG_VA + k];
    px[k][1]      = pos_sinal[SIG_VA + ((k + 1u) % 3u)];
    px[k + 3u][0] = pos_sinal[SIG_IA + k];
    px[k + 3u][1] = pos_sinal[SIG_IA + ((k + 1u) % 3u)];
  }
  for (k = 0u; k < 6u; k++) { sx[k] = 0u; }

  /* ---- passe 1: somas por janela, produtos cruzados, min/max ------------- */
  q = adc_buf;
  for (c = 0u; c < n_ciclos; c++)
  {
    uint16_t cmin[N_CANAIS], cmax[N_CANAIS];
    w = c / CICLOS_JANELA;
    for (k = 0u; k < N_CANAIS; k++)
    {
      cmin[k] = 0xFFFFu;
      cmax[k] = 0u;
    }
    for (r = 0u; r < N_FFT; r++, q += N_CANAIS)
    {
      for (k = 0u; k < N_CANAIS; k++)
      {
        const uint16_t x = q[k];
        somaj[w][k]  += x;
        soma2j[w][k] += (uint32_t)x * x;
        if (x < cmin[k]) { cmin[k] = x; }
        if (x > cmax[k]) { cmax[k] = x; }
      }
      for (k = 0u; k < 6u; k++)
      {
        sx[k] += (uint32_t)q[px[k][0]] * q[px[k][1]];
      }
    }
    for (k = 0u; k < N_CANAIS; k++)
    {
      amp[k] += (uint32_t)cmax[k] - (uint32_t)cmin[k];
      if (cmin[k] < gmin[k]) { gmin[k] = cmin[k]; }
      if (cmax[k] > gmax[k]) { gmax[k] = cmax[k]; }
    }
  }
  for (k = 0u; k < N_CANAIS; k++)
  {
    soma[k]  = 0u;
    soma2[k] = 0u;
    for (w = 0u; w < JANELAS; w++)
    {
      soma[k]  += somaj[w][k];
      soma2[k] += soma2j[w][k];
    }
    m[k] = (float)((double)soma[k] / nn);
  }

  /* ---- correcao do atraso entre ranks (so para a potencia) --------------
   * dt = t(V) - t(I). Com dt > 0 a tensao foi amostrada depois da corrente:
   * v(t_i) = (1-mu)*v[n] + mu*v[n-1], com mu = |dt|*fs (interpolacao linear).
   * Indice circular: a captura tem um numero inteiro de ciclos.            */
  for (p = 0u; p < 3u; p++)
  {
    const float dt = ((float)rank_sinal[SIG_VA + p] - (float)rank_sinal[SIG_IA + p]) * atraso_rank_s;
    kv[p]   = pos_sinal[SIG_VA + p];
    ki[p]   = pos_sinal[SIG_IA + p];
    mu[p]   = fabsf(dt) * fs_uso;
    lag[p]  = (dt > 0.0f) ? (AMOSTRAS_CANAL - 1u) : 1u;
    svi[p]  = 0;
    svil[p] = 0;
  }

  /* ---- passe 2: valor medio retificado e somatorio de v*i ---------------- */
  q = adc_buf;
  n = 0u;
  for (c = 0u; c < n_ciclos; c++)
  {
    float pr[N_CANAIS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (r = 0u; r < N_FFT; r++, q += N_CANAIS, n++)
    {
      for (k = 0u; k < N_CANAIS; k++)
      {
        pr[k] += fabsf((float)q[k] - m[k]);
      }
      for (p = 0u; p < 3u; p++)
      {
        uint32_t nl = n + lag[p];
        if (nl >= AMOSTRAS_CANAL) { nl -= AMOSTRAS_CANAL; }
        {
          const int32_t ii = (int32_t)q[ki[p]];
          svi[p]  += (int32_t)q[kv[p]] * ii;
          svil[p] += (int32_t)adc_buf[nl * N_CANAIS + kv[p]] * ii;
        }
      }
    }
    for (k = 0u; k < N_CANAIS; k++)
    {
      ret[k] += pr[k];
    }
  }

  /* ---- grandezas fisicas por sinal (k = SIG_VA..SIG_IC) ------------------ */
  for (k = 0u; k < N_CANAIS; k++)
  {
    const uint32_t ps = pos_sinal[k];
    const float    g  = ganho_sinal[k];
    const float    ga = fabsf(g);
    const double   md = (double)soma[ps] / nn;
    double var = (double)soma2[ps] / nn - md * md;
    Estat_t *e = &estat[k];
    const float a = ((float)gmin[ps] - m[ps]) * g;
    const float b = ((float)gmax[ps] - m[ps]) * g;
    float pico, rmin = 3.4e38f, rmax = 0.0f;

    if (var < 0.0) { var = 0.0; }
    var_s[k]      = var;
    media_cnt[k]  = m[ps];
    e->rms        = sqrtf((float)var) * ga;
    e->max        = (a > b) ? a : b;
    e->min        = (a > b) ? b : a;
    e->amp_media  = 0.5f * ((float)amp[ps] / (float)n_ciclos) * ga;
    e->retif      = (ret[ps] / (float)AMOSTRAS_CANAL) * ga;
    pico          = (fabsf(e->max) > fabsf(e->min)) ? fabsf(e->max) : fabsf(e->min);
    e->fc         = (e->rms > 1.0e-6f) ? (pico / e->rms) : 0.0f;
    e->offset_v   = m[ps] * (VDDA_V / ADC_FUNDO_ESCALA);
#if SOBREAMOSTRAGEM
    e->saturou    = os_sat[ps];                 /* conferido nas amostras brutas */
#else
    e->saturou    = ((gmin[ps] == 0u) || (gmax[ps] >= 4095u)) ? 1u : 0u;
#endif

    /* RMS de cada janela de 12 ciclos, com o mesmo offset dos 60 ciclos:
     * <(x - md)^2>_j = <x^2>_j - 2*md*<x>_j + md^2                          */
    for (w = 0u; w < JANELAS; w++)
    {
      const double mj = (double)somaj[w][ps] / (double)n_jan;
      double vj = (double)soma2j[w][ps] / (double)n_jan - 2.0 * md * mj + md * md;
      float  rj;
      if (vj < 0.0) { vj = 0.0; }
      rj = sqrtf((float)vj) * ga;
      if (rj < rmin) { rmin = rj; }
      if (rj > rmax) { rmax = rj; }
    }
    e->rms_min = rmin;
    e->rms_max = rmax;
  }

  /* ---- tensoes de linha e corrente de neutro (true RMS) ------------------
   * v_ab = v_a - v_b      -> <v_ab^2> = ga^2*var_a + gb^2*var_b - 2*ga*gb*cov_ab
   * i_n  = i_a + i_b + i_c -> <i_n^2> = soma(g^2*var) + 2*soma(gj*gk*cov_jk)
   * Os ganhos entram com sinal (ganho negativo = sensor invertido).        */
  {
    double cov[6], in2 = 0.0;

    for (k = 0u; k < 6u; k++)
    {
      const double ma = (double)soma[px[k][0]] / nn;
      const double mb = (double)soma[px[k][1]] / nn;
      cov[k] = (double)sx[k] / nn - ma * mb;
    }
    for (k = 0u; k < 3u; k++)
    {
      const uint32_t sa = SIG_VA + k;
      const uint32_t sb = SIG_VA + ((k + 1u) % 3u);
      const double   ga = (double)ganho_sinal[sa];
      const double   gb = (double)ganho_sinal[sb];
      double v2 = ga * ga * var_s[sa] + gb * gb * var_s[sb] - 2.0 * ga * gb * cov[k];
      if (v2 < 0.0) { v2 = 0.0; }
      cap.v_linha[k] = sqrtf((float)v2);
    }
    for (k = 0u; k < 3u; k++)
    {
      const uint32_t sa = SIG_IA + k;
      const uint32_t sb = SIG_IA + ((k + 1u) % 3u);
      const double   ga = (double)ganho_sinal[sa];
      const double   gb = (double)ganho_sinal[sb];
      in2 += ga * ga * var_s[sa] + 2.0 * ga * gb * cov[3u + k];
    }
    if (in2 < 0.0) { in2 = 0.0; }
    cap.i_neutro = sqrtf((float)in2);
  }

  /* ---- potencia ativa (media de v*i) e aparente por fase ----------------- */
  for (p = 0u; p < 3u; p++)
  {
    const double mv = (double)soma[kv[p]] / nn;
    const double mi = (double)soma[ki[p]] / nn;
    const double c0 = (double)svi[p]  / nn - mv * mi;
    const double c1 = (double)svil[p] / nn - mv * mi;
    const double cv = (1.0 - (double)mu[p]) * c0 + (double)mu[p] * c1;
    Potencia_t *pt  = &pot[p];

    pt->p  = (float)cv * ganho_sinal[SIG_VA + p] * ganho_sinal[SIG_IA + p];
    pt->s  = estat[SIG_VA + p].rms * estat[SIG_IA + p].rms;
    pt->fp = (pt->s > 1.0e-3f) ? (pt->p / pt->s) : 0.0f;
  }
}

/* =============================================================================
 *  4) SOGI-PLL (fase A) - mede a frequencia da rede
 *     SOGI discretizado por Tustin e sintonizado na frequencia estimada:
 *        D(s) = k*w*s / (s^2 + k*w*s + w^2)  -> v'  (em fase)
 *        Q(s) = k*w^2 / (s^2 + k*w*s + w^2)  -> qv' (90 graus atrasado)
 *     Park no eixo q (seno/cosseno pela tabela) + PI + VCO.
 * ============================================================================= */
static void SogiPll_Reinicia(SogiPll_t *s, uint8_t manter_freq)
{
  const float f = s->f;
  const float integ = s->integ;

  memset(s, 0, sizeof(*s));
  s->f     = (manter_freq != 0u) ? f : F_NOMINAL_HZ;
  s->integ = (manter_freq != 0u) ? integ : 0.0f;
}

static float SogiPll_Passo(SogiPll_t *s, float u, float ts, float *amp)
{
  /* coeficientes Tustin para w' = 2*pi*f */
  const float W   = DOIS_PI * s->f * ts;
  const float x   = 2.0f * SOGI_K * W;
  const float y   = W * W;
  const float inv = 1.0f / (4.0f + x + y);
  const float b0  = x * inv;
  const float c1  = 2.0f * (4.0f - y) * inv;
  const float c2  = (x - y - 4.0f) * inv;
  const float qb0 = SOGI_K * y * inv;

  const float alfa = b0 * (u - s->u2) + c1 * s->a1 + c2 * s->a2;                /* v'  */
  const float beta = qb0 * (u + 2.0f * s->u1 + s->u2) + c1 * s->b1 + c2 * s->b2;  /* qv' */
  s->u2 = s->u1;  s->u1 = u;
  s->a2 = s->a1;  s->a1 = alfa;
  s->b2 = s->b1;  s->b1 = beta;

  /* Park: u = A*sen(tr) -> alfa = A*sen(tr), beta = -A*cos(tr)
   *       vq = alfa*cos(t) + beta*sen(t) = A*sen(tr - t)                     */
  {
    const float sn = Seno_Tab(s->fase);
    const float cs = Cosseno_Tab(s->fase);
    const float vq = alfa * cs + beta * sn;
    const float a  = sqrtf(alfa * alfa + beta * beta);
    const float e  = (a > 1.0e-3f) ? (vq / a) : 0.0f;   /* erro de fase ~ rad */
    float f;

    /* PI (saida em Hz) com anti-windup */
    s->integ += PLL_KI_HZ * ts * e;
    if (s->integ >  PLL_DESVIO_MAX_HZ) { s->integ =  PLL_DESVIO_MAX_HZ; }
    if (s->integ < -PLL_DESVIO_MAX_HZ) { s->integ = -PLL_DESVIO_MAX_HZ; }
    f = F_NOMINAL_HZ + PLL_KP_HZ * e + s->integ;
    if (f > (F_NOMINAL_HZ + PLL_DESVIO_MAX_HZ)) { f = F_NOMINAL_HZ + PLL_DESVIO_MAX_HZ; }
    if (f < (F_NOMINAL_HZ - PLL_DESVIO_MAX_HZ)) { f = F_NOMINAL_HZ - PLL_DESVIO_MAX_HZ; }
    s->f = f;

    /* VCO: 128 passos de tabela por ciclo */
    s->fase += (float)N_TAB * f * ts;
    if (s->fase >= (float)N_TAB) { s->fase -= (float)N_TAB; }

    *amp = a;
    return e;
  }
}

static void Pll_Processar(void)
{
  const float    ts  = 1.0f / fs_uso;
  const uint32_t k   = pos_sinal[SIG_VA];
  const float    m   = media_cnt[SIG_VA];
  const float    g   = ganho_sinal[SIG_VA];
  const uint32_t n0  = PLL_CICLOS_AQUEC * N_FFT;               /* aquecimento */
  const float    cnt = (float)(AMOSTRAS_CANAL - n0);
  float soma_df = 0.0f, soma_e2 = 0.0f, amp_min = 1.0e30f, amp = 0.0f;
  uint32_t n;

  /* partida "morna": aproveita a frequencia da captura anterior */
  SogiPll_Reinicia(&pll, pll_travado);

  for (n = 0u; n < AMOSTRAS_CANAL; n++)
  {
    const float u = ((float)adc_buf[n * N_CANAIS + k] - m) * g;
    const float e = SogiPll_Passo(&pll, u, ts, &amp);
    if (n >= n0)                    /* 44 ciclos inteiros: o ripple se anula */
    {
      soma_df += pll.f - F_NOMINAL_HZ;          /* soma o desvio: + precisao */
      soma_e2 += e * e;
      if (amp < amp_min) { amp_min = amp; }
    }
  }

  f_medida    = F_NOMINAL_HZ + soma_df / cnt;
  pll_travado = ((amp_min > PLL_AMP_MIN_PICO) &&
                 (sqrtf(soma_e2 / cnt) < PLL_ERRO_MAX_RAD) &&
                 (fabsf(f_medida - F_NOMINAL_HZ) < (PLL_DESVIO_MAX_HZ - 0.1f))) ? 1u : 0u;
}

/* =============================================================================
 *  5) HARMONICOS - IEC 61000-4-7 (janela de 12 ciclos) + IEC 61000-4-30
 *     Com fs = 128*f, a DFT de 12 ciclos (1536 pontos) nas ordens harmonicas
 *     e X1536[12h] = FFT128(soma dos 12 ciclos)[h]  (identidade exata).
 *     V e I da mesma fase entram juntos: z = v + j*i ->
 *        V[h] = (Z[h] + Z*[128-h]) / 2      I[h] = (Z[h] - Z*[128-h]) / 2j
 *     RMS: sqrt(2)*|X[h]|/128 (h < 64); a ordem 64 cai em Nyquist (fs/2).
 *     Os fasores da fundamental de cada janela sao somados em fas_re/fas_im
 *     para o calculo dos angulos e do desequilibrio (secao 5b).
 * ============================================================================= */
static void Harmonicos_Calcular(void)
{
  const float k_rms = 2.0f / ((float)N_FFT * (float)N_FFT);   /* |X|^2 -> RMS^2 */
  const float k_nyq = 1.0f / ((float)N_FFT * (float)N_FFT);   /* ordem 64       */
  const float f1    = fs_uso / (float)N_FFT;                   /* fundamental    */
  float p1[3]  = {0.0f, 0.0f, 0.0f};
  float q1[3]  = {0.0f, 0.0f, 0.0f};
  float p1s[3] = {0.0f, 0.0f, 0.0f};    /* P1 sem a correcao dos sensores   */
  float rc[3], rs[3], rc0[3], rs0[3];
  uint32_t w, p, r, c, h, s;

  memset(harm, 0, sizeof(harm));
  memset(fas_re, 0, sizeof(fas_re));
  memset(fas_im, 0, sizeof(fas_im));

  /* Rotacao que alinha o fasor fundamental de V ao instante da amostra de I:
   * desfaz o atraso entre ranks (dt) e os atrasos proprios dos sensores
   * (atraso de V menos atraso de I, um valor por fase).
   * Angulo em passos de tabela (128 = 360 graus): 128*f1*dt = fs*dt.
   * rc0/rs0: so o atraso entre ranks, para saber quanto a correcao dos
   * sensores muda a P1 (essa diferenca e somada a P no fim).               */
  for (p = 0u; p < 3u; p++)
  {
    const float dt   = ((float)rank_sinal[SIG_VA + p] - (float)rank_sinal[SIG_IA + p]) * atraso_rank_s;
    const float rank = -fs_uso * dt;
    const float sens = (atraso_sensor_graus[SIG_VA + p] - atraso_sensor_graus[SIG_IA + p])
                       * ((float)N_TAB / 360.0f);
    const float rho  = Fase_Normaliza(sens + rank);
    const float rho0 = Fase_Normaliza(rank);
    rc[p]  = Cosseno_Tab(rho);
    rs[p]  = Seno_Tab(rho);
    rc0[p] = Cosseno_Tab(rho0);
    rs0[p] = Seno_Tab(rho0);
  }

  for (w = 0u; w < JANELAS; w++)
  {
    const uint16_t *jan = &adc_buf[w * CICLOS_JANELA * N_FFT * N_CANAIS];

    for (p = 0u; p < 3u; p++)
    {
      const uint32_t sv = SIG_VA + p;
      const uint32_t si = SIG_IA + p;
      const uint32_t kv = pos_sinal[sv];
      const uint32_t ki = pos_sinal[si];
      const float    mv = (float)CICLOS_JANELA * media_cnt[sv];
      const float    mi = (float)CICLOS_JANELA * media_cnt[si];
      const float    gv = ganho_sinal[sv] / (float)CICLOS_JANELA;
      const float    gi = ganho_sinal[si] / (float)CICLOS_JANELA;

      /* media coerente dos 12 ciclos: z[r] = v[r] + j*i[r] */
      for (r = 0u; r < N_FFT; r++)
      {
        const uint16_t *q = jan + r * N_CANAIS;
        uint32_t av = 0u, ai = 0u;
        for (c = 0u; c < CICLOS_JANELA; c++, q += N_FFT * N_CANAIS)
        {
          av += q[kv];
          ai += q[ki];
        }
        fft_re[r] = ((float)av - mv) * gv;
        fft_im[r] = ((float)ai - mi) * gi;
      }

      FFT128(fft_re, fft_im);

      for (h = 1u; h <= H_MAX; h++)
      {
        const uint32_t hn = (N_FFT - h) & (N_FFT - 1u);
        const float xr = 0.5f * (fft_re[h] + fft_re[hn]);      /* V[h] */
        const float xi = 0.5f * (fft_im[h] - fft_im[hn]);
        const float yr = 0.5f * (fft_im[h] + fft_im[hn]);      /* I[h] */
        const float yi = 0.5f * (fft_re[hn] - fft_re[h]);
        const float kk = (h < H_MAX) ? k_rms : k_nyq;

        harm[sv][h] += (xr * xr + xi * xi) * kk;
        harm[si][h] += (yr * yr + yi * yi) * kk;

        if (h == 1u)                                           /* S1 = V1*I1' */
        {
          const float vr = xr * rc[p] - xi * rs[p];
          const float vi = xr * rs[p] + xi * rc[p];
          const float vr0 = xr * rc0[p] - xi * rs0[p];
          const float vi0 = xr * rs0[p] + xi * rc0[p];
          p1[p]  += (vr * yr + vi * yi) * k_rms;
          q1[p]  += (vi * yr - vr * yi) * k_rms;
          p1s[p] += (vr0 * yr + vi0 * yi) * k_rms;
          fas_re[sv] += xr;  fas_im[sv] += xi;                 /* fasores    */
          fas_re[si] += yr;  fas_im[si] += yi;
        }
      }
    }
  }

  /* Agregacao IEC 61000-4-30: raiz da media dos quadrados das 5 janelas.
   * Compensacao do filtro RC anti-aliasing: multiplica por sqrt(1+(f/fc)^2). */
  {
    const float inv_fc = (AAF_FC_HZ > 0.0f) ? (1.0f / AAF_FC_HZ) : 0.0f;
    for (s = 0u; s < N_CANAIS; s++)
    {
      harm[s][0] = 0.0f;                        /* DC = offset do sensor   */
      for (h = 1u; h <= H_MAX; h++)
      {
        const float x = (float)h * f1 * inv_fc;
        harm[s][h] = sqrtf(harm[s][h] / (float)JANELAS) * sqrtf(1.0f + x * x);
      }
    }
  }

  for (p = 0u; p < 3u; p++)
  {
    const float pm = p1[p] / (float)JANELAS;
    const float qm = q1[p] / (float)JANELAS;
    const float sm = sqrtf(pm * pm + qm * qm);
    pot[p].p1  = pm;
    pot[p].q1  = qm;
    pot[p].fpd = (sm > 1.0e-3f) ? (pm / sm) : 0.0f;

    /* Correcao de fase dos sensores na potencia ativa total: P vem das
     * amostras (Estatisticas_Calcular) e nao passa pela rotacao acima; soma
     * a mudanca que a correcao causou na fundamental (a parcela dominante).
     * Com os atrasos em 0, pm == pm0 e P fica exatamente igual.             */
    {
      const float pm0 = p1s[p] / (float)JANELAS;
      pot[p].p += pm - pm0;
      pot[p].fp = (pot[p].s > 1.0e-3f) ? (pot[p].p / pot[p].s) : 0.0f;
    }
  }
}

/* =============================================================================
 *  5b) GRANDEZAS DERIVADAS DE UMA CAPTURA
 *      - fasor da fundamental de cada canal (media das 5 janelas), corrigido
 *        do atraso do rank e do atraso proprio do sensor;
 *      - angulo de cada canal relativo a VA: atan2(Im(X*VA'), Re(X*VA'));
 *      - componentes simetricas (Fortescue, a = 1/120 graus):
 *          X0 = (Xa + Xb + Xc)/3   X1 = (Xa + a*Xb + a^2*Xc)/3
 *          X2 = (Xa + a^2*Xb + a*Xc)/3      desequilibrio = |X2|/|X1|*100;
 *      - DHT (h = 2..H_DHT_MAX), N = raiz(S^2 - P^2), phi1 e totais.
 * ============================================================================= */
static void Sequencias(const float *re, const float *im, float *mag)
{
  const float c120 = -0.5f, s120 = 0.866025404f;
  /* a*Xb, a^2*Xb, a*Xc, a^2*Xc */
  const float abr  = c120 * re[1] - s120 * im[1], abi  = s120 * re[1] + c120 * im[1];
  const float a2br = c120 * re[1] + s120 * im[1], a2bi = c120 * im[1] - s120 * re[1];
  const float acr  = c120 * re[2] - s120 * im[2], aci  = s120 * re[2] + c120 * im[2];
  const float a2cr = c120 * re[2] + s120 * im[2], a2ci = c120 * im[2] - s120 * re[2];
  const float x0r = (re[0] + re[1] + re[2]) / 3.0f, x0i = (im[0] + im[1] + im[2]) / 3.0f;
  const float x1r = (re[0] + abr + a2cr) / 3.0f,    x1i = (im[0] + abi + a2ci) / 3.0f;
  const float x2r = (re[0] + a2br + acr) / 3.0f,    x2i = (im[0] + a2bi + aci) / 3.0f;

  mag[0] = sqrtf(x0r * x0r + x0i * x0i);
  mag[1] = sqrtf(x1r * x1r + x1i * x1i);
  mag[2] = sqrtf(x2r * x2r + x2i * x2i);
}

static void Grandezas_Calcular(void)
{
  const float f1 = fs_uso / (float)N_FFT;
  const float xf = (AAF_FC_HZ > 0.0f) ? (f1 / AAF_FC_HZ) : 0.0f;
  /* soma de 5 janelas -> media; |X|*sqrt(2)/128 -> RMS; filtro RC */
  const float kf = 1.41421356f / ((float)N_FFT * (float)JANELAS) * sqrtf(1.0f + xf * xf);
  const float rad_graus = 57.2957795f;
  float ptot = 0.0f, qtot = 0.0f, ntot = 0.0f, stot = 0.0f;
  uint8_t algum_fp = 0u;
  uint32_t s, p, h;

  /* 1) fasores corrigidos: gira de (atraso do sensor - 360*f1*t_rank) graus */
  for (s = 0u; s < N_CANAIS; s++)
  {
    const float graus = atraso_sensor_graus[s]
                      - 360.0f * f1 * (float)rank_sinal[s] * atraso_rank_s;
    const float pos = Fase_Normaliza(graus * ((float)N_TAB / 360.0f));
    const float cr  = Cosseno_Tab(pos);
    const float sr  = Seno_Tab(pos);
    const float re  = fas_re[s] * kf;
    const float im  = fas_im[s] * kf;
    cap.fre[s] = re * cr - im * sr;
    cap.fim[s] = re * sr + im * cr;
  }

  /* 2) angulos relativos a VA (graus, -180..180) */
  {
    const float    rr  = cap.fre[SIG_VA];
    const float    ri  = cap.fim[SIG_VA];
    const uint8_t  ref = (harm[SIG_VA][1] >= V_MIN_VALIDO) ? 1u : 0u;
    for (s = 0u; s < N_CANAIS; s++)
    {
      const float lim = (s <= (uint32_t)SIG_VC) ? V_MIN_VALIDO : I_MIN_VALIDO;
      const float a   = cap.fre[s] * rr + cap.fim[s] * ri;     /* X * VA'    */
      const float b   = cap.fim[s] * rr - cap.fre[s] * ri;
      cap.ang_ok[s] = ((ref != 0u) && (harm[s][1] >= lim)) ? 1u : 0u;
      cap.ang[s]    = (cap.ang_ok[s] != 0u) ? (atan2f(b, a) * rad_graus) : 0.0f;
    }
  }

  /* 3) componentes simetricas e desequilibrio */
  Sequencias(&cap.fre[SIG_VA], &cap.fim[SIG_VA], cap.seq_v);
  Sequencias(&cap.fre[SIG_IA], &cap.fim[SIG_IA], cap.seq_i);
  cap.deseq_v_ok = (cap.seq_v[1] >= V_MIN_VALIDO) ? 1u : 0u;
  cap.deseq_i_ok = (cap.seq_i[1] >= I_MIN_VALIDO) ? 1u : 0u;
  cap.deseq_v    = (cap.deseq_v_ok != 0u) ? (100.0f * cap.seq_v[2] / cap.seq_v[1]) : 0.0f;
  cap.deseq_i    = (cap.deseq_i_ok != 0u) ? (100.0f * cap.seq_i[2] / cap.seq_i[1]) : 0.0f;

  /* 4) DHT de cada canal com h = 2..H_DHT_MAX (mesma faixa do HARMON.CSV) */
  for (s = 0u; s < N_CANAIS; s++)
  {
    float s2 = 0.0f;
    for (h = 2u; h <= H_DHT_MAX; h++)
    {
      s2 += harm[s][h] * harm[s][h];
    }
    cap.dht[s] = (iec[s].valido != 0u) ? (100.0f * sqrtf(s2) / harm[s][1]) : 0.0f;
  }

  /* 5) por fase: N, phi1, validade do FP; e os totais trifasicos */
  for (p = 0u; p < 3u; p++)
  {
    Potencia_t *pt = &pot[p];
    float n2 = pt->s * pt->s - pt->p * pt->p;
    if (n2 < 0.0f) { n2 = 0.0f; }
    pt->n    = (pt->q1 < 0.0f) ? -sqrtf(n2) : sqrtf(n2);
    pt->phi1 = atan2f(pt->q1, pt->p1) * rad_graus;
    cap.fp_ok[p] = ((estat[SIG_VA + p].rms >= V_MIN_VALIDO) &&
                    (estat[SIG_IA + p].rms >= I_MIN_VALIDO)) ? 1u : 0u;
    if (cap.fp_ok[p] != 0u) { algum_fp = 1u; }
    ptot += pt->p;
    qtot += pt->q1;
    ntot += pt->n;
    stot += pt->s;
  }
  cap.p_tri    = ptot;
  cap.q_tri    = qtot;
  cap.n_tri    = ntot;
  cap.s_tri    = stot;
  cap.fp_ok[3] = ((algum_fp != 0u) && (stot > 1.0e-3f)) ? 1u : 0u;
  cap.fp_tri   = (cap.fp_ok[3] != 0u) ? (ptot / stot) : 0.0f;
}

/* =============================================================================
 *  6) INDICADORES (THD, PWHD) E LIMITES
 *     Tensao  : IEC 61000-2-2, Tabela 1 - niveis de compatibilidade em BT (%)
 *     Corrente: IEC 61000-3-2, classe A (A) - equipamento <= 16 A por fase.
 *               Harmonicas < 0,6 % da corrente ou < 5 mA sao desprezadas.
 * ============================================================================= */
static float Limite_IEC61000_2_2_pct(uint32_t h)
{
  if ((h < 2u) || (h > 50u)) { return 0.0f; }
  if ((h & 1u) != 0u)
  {
    if ((h % 3u) != 0u)                           /* impares nao mult. de 3 */
    {
      switch (h)
      {
        case 5u:  return 6.0f;
        case 7u:  return 5.0f;
        case 11u: return 3.5f;
        case 13u: return 3.0f;
        default:  return 2.27f * (17.0f / (float)h) - 0.27f;   /* 17..49 */
      }
    }
    switch (h)                                    /* impares mult. de 3     */
    {
      case 3u:  return 5.0f;
      case 9u:  return 1.5f;
      case 15u: return 0.4f;
      case 21u: return 0.3f;
      default:  return 0.2f;                      /* 21 < h <= 45           */
    }
  }
  switch (h)                                      /* pares                  */
  {
    case 2u: return 2.0f;
    case 4u: return 1.0f;
    case 6u: return 0.5f;
    case 8u: return 0.5f;
    default: return 0.25f * (10.0f / (float)h) + 0.25f;       /* 10..50 */
  }
}

static float Limite_IEC61000_3_2A_A(uint32_t h)
{
  if ((h < 2u) || (h > 40u)) { return 0.0f; }
  if ((h & 1u) != 0u)
  {
    switch (h)
    {
      case 3u:  return 2.30f;
      case 5u:  return 1.14f;
      case 7u:  return 0.77f;
      case 9u:  return 0.40f;
      case 11u: return 0.33f;
      case 13u: return 0.21f;
      default:  return 0.15f * 15.0f / (float)h;              /* 15..39 */
    }
  }
  switch (h)
  {
    case 2u: return 1.08f;
    case 4u: return 0.43f;
    case 6u: return 0.30f;
    default: return 0.23f * 8.0f / (float)h;                  /* 8..40  */
  }
}

static void IEC_Registra(ResultIEC_t *r, uint32_t h, float razao)
{
  if (razao > 1.0f) { r->excede++; }
  if ((100.0f * razao) > r->pior_pct)
  {
    r->pior_pct = 100.0f * razao;
    r->pior_h   = (uint8_t)h;
  }
}

static void IEC_Avaliar(void)
{
  uint32_t s, h;

  for (s = 0u; s < N_CANAIS; s++)
  {
    ResultIEC_t *r = &iec[s];
    const uint8_t tensao = (s <= (uint32_t)SIG_VC) ? 1u : 0u;
    const float h1 = harm[s][1];
    float s40 = 0.0f, s63 = 0.0f, sw = 0.0f;

    for (h = 2u; h < H_MAX; h++)                 /* 2..63 (64 = Nyquist)    */
    {
      const float c2 = harm[s][h] * harm[s][h];
      s63 += c2;
      if (h <= IEC_ORDEM_THD)
      {
        s40 += c2;
        if (h >= IEC_PWHD_INICIO) { sw += (float)h * c2; }
      }
    }

    memset(r, 0, sizeof(*r));
    r->h1     = h1;
    r->hrms40 = sqrtf(s40);
    r->valido = (h1 >= ((tensao != 0u) ? V_MIN_VALIDO : I_MIN_VALIDO)) ? 1u : 0u;
    if (r->valido == 0u) { continue; }

    r->thd40 = 100.0f * sqrtf(s40) / h1;
    r->thd63 = 100.0f * sqrtf(s63) / h1;
    r->pwhd  = 100.0f * sqrtf(sw)  / h1;

    if (tensao != 0u)
    {
      for (h = 2u; h <= 50u; h++)
      {
        IEC_Registra(r, h, (100.0f * harm[s][h] / h1) / Limite_IEC61000_2_2_pct(h));
      }
      r->conforme = ((r->excede == 0u) && (r->thd40 <= IEC_THD_V_LIMITE_PCT)) ? 1u : 0u;
    }
    else
    {
      float desprezar = 0.006f * estat[s].rms;
      if (desprezar < 0.005f) { desprezar = 0.005f; }
      for (h = 2u; h <= 40u; h++)
      {
        if (harm[s][h] >= desprezar)
        {
          IEC_Registra(r, h, harm[s][h] / Limite_IEC61000_3_2A_A(h));
        }
      }
      r->conforme = (r->excede == 0u) ? 1u : 0u;
    }
  }
}

/* =============================================================================
 *  7) FORMATACAO CSV (sem printf/float da newlib: menos pilha, sem malloc)
 * ============================================================================= */
static void W_Inicia(FIL *f)
{
  wfil = f;
  wlen = 0u;
  wres = FR_OK;
}

static void W_Descarrega(void)
{
  UINT escritos = 0u;
  if ((wlen > 0u) && (wres == FR_OK))
  {
    wres = f_write(wfil, wbuf, wlen, &escritos);
    if ((wres == FR_OK) && (escritos != wlen)) { wres = FR_DENIED; }  /* cheio */
  }
  wlen = 0u;
}

static void W_Char(char ch)
{
  wbuf[wlen++] = ch;
  if (wlen >= (UINT)sizeof(wbuf)) { W_Descarrega(); }
}

static void W_Texto(const char *t)
{
  while (*t != '\0') { W_Char(*t++); }
}

static void W_Sep(void) { W_Char(CSV_SEP); }

static void W_Fim(void) { W_Char('\r'); W_Char('\n'); }

static void W_Uint(uint32_t v)
{
  char tmp[10];
  uint32_t n = 0u;
  do
  {
    tmp[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u);
  while (n > 0u) { W_Char(tmp[--n]); }
}

static void W_Float(float v, uint32_t dec)
{
  uint32_t esc = 1u, ip, fp, i;
  uint8_t neg = 0u;
  float fr;

  if (v != v) { W_Texto("NaN"); return; }
  if (v < 0.0f) { neg = 1u; v = -v; }
  if (v >= 4.0e9f) { W_Texto((neg != 0u) ? "-ovf" : "ovf"); return; }
  for (i = 0u; i < dec; i++) { esc *= 10u; }

  ip = (uint32_t)v;
  fr = v - (float)ip;
  fp = (uint32_t)(fr * (float)esc + 0.5f);
  if (fp >= esc) { ip++; fp -= esc; }

  if ((neg != 0u) && ((ip | fp) != 0u)) { W_Char('-'); }
  W_Uint(ip);
  if (dec > 0u)
  {
    W_Char(CSV_DEC);
    for (esc /= 10u; esc > 0u; esc /= 10u)
    {
      W_Char((char)('0' + ((fp / esc) % 10u)));
    }
  }
}

static void W_Uint64(uint64_t v)
{
  char tmp[20];
  uint32_t n = 0u;
  do
  {
    tmp[n++] = (char)('0' + (uint32_t)(v % 10u));
    v /= 10u;
  } while (v != 0u);
  while (n > 0u) { W_Char(tmp[--n]); }
}

static void W_Dois(uint32_t v)                 /* 2 digitos, zero a esquerda */
{
  W_Char((char)('0' + ((v / 10u) % 10u)));
  W_Char((char)('0' + (v % 10u)));
}

static void W_Hora(uint64_t t_ms)              /* HH:MM:SS do relogio        */
{
  const uint32_t s = (uint32_t)((t_ms / 1000u) % 86400u);
  W_Dois(s / 3600u);        W_Char(':');
  W_Dois((s / 60u) % 60u);  W_Char(':');
  W_Dois(s % 60u);
}

static uint32_t Dia(uint64_t t_ms)             /* 1 = primeiro dia           */
{
  return (uint32_t)(t_ms / 86400000u) + 1u;
}

static void Txt_Uint(char *dst, uint32_t v)    /* inteiro -> texto           */
{
  char tmp[10];
  uint32_t n = 0u, i = 0u;
  do
  {
    tmp[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u);
  while (n > 0u) { dst[i++] = tmp[--n]; }
  dst[i] = '\0';
}

/* ---- Colunas: a mesma rotina gera o cabecalho (col_cab = 1) ou a linha de
 *      valores (col_cab = 0), entao nome e valor ficam sempre alinhados.   */
static void Col_Inicio(uint8_t cabecalho)
{
  col_cab  = cabecalho;
  col_prim = 1u;
}

static void Nome(const char *s, ...)           /* concatena os textos ate FIM */
{
  va_list ap;
  size_t  n = 0u;

  if (col_cab == 0u) { return; }
  va_start(ap, s);
  while (s != FIM)
  {
    while ((*s != '\0') && (n < (sizeof(col_nome) - 1u))) { col_nome[n++] = *s++; }
    s = va_arg(ap, const char *);
  }
  va_end(ap);
  col_nome[n] = '\0';
}

static void Col_Abre(void)
{
  if (col_prim == 0u) { W_Sep(); }
  col_prim = 0u;
  if (col_cab != 0u) { W_Texto(col_nome); }
}

static void ColF(float v, uint32_t dec)
{
  Col_Abre();
  if (col_cab == 0u) { W_Float(v, dec); }
}

static void ColFs(uint8_t ok, float v, uint32_t dec)   /* vazio se ok = 0 */
{
  Col_Abre();
  if ((col_cab == 0u) && (ok != 0u)) { W_Float(v, dec); }
}

static void ColU(uint32_t v)
{
  Col_Abre();
  if (col_cab == 0u) { W_Uint(v); }
}

static void ColCh(uint8_t ok, char ch)
{
  Col_Abre();
  if ((col_cab == 0u) && (ok != 0u)) { W_Char(ch); }
}

static void ColVazio(void)                     /* grandeza nao medida */
{
  Col_Abre();
}

static const char *const EST_NOME[3]       = { "m" U_EA "dio", "m" U_IA "n", "m" U_AA "x" };
static const char *const FASE_N[4]         = { "A", "B", "C", "TRI" };
static const char *const FASE_TRIF[4]      = { "A", "B", "C", "Trif" U_AA "sica" };
static const char *const FASE_AP[4]        = { "A", "B", "C", "Trif." };
static const char *const LINHA_N[3]        = { "AB", "BC", "CA" };
static const char *const CANAL_N[N_CANAIS] = { "VA", "VB", "VC", "IA", "IB", "IC" };

/* ---- BRUTO.CSV: uma linha por captura de 60 ciclos ------------------------
 *  Angulos (_ang_graus): fundamental de cada canal relativa a VA, em graus
 *  (-180..180), ja corrigida do atraso entre ranks e dos sensores. Em
 *  sequencia ABC: VB ~ -120, VC ~ +120; a corrente de carga indutiva fica
 *  atrasada da sua tensao. phi1 = angulo(V1) - angulo(I1) da mesma fase.  */
static void Bruto_Colunas(void)
{
  uint32_t s, p;

  Nome("seq", FIM);            ColU(seq_medicao);
  Nome("Data", FIM);           ColU(Dia(t_medicao_ms));
  Nome("Hora", FIM);           Col_Abre(); if (col_cab == 0u) { W_Hora(t_medicao_ms); }
  Nome("t_ms", FIM);           Col_Abre(); if (col_cab == 0u) { W_Uint64(t_medicao_ms); }
  Nome("f_Hz", FIM);           ColF(f_medida, 4u);
  Nome("pll", FIM);            ColU(pll_travado);
  Nome("sinc", FIM);           ColU(sinc_ok);
  Nome("erro_sinc_pct", FIM);  ColFs(pll_travado, 100.0f * erro_sinc, 4u);
  Nome("fs_Hz", FIM);          ColF(cap.fs, 2u);
  Nome("recapturas", FIM);     ColU(recapturas);
  Nome("saturados", FIM);      Col_Abre();
  if (col_cab == 0u)
  {
    uint32_t n = 0u;
    for (s = 0u; s < N_CANAIS; s++)
    {
      if (estat[s].saturou != 0u)
      {
        if (n != 0u) { W_Char(' '); }
        W_Texto(CANAL_N[s]);
        n++;
      }
    }
    if (n == 0u) { W_Char('-'); }
  }

  for (s = 0u; s < N_CANAIS; s++)
  {
    const char        *c = CANAL_N[s];
    const char        *u = (s <= (uint32_t)SIG_VC) ? "_V" : "_A";
    const Estat_t     *e = &estat[s];
    const ResultIEC_t *r = &iec[s];

    Nome(c, "_rms", u, FIM);           ColF(e->rms, 3u);
    Nome(c, "_rms_min12c", u, FIM);    ColF(e->rms_min, 3u);
    Nome(c, "_rms_max12c", u, FIM);    ColF(e->rms_max, 3u);
    Nome(c, "_h1", u, FIM);            ColF(harm[s][1], 3u);
    Nome(c, "_ang_graus", FIM);        ColFs(cap.ang_ok[s], cap.ang[s], 2u);
    Nome(c, "_max", u, FIM);           ColF(e->max, 3u);
    Nome(c, "_min", u, FIM);           ColF(e->min, 3u);
    Nome(c, "_amp_media", u, FIM);     ColF(e->amp_media, 3u);
    Nome(c, "_med_retif", u, FIM);     ColF(e->retif, 3u);
    Nome(c, "_fcrista", FIM);          ColF(e->fc, 3u);
    Nome(c, "_offset_pino_V", FIM);    ColF(e->offset_v, 4u);
    Nome(c, "_DHT_pct", FIM);          ColFs(r->valido, cap.dht[s], 3u);
    Nome(c, "_THD40_pct", FIM);        ColFs(r->valido, r->thd40, 3u);
    Nome(c, "_IEC_ok", FIM);           ColCh(r->valido, (r->conforme != 0u) ? '1' : '0');
  }

  for (p = 0u; p < 3u; p++)
  {
    const char       *f  = FASE_N[p];
    const Potencia_t *pt = &pot[p];

    Nome("phi1_", f, "_graus", FIM);   ColFs(cap.fp_ok[p], pt->phi1, 2u);
    Nome("P_", f, "_W", FIM);          ColF(pt->p, 1u);
    Nome("P1_", f, "_W", FIM);         ColF(pt->p1, 1u);
    Nome("Q1_", f, "_var", FIM);       ColF(pt->q1, 1u);
    Nome("N_", f, "_var", FIM);        ColF(pt->n, 1u);
    Nome("S_", f, "_VA", FIM);         ColF(pt->s, 1u);
    Nome("FP_", f, FIM);               ColFs(cap.fp_ok[p], pt->fp, 4u);
    Nome("FPD_", f, FIM);              ColFs(cap.fp_ok[p], pt->fpd, 4u);
  }
  Nome("P_TRI_W", FIM);        ColF(cap.p_tri, 1u);
  Nome("Q1_TRI_var", FIM);     ColF(cap.q_tri, 1u);
  Nome("N_TRI_var", FIM);      ColF(cap.n_tri, 1u);
  Nome("S_TRI_VA", FIM);       ColF(cap.s_tri, 1u);
  Nome("FP_TRI", FIM);         ColFs(cap.fp_ok[3], cap.fp_tri, 4u);
  for (p = 0u; p < 3u; p++)
  {
    Nome("V", LINHA_N[p], "_V", FIM);  ColF(cap.v_linha[p], 3u);
  }
  Nome("IN_calc_A", FIM);      ColF(cap.i_neutro, 3u);
  Nome("V0_V", FIM);           ColF(cap.seq_v[0], 3u);
  Nome("V1_V", FIM);           ColF(cap.seq_v[1], 3u);
  Nome("V2_V", FIM);           ColF(cap.seq_v[2], 3u);
  Nome("Deseq_V_pct", FIM);    ColFs(cap.deseq_v_ok, cap.deseq_v, 3u);
  Nome("I0_A", FIM);           ColF(cap.seq_i[0], 3u);
  Nome("I1_A", FIM);           ColF(cap.seq_i[1], 3u);
  Nome("I2_A", FIM);           ColF(cap.seq_i[2], 3u);
  Nome("Deseq_I_pct", FIM);    ColFs(cap.deseq_i_ok, cap.deseq_i, 3u);
}

static void Bruto_Cabecalho(void)
{
  Col_Inicio(1u);
  Bruto_Colunas();
  W_Fim();
}

static void Bruto_Linha(void)
{
  Col_Inicio(0u);
  Bruto_Colunas();
  W_Fim();
}

/* ---- HARMON.CSV: uma linha por intervalo, leiaute de colunas do P55 -------
 *  Mesma ordem e nomes de coluna do P55, com estas diferencas:
 *   - harmonicas em valor RMS: "... - 2(V)" / "... - 2(A)" no lugar de (%);
 *   - "Corr. N. Calculada" (ia+ib+ic) no lugar de "Corr. N. Medida";
 *   - Tensao Neutro, Pst e Plt nao sao medidos: colunas vazias;
 *   - sem as 8 linhas de identificacao do P55: o cabecalho e a linha 1;
 *   - Data = dia do ensaio (1, 2...); Hora = inicio do intervalo.       */
static void Harm_Colunas(void)
{
  const Acc_t   *a  = &acc;
  const uint64_t t0 = a->idx * (uint64_t)a->intervalo_ms;      /* inicio    */
  const float    th = (float)a->intervalo_ms / 3600000.0f;     /* horas     */
  const float    k  = 0.001f;                                  /* W -> kW   */
  const double   na = (a->n > 0u) ? (double)a->n : 1.0;
  float pm[4], qm[4], sm[4];
  char  hs[4];
  uint32_t p, e, s, h;

  for (p = 0u; p < 4u; p++)
  {
    pm[p] = Agg_Valor(&a->pt[p], 0u, 0u);
    qm[p] = Agg_Valor(&a->qt[p], 0u, 0u);
    sm[p] = Agg_Valor(&a->sa[p], 0u, 0u);
  }

  Nome("Data", FIM);   ColU(Dia(t0));
  Nome("Hora ", FIM);  Col_Abre(); if (col_cab == 0u) { W_Hora(t0); }

  /* tensao e corrente RMS: media quadratica; min/max dos valores de 12 ciclos */
  for (s = 0u; s < N_CANAIS; s++)
  {
    const uint8_t tensao = (s <= (uint32_t)SIG_VC) ? 1u : 0u;
    for (e = 0u; e < 3u; e++)
    {
      Nome((tensao != 0u) ? ("Tens" U_AT "o ") : "Corrente ", FASE_N[s % 3u], " (", EST_NOME[e],
           (tensao != 0u) ? ")(V)" : ")(A)", FIM);
      ColFs((a->rms[s].n > 0u) ? 1u : 0u, Agg_Valor(&a->rms[s], e, 1u), 3u);
    }
  }

  /* fator de potencia (L = indutivo, C = capacitivo) */
  for (p = 0u; p < 4u; p++)
  {
    const AggFP_t *f = &a->fp[p];
    for (e = 0u; e < 3u; e++)
    {
      uint8_t ok = (f->n > 0u) ? 1u : 0u;
      float   v;
      char    t;
      if (e == 0u)
      {
        ok = ((ok != 0u) && (sm[p] > 1.0e-3f)) ? 1u : 0u;
        v  = (ok != 0u) ? (pm[p] / sm[p]) : 0.0f;
        t  = (qm[p] < 0.0f) ? 'C' : 'L';
      }
      else if (e == 1u) { v = f->min; t = f->tmin; }
      else              { v = f->max; t = f->tmax; }
      Nome("FP ", FASE_N[p], " (", EST_NOME[e], ")()", FIM);      ColFs(ok, v, 4u);
      Nome("FP ", FASE_N[p], " (", EST_NOME[e], ")(Tipo)", FIM);  ColCh(ok, t);
    }
  }

  /* potencia ativa consumida / injetada / total e energia do intervalo */
  for (p = 0u; p < 4u; p++)
  {
    for (e = 0u; e < 3u; e++)
    {
      Nome("Pot Ativa Cons. ", FASE_TRIF[p], " Cons. (", EST_NOME[e], ")(kW)", FIM);
      ColF(Agg_Valor(&a->pc[p], e, 0u) * k, 5u);
      Nome("Pot Ativa Inj. ", FASE_TRIF[p], " Forn. (", EST_NOME[e], ")(kW)", FIM);
      ColF(Agg_Valor(&a->pi[p], e, 0u) * k, 5u);
      Nome("Pot Ativa ", FASE_N[p], " (", EST_NOME[e], ")(kW)", FIM);
      ColF(Agg_Valor(&a->pt[p], e, 0u) * k, 5u);
      if (e == 0u)
      {
        Nome("Energia ", FASE_N[p], " Cons. (", EST_NOME[0], ")((Kwh))", FIM);
        ColF(Agg_Valor(&a->pc[p], 0u, 0u) * k * th, 7u);
        Nome("Energia ", FASE_N[p], " Forn. (", EST_NOME[0], ")((Kwh))", FIM);
        ColF(Agg_Valor(&a->pi[p], 0u, 0u) * k * th, 7u);
      }
    }
  }

  /* potencia reativa por quadrante (Ind.: Q > 0; Cap.: Q < 0) e total */
  for (p = 0u; p < 4u; p++)
  {
    for (e = 0u; e < 3u; e++)
    {
      Nome("Pot Reativa Ind. Cons. ", FASE_TRIF[p], " Cons. (", EST_NOME[e], ")(kVAr)", FIM);
      ColF(Agg_Valor(&a->qic[p], e, 0u) * k, 5u);
      Nome("Pot Reativa Ind. Inj. ", FASE_TRIF[p], " Forn. (", EST_NOME[e], ")(kVAr)", FIM);
      ColF(Agg_Valor(&a->qii[p], e, 0u) * k, 5u);
      Nome("Pot Reativa Cap. Cons. ", FASE_TRIF[p], " Cons. (", EST_NOME[e], ")(kVAr)", FIM);
      ColF(Agg_Valor(&a->qcc[p], e, 0u) * k, 5u);
      Nome("Pot Reativa Cap. Inj. ", FASE_TRIF[p], " Forn. (", EST_NOME[e], ")(kVAr)", FIM);
      ColF(Agg_Valor(&a->qci[p], e, 0u) * k, 5u);
      Nome("Pot Reativa ", FASE_N[p], " (", EST_NOME[e], ")(kVAr)", FIM);
      ColF(Agg_Valor(&a->qt[p], e, 0u) * k, 5u);
    }
  }

  /* aparente: (a) aritmetica Vrms*Irms; (v) vetorial raiz(P^2 + Q^2) */
  for (p = 0u; p < 4u; p++)
  {
    for (e = 0u; e < 3u; e++)
    {
      Nome("Pot Aparente ", FASE_AP[p], " (a) (", EST_NOME[e], ")(kVA)", FIM);
      ColF(Agg_Valor(&a->sa[p], e, 0u) * k, 5u);
    }
  }
  for (p = 0u; p < 4u; p++)
  {
    Nome("Pot Aparente ", FASE_AP[p], " (v) (", EST_NOME[0], ")(kVA)", FIM);
    ColF(sqrtf(pm[p] * pm[p] + qm[p] * qm[p]) * k, 5u);
  }

  for (e = 0u; e < 3u; e++)
  {
    Nome("Frequ" U_EC "ncia (", EST_NOME[e], ")(Hz)", FIM);
    ColF(Agg_Valor(&a->f, e, 0u), 4u);
  }

  /* DHT total, par, impar e tripla (PRODIST: pares e impares nao multiplos
   * de 3; triplas = multiplos de 3), h = 2..H_DHT_MAX, e a fundamental    */
  for (s = 0u; s < N_CANAIS; s++)
  {
    const uint8_t tensao = (s <= (uint32_t)SIG_VC) ? 1u : 0u;
    const float   h1     = sqrtf((float)(a->h2[s][1] / na));
    const uint8_t ok     = (h1 >= ((tensao != 0u) ? V_MIN_VALIDO : I_MIN_VALIDO)) ? 1u : 0u;
    double st = 0.0, sp = 0.0, si = 0.0, s3 = 0.0;
    float  kd;

    for (h = 2u; h <= H_DHT_MAX; h++)
    {
      const double x = a->h2[s][h] / na;
      st += x;
      if ((h % 3u) == 0u)      { s3 += x; }
      else if ((h & 1u) != 0u) { si += x; }
      else                     { sp += x; }
    }
    kd = (ok != 0u) ? (100.0f / h1) : 0.0f;
    Nome("DHT ", CANAL_N[s], " (", EST_NOME[0], ")(%)", FIM);
    ColFs(ok, sqrtf((float)st) * kd, 3u);
    Nome((tensao != 0u) ? ("Tens" U_AT "o ") : "Corrente ", FASE_N[s % 3u], " (fundamental) (",
         EST_NOME[0], (tensao != 0u) ? ")(V)" : ")(A)", FIM);
    ColF(h1, 3u);
    Nome("DHT ", CANAL_N[s], " Par (", EST_NOME[0], ")(%)", FIM);
    ColFs(ok, sqrtf((float)sp) * kd, 3u);
    Nome("DHT ", CANAL_N[s], " Impar (", EST_NOME[0], ")(%)", FIM);
    ColFs(ok, sqrtf((float)si) * kd, 3u);
    Nome("DHT ", CANAL_N[s], " Tripla (", EST_NOME[0], ")(%)", FIM);
    ColFs(ok, sqrtf((float)s3) * kd, 3u);
  }

  /* harmonicas em RMS: pares 2..50 e depois impares 3..49, por canal */
  for (s = 0u; s < N_CANAIS; s++)
  {
    const char *u = (s <= (uint32_t)SIG_VC) ? "(V)" : "(A)";
    for (h = 2u; h <= H_DHT_MAX; h += 2u)
    {
      Txt_Uint(hs, h);
      Nome("Harm" U_OC "nica (", hs, U_ORD ") ", CANAL_N[s], " (", EST_NOME[0], ") - ", hs, u, FIM);
      ColF(sqrtf((float)(a->h2[s][h] / na)), 4u);
    }
    for (h = 3u; h <= H_DHT_MAX; h += 2u)
    {
      Txt_Uint(hs, h);
      Nome("Harm" U_OC "nica (", hs, U_ORD ") ", CANAL_N[s], " (", EST_NOME[0], ") - ", hs, u, FIM);
      ColF(sqrtf((float)(a->h2[s][h] / na)), 4u);
    }
  }

  /* desequilibrio |X2|/|X1| (componentes simetricas da fundamental) */
  for (e = 0u; e < 3u; e++)
  {
    Nome("Deseq. Tens" U_AT "o (", EST_NOME[e], ")(%)", FIM);
    ColFs((a->deseq_v.n > 0u) ? 1u : 0u, Agg_Valor(&a->deseq_v, e, 1u), 3u);
  }
  for (e = 0u; e < 3u; e++)
  {
    Nome("Deseq. Corrente (", EST_NOME[e], ")(%)", FIM);
    ColFs((a->deseq_i.n > 0u) ? 1u : 0u, Agg_Valor(&a->deseq_i, e, 1u), 3u);
  }
  for (e = 0u; e < 3u; e++)                     /* nao medida */
  {
    Nome("Tens" U_AT "o Neutro (", EST_NOME[e], ")(V)", FIM);
    ColVazio();
  }
  for (e = 0u; e < 3u; e++)
  {
    Nome("Corr. N. Calculada (", EST_NOME[e], ")(A)", FIM);
    ColF(Agg_Valor(&a->i_n, e, 1u), 3u);
  }
  for (p = 0u; p < 3u; p++)                     /* flicker nao medido */
  {
    Nome("Pst ", FASE_N[p], " (", EST_NOME[0], ")(pu)", FIM);  ColVazio();
    Nome("Plt ", FASE_N[p], " (", EST_NOME[0], ")(pu)", FIM);  ColVazio();
  }
  for (p = 0u; p < 4u; p++)
  {
    Nome("Demanda ", (p < 3u) ? FASE_N[p] : ("Trif" U_AA "sico"), " Cons. (", EST_NOME[0], ")(kW)", FIM);
    ColF(Demanda_Valor(p, a->t_ult) * k, 5u);
  }
  for (p = 0u; p < 3u; p++)
  {
    Nome("Tensao ", LINHA_N[p], " (", EST_NOME[0], ")(V)", FIM);
    ColF(sqrtf((float)(a->vl2[p] / na)), 3u);
  }
}

static void Harm_Cabecalho(void)
{
  Col_Inicio(1u);
  W_Texto("\xEF\xBB\xBF");                     /* BOM: Excel le os acentos */
  Harm_Colunas();
  W_Fim();
}

static void Harm_Linha(void)
{
  Col_Inicio(0u);
  Harm_Colunas();
  W_Fim();
}

/* =============================================================================
 *  7b) RELOGIO E AGREGACAO DO INTERVALO DE REGISTRO (HARMON.CSV)
 *      - RMS de tensao, corrente, harmonicas, neutro, linha e desequilibrio:
 *        raiz da media dos quadrados das capturas (IEC 61000-4-30);
 *      - min/max de tensao e corrente: RMS de cada janela de 12 ciclos;
 *      - potencias e frequencia: media aritmetica; min/max por captura;
 *      - FP medio = P medio / S(a) medio; min/max = menor/maior FP de captura;
 *      - DHT = raiz(soma h^2)/h1 sobre as harmonicas ja agregadas;
 *      - energia = potencia media x duracao do intervalo;
 *      - demanda = media movel de DEMANDA_MIN minutos da P consumida.
 * ============================================================================= */
static uint64_t Tempo_ms(void)                 /* HAL_GetTick em 64 bits      */
{
  const uint32_t t = HAL_GetTick();
  if (t < tick_ult) { tick_voltas++; }         /* volta a cada 49,7 dias      */
  tick_ult = t;
  return ((uint64_t)tick_voltas << 32) | (uint64_t)t;
}

static void Agg_Soma(Agg_t *a, float x)        /* media aritmetica            */
{
  a->soma += (double)x;
  if ((a->n == 0u) || (x < a->min)) { a->min = x; }
  if ((a->n == 0u) || (x > a->max)) { a->max = x; }
  a->n++;
}

static void Agg_SomaQ(Agg_t *a, float x, float xmin, float xmax)  /* quadratica */
{
  a->soma += (double)x * (double)x;
  if ((a->n == 0u) || (xmin < a->min)) { a->min = xmin; }
  if ((a->n == 0u) || (xmax > a->max)) { a->max = xmax; }
  a->n++;
}

static float Agg_Valor(const Agg_t *a, uint32_t e, uint8_t quadratica)  /* e: 0 medio, 1 min, 2 max */
{
  double m;
  if (a->n == 0u) { return 0.0f; }
  if (e == 1u)    { return a->min; }
  if (e == 2u)    { return a->max; }
  m = a->soma / (double)a->n;
  return (quadratica != 0u) ? sqrtf((float)m) : (float)m;
}

static void AggFP_Soma(AggFP_t *a, float fp, char tipo)
{
  if ((a->n == 0u) || (fp < a->min)) { a->min = fp; a->tmin = tipo; }
  if ((a->n == 0u) || (fp > a->max)) { a->max = fp; a->tmax = tipo; }
  a->n++;
}

/* Soma a captura atual ao intervalo em andamento (indice 3 = trifasico) */
static void Acc_Adicionar(void)
{
  Acc_t   *a = &acc;
  float    pc[4];
  uint32_t s, p, h;

  a->n++;
  a->t_ult = t_medicao_ms;
  for (s = 0u; s < N_CANAIS; s++)
  {
    Agg_SomaQ(&a->rms[s], estat[s].rms, estat[s].rms_min, estat[s].rms_max);
    for (h = 1u; h <= H_DHT_MAX; h++)
    {
      a->h2[s][h] += (double)harm[s][h] * (double)harm[s][h];
    }
  }
  Agg_Soma(&a->f, f_medida);

  for (p = 0u; p < 4u; p++)
  {
    const float   P    = (p < 3u) ? pot[p].p : cap.p_tri;
#if (Q_HARMON_NAO_ATIVA != 0)
    const float   Q    = (p < 3u) ? pot[p].n : cap.n_tri;
#else
    const float   Q    = (p < 3u) ? pot[p].q1 : cap.q_tri;
#endif
    const float   S    = (p < 3u) ? pot[p].s : cap.s_tri;
    const uint8_t cons = (P >= 0.0f) ? 1u : 0u;
    const uint8_t ind  = (Q >= 0.0f) ? 1u : 0u;

    pc[p] = (cons != 0u) ? P : 0.0f;
    Agg_Soma(&a->pc[p], pc[p]);
    Agg_Soma(&a->pi[p], (cons != 0u) ? 0.0f : -P);
    Agg_Soma(&a->pt[p], P);
    Agg_Soma(&a->qic[p], ((ind != 0u) && (cons != 0u)) ?  Q : 0.0f);
    Agg_Soma(&a->qii[p], ((ind != 0u) && (cons == 0u)) ?  Q : 0.0f);
    Agg_Soma(&a->qcc[p], ((ind == 0u) && (cons != 0u)) ? -Q : 0.0f);
    Agg_Soma(&a->qci[p], ((ind == 0u) && (cons == 0u)) ? -Q : 0.0f);
    Agg_Soma(&a->qt[p], Q);
    Agg_Soma(&a->sa[p], S);
    if (cap.fp_ok[p] != 0u)
    {
      AggFP_Soma(&a->fp[p], (p < 3u) ? pot[p].fp : cap.fp_tri, (ind != 0u) ? 'L' : 'C');
    }
  }

  if (cap.deseq_v_ok != 0u) { Agg_SomaQ(&a->deseq_v, cap.deseq_v, cap.deseq_v, cap.deseq_v); }
  if (cap.deseq_i_ok != 0u) { Agg_SomaQ(&a->deseq_i, cap.deseq_i, cap.deseq_i, cap.deseq_i); }
  Agg_SomaQ(&a->i_n, cap.i_neutro, cap.i_neutro, cap.i_neutro);
  for (p = 0u; p < 3u; p++)
  {
    a->vl2[p] += (double)cap.v_linha[p] * (double)cap.v_linha[p];
  }
  Demanda_Adicionar(t_medicao_ms, pc);
}

static void Demanda_Adicionar(uint64_t t_ms, const float *pc)
{
  const uint32_t m = (uint32_t)(t_ms / 60000u);
  const uint32_t b = m % DEMANDA_MIN;
  uint32_t i;

  if ((dem.usado[b] == 0u) || (dem.minuto[b] != m))   /* novo minuto */
  {
    dem.usado[b]  = 1u;
    dem.minuto[b] = m;
    dem.n[b]      = 0u;
    for (i = 0u; i < 4u; i++) { dem.soma[b][i] = 0.0; }
  }
  for (i = 0u; i < 4u; i++) { dem.soma[b][i] += (double)pc[i]; }
  dem.n[b]++;
}

static float Demanda_Valor(uint32_t i, uint64_t t_ms)   /* W */
{
  const uint32_t m = (uint32_t)(t_ms / 60000u);
  double   soma = 0.0;
  uint32_t n = 0u, b;

  for (b = 0u; b < DEMANDA_MIN; b++)
  {
    if ((dem.usado[b] != 0u) && (dem.minuto[b] <= m) && ((m - dem.minuto[b]) < DEMANDA_MIN))
    {
      soma += dem.soma[b][i];
      n    += dem.n[b];
    }
  }
  return (n > 0u) ? (float)(soma / (double)n) : 0.0f;
}

/* =============================================================================
 *  8) HARDWARE: conferencia do CubeMX, TIM3, aquisicao e LEDs
 * ============================================================================= */
static ADC_TypeDef *ADC_Instancia(uint32_t k)
{
  return (k == 0u) ? ADC1 : ((k == 1u) ? ADC2 : ADC3);
}

static int32_t Canal_ParaSinal(uint32_t canal)
{
  switch (canal)
  {
    case 11u: return SIG_VA;     /* PC1 */
    case 12u: return SIG_VB;     /* PC2 */
    case 13u: return SIG_VC;     /* PC3 */
    case 1u:  return SIG_IA;     /* PA1 */
    case 2u:  return SIG_IB;     /* PA2 */
    case 3u:  return SIG_IC;     /* PA3 */
    default:  return -1;
  }
}

static uint32_t Ciclos_Amostragem(uint32_t smp)
{
  switch (smp & 7u)
  {
    case 0u: return 3u;
    case 1u: return 15u;
    case 2u: return 28u;
    case 3u: return 56u;
    case 4u: return 84u;
    case 5u: return 112u;
    case 6u: return 144u;
    default: return 480u;
  }
}

/* Confere o que foi configurado no CubeMX e monta o mapa do buffer do DMA.
 * DMA modo 1 no triplo: por gatilho vem ADC1,ADC2,ADC3 do rank 1 e depois
 * ADC1,ADC2,ADC3 do rank 2 -> posicao = rank*3 + indice do ADC.             */
static uint32_t Config_Verificar(void)
{
  uint32_t k, r, usados = 0u;

  if ((ADC123_COMMON->CCR & ADC_CCR_MULTI) != ADC_TRIPLEMODE_REGSIMULT) { return ERRO_CFG_MULTIMODO; }
  if ((ADC123_COMMON->CCR & ADC_CCR_DMA) != ADC_DMAACCESSMODE_1)        { return ERRO_CFG_DMA_MODO; }
  if ((hadc1.DMA_Handle == NULL) ||
      (hadc1.DMA_Handle->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) ||
      (hadc1.DMA_Handle->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD))   { return ERRO_CFG_DMA_LARGURA; }
#if SOBREAMOSTRAGEM
  if (((ADC1->CR2 & ADC_CR2_EXTSEL) != ADC_EXTERNALTRIGCONV_T8_TRGO) ||
      ((ADC1->CR2 & ADC_CR2_EXTEN) == 0u))                                  { return ERRO_CFG_GATILHO; }
  if ((hadc1.DMA_Handle->Init.Mode != DMA_CIRCULAR) ||
      (hadc1.Init.DMAContinuousRequests != ENABLE))                         { return ERRO_CFG_DMA_MODO; }
#else
  if (((ADC1->CR2 & ADC_CR2_EXTSEL) != ADC_EXTERNALTRIGCONV_T3_TRGO) ||
      ((ADC1->CR2 & ADC_CR2_EXTEN) == 0u))                                  { return ERRO_CFG_GATILHO; }
#endif

  for (k = 0u; k < 3u; k++)
  {
    ADC_TypeDef *adc = ADC_Instancia(k);
    if ((((adc->SQR1 & ADC_SQR1_L) >> ADC_SQR1_L_Pos) + 1u) != 2u) { return ERRO_CFG_N_CONV; }
    for (r = 0u; r < 2u; r++)
    {
      const int32_t s = Canal_ParaSinal((adc->SQR3 >> (5u * r)) & 0x1Fu);
      if ((s < 0) || ((usados & (1u << (uint32_t)s)) != 0u)) { return ERRO_CFG_CANAIS; }
      usados |= (1u << (uint32_t)s);
      pos_sinal[s]  = (uint8_t)(r * 3u + k);
      rank_sinal[s] = (uint8_t)r;
    }
  }

  /* intervalo entre a amostra do rank 1 e a do rank 2 = (Tsample_rank2 + 12) ciclos */
  {
    const uint32_t ch2 = (ADC1->SQR3 >> 5u) & 0x1Fu;
    const uint32_t smp = (ch2 < 10u) ? (ADC1->SMPR2 >> (3u * ch2))
                                     : (ADC1->SMPR1 >> (3u * (ch2 - 10u)));
    const uint32_t div = 2u * (((ADC123_COMMON->CCR & ADC_CCR_ADCPRE) >> ADC_CCR_ADCPRE_Pos) + 1u);
    const float fadc = (float)HAL_RCC_GetPCLK2Freq() / (float)div;
    atraso_rank_s = (float)(Ciclos_Amostragem(smp) + 12u) / fadc;
  }
  return 0u;
}

static uint32_t TIM3_Clock(void)
{
  const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? pclk1 : (2u * pclk1);
}

/* Programa o TIM3 para a taxa fs (Hz) e devolve a taxa real obtida.
 * Com 84 MHz: ARR ~ 10937 -> resolucao de 0,009 % (norma: 0,03 %).          */
static float TIM3_ProgramarTaxa(float fs)
{
  const float    clk = (float)TIM3_Clock();
  const uint32_t div = (uint32_t)(clk / fs + 0.5f);
  const uint32_t psc = (div - 1u) / 65536u;
  const uint32_t arr = (uint32_t)(clk / ((float)(psc + 1u) * fs) + 0.5f) - 1u;

  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->PSC  = psc;
  TIM3->ARR  = arr;
  TIM3->CR2  = (TIM3->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;   /* TRGO = update */
  TIM3->EGR  = TIM_EGR_UG;                                    /* carrega o PSC */
  TIM3->CNT  = 0u;
  TIM3->SR   = 0u;
  htim3.Init.Prescaler = psc;
  htim3.Init.Period    = arr;
  return clk / ((float)(psc + 1u) * (float)(arr + 1u));
}

#if SOBREAMOSTRAGEM
/* -----------------------------------------------------------------------------
 *  8b) SOBREAMOSTRAGEM: TIM8, configuracao do ADC/DMA e filtro FIR decimador
 * --------------------------------------------------------------------------- */
static uint32_t TIM8_Clock(void)
{
  const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1) ? pclk2 : (2u * pclk2);
}

/* Igual ao TIM3_ProgramarTaxa, para o TIM8 (168 MHz: ARR ~ 2734 a 61,4 kHz). */
static float TIM8_ProgramarTaxa(float fs)
{
  const float    clk = (float)TIM8_Clock();
  const uint32_t div = (uint32_t)(clk / fs + 0.5f);
  const uint32_t psc = (div - 1u) / 65536u;
  const uint32_t arr = (uint32_t)(clk / ((float)(psc + 1u) * fs) + 0.5f) - 1u;

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->PSC  = psc;
  TIM8->ARR  = arr;
  TIM8->RCR  = 0u;
  TIM8->CR2  = (TIM8->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;   /* TRGO = update */
  TIM8->EGR  = TIM_EGR_UG;
  TIM8->CNT  = 0u;
  TIM8->SR   = 0u;
  return clk / ((float)(psc + 1u) * (float)(arr + 1u));
}

/* Reprograma, sem mexer no CubeMX: gatilho do ADC1 = TIM8 TRGO, 144 ciclos de
 * amostragem nos 6 canais e DMA em modo circular. Chamar antes de
 * Config_Verificar (que recalcula o atraso entre ranks: 156 ciclos = 7,4 us). */
static void OS_ConfigurarADC(void)
{
  uint32_t k, r;

  __HAL_RCC_TIM8_CLK_ENABLE();
#if OS_LINHA_NA_CCM
  __HAL_RCC_CCMDATARAMEN_CLK_ENABLE();       /* ja vem ligado apos o reset */
#endif
  TIM8->CR1 = 0u;

  ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_EXTSEL) | ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;

  for (k = 0u; k < 3u; k++)
  {
    ADC_TypeDef *adc = ADC_Instancia(k);
    for (r = 0u; r < 2u; r++)
    {
      const uint32_t ch = (adc->SQR3 >> (5u * r)) & 0x1Fu;
      if (ch < 10u)
      {
        adc->SMPR2 = (adc->SMPR2 & ~(7u << (3u * ch))) | ((uint32_t)OS_SMP << (3u * ch));
      }
      else
      {
        adc->SMPR1 = (adc->SMPR1 & ~(7u << (3u * (ch - 10u)))) | ((uint32_t)OS_SMP << (3u * (ch - 10u)));
      }
    }
  }

  hadc1.DMA_Handle->Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(hadc1.DMA_Handle) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Taxa de saida (128 x f): o TIM8 e programado OS_FATOR vezes mais rapido. */
static float Gatilho_ProgramarTaxa(float fs)
{
  return TIM8_ProgramarTaxa(fs * (float)OS_FATOR) / (float)OS_FATOR;
}

static void OS_Reiniciar(void)
{
  uint32_t k;
  os_pos    = 0u;
  os_n      = 0u;
  os_saidas = 0u;
  for (k = 0u; k < N_CANAIS; k++) { os_sat[k] = 0u; }
}

/* O tempo de 0,5 ms citado abaixo so vale com otimizacao. Compilado em -O0
 * (configuracao Debug do CubeIDE) o calculo passa de 2,1 ms: a interrupcao
 * nao acompanha o DMA, metades do buffer sao sobrescritas antes de lidas
 * (amostras fora de ordem no tempo) e o SysTick para durante a aquisicao.
 * Por isso o nucleo e sempre compilado com O3, qualquer que seja a
 * configuracao do projeto, e os callbacks conferem o atraso (secao 11).   */
#pragma GCC push_options
#pragma GCC optimize ("O3")
/* OS_NUCLEO_INICIO */
/* Recebe meia transferencia do DMA (OS_BLOCO x OS_FATOR quadros de 6 amostras
 * na ordem do DMA), empurra cada amostra na linha de atraso do seu canal e, a
 * cada OS_FATOR quadros, calcula a saida do FIR e grava no adc_buf.
 * A linha tem 2 x FIR_N posicoes: cada amostra e escrita em p e em p + FIR_N,
 * de modo que as FIR_N ultimas ficam sempre contiguas a partir de os_pos
 * (mais antiga em [os_pos], mais nova em [os_pos + FIR_N - 1]).
 * Custo: 91 multiplicacoes por saida e canal (FIR simetrico) -> cerca de
 * 0,5 ms por meia transferencia (2,1 ms), executado na interrupcao do DMA.  */
static void OS_ProcessarMeia(const uint16_t *q)
{
  uint32_t f, k, i;

  for (f = 0u; f < (OS_BLOCO * OS_FATOR); f++, q += N_CANAIS)
  {
    if (os_saidas >= AMOSTRAS_CANAL) { return; }

    for (k = 0u; k < N_CANAIS; k++)
    {
      const uint16_t x = q[k];
      if ((x == 0u) || (x >= 4095u)) { os_sat[k] = 1u; }
      OS_LINHA[k][os_pos]         = (float)x;
      OS_LINHA[k][os_pos + FIR_N] = (float)x;
    }
    os_pos = (os_pos + 1u < FIR_N) ? (os_pos + 1u) : 0u;
    os_n++;

    if ((os_n >= FIR_N) && (((os_n - FIR_N) % OS_FATOR) == 0u))
    {
      uint16_t *dst = &adc_buf[os_saidas * N_CANAIS];
      for (k = 0u; k < N_CANAIS; k++)
      {
        const float *w = &OS_LINHA[k][os_pos];
        float y = FIR_C[FIR_MEIO] * w[FIR_MEIO];
        for (i = 0u; i < FIR_MEIO; i++)
        {
          y += FIR_C[i] * (w[i] + w[FIR_N - 1u - i]);
        }
        y = y * OS_ESCALA + 0.5f;
        dst[k] = (y <= 0.0f) ? 0u : ((y >= 65535.0f) ? 65535u : (uint16_t)y);
      }
      os_saidas++;
      if (os_saidas >= AMOSTRAS_CANAL)
      {
        TIM8->CR1 &= ~TIM_CR1_CEN;          /* 60 ciclos completos            */
        acq_concluida = 1u;
        return;
      }
    }
  }
}
/* OS_NUCLEO_FIM */
#pragma GCC pop_options

/* Confere, ao fim do processamento de uma metade, se a interrupcao ficou para
 * tras. O DMA deve estar enchendo a OUTRA metade: NDTR em (OS_MEIA_LEN,
 * OS_DMA_LEN] = 1a metade; NDTR em [0, OS_MEIA_LEN] = 2a metade. Se ja voltou
 * para a metade que acabou de ser lida, parte dela foi sobrescrita durante a
 * leitura: a captura e descartada (acq_erro -> LEDs vermelho+laranja piscam). */
static void OS_ConfereAtraso(uint8_t metade_lida)        /* 0 = 1a, 1 = 2a */
{
  const uint32_t ndtr      = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
  const uint8_t  dma_na_2a = (ndtr <= OS_MEIA_LEN) ? 1u : 0u;

  if (dma_na_2a == metade_lida)
  {
    TIM8->CR1 &= ~TIM_CR1_CEN;
    os_atrasos++;
    acq_erro = 1u;
  }
}
#else
static float Gatilho_ProgramarTaxa(float fs)
{
  return TIM3_ProgramarTaxa(fs);
}
#endif

static HAL_StatusTypeDef Aquisicao_Iniciar(void)
{
  uint32_t modo_dma;

  acq_concluida = 0u;
  acq_erro = 0u;

  (void)HAL_ADC_Start(&hadc3);           /* escravos: so ligam (ADON)       */
  (void)HAL_ADC_Start(&hadc2);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);

  /* re-arma o modo DMA do multimodo (necessario quando DDS = 0) */
  modo_dma = ADC123_COMMON->CCR & ADC_CCR_DMA;
  ADC123_COMMON->CCR &= ~ADC_CCR_DMA;
  ADC123_COMMON->CCR |= modo_dma;

#if SOBREAMOSTRAGEM
  OS_Reiniciar();
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)os_dma, OS_DMA_LEN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  TIM8->CNT = 0u;
  t_inicio_acq   = HAL_GetTick();
  t_inicio_acq64 = Tempo_ms();
  TIM8->CR1 |= TIM_CR1_CEN;
  return HAL_OK;
#else
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  TIM3->CNT = 0u;
  t_inicio_acq   = HAL_GetTick();
  t_inicio_acq64 = Tempo_ms();
  return HAL_TIM_Base_Start(&htim3);
#endif
}

static uint8_t Aquisicao_Encerrar(void)       /* 1 = dados validos */
{
  /* RM0090 (multi ADC): um overrun DURANTE a captura faz o ADC parar de pedir
   * DMA, entao o DMA nunca completaria. Se completou, as 46080 amostras sao
   * validas; um OVR que apareca depois (gatilho extra antes de o TIM3 parar)
   * nao afeta o buffer e apenas e limpo.                                    */
  const uint8_t ok = ((acq_concluida != 0u) && (acq_erro == 0u)) ? 1u : 0u;

#if SOBREAMOSTRAGEM
  TIM8->CR1 &= ~TIM_CR1_CEN;
#else
  (void)HAL_TIM_Base_Stop(&htim3);
#endif
  (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
  __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_OVR);
  return ok;
}

static void LED_Mascara(uint16_t acesos)
{
  GPIOD->BSRR = ((uint32_t)(LED_TODOS & (uint16_t)~acesos) << 16) | (uint32_t)(acesos & LED_TODOS);
}

static void LED_Barra(uint32_t n)             /* acende n LEDs: PD12 -> PD15 */
{
  uint16_t m = 0u;
  if (n >= 1u) { m |= LED_VERDE; }
  if (n >= 2u) { m |= LED_LARANJA; }
  if (n >= 3u) { m |= LED_VERMELHO; }
  if (n >= 4u) { m |= LED_AZUL; }
  LED_Mascara(m);
}

static void Espera_ms(uint32_t ms)            /* espera sem parar o USB Host */
{
  const uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms)
  {
    MX_USB_HOST_Process();
  }
}

static void LED_Piscar(uint16_t leds, uint32_t vezes, uint32_t periodo_ms)
{
  uint32_t i;
  for (i = 0u; i < vezes; i++)
  {
    LED_Mascara(leds);
    Espera_ms(periodo_ms / 2u);
    LED_Mascara(0u);
    Espera_ms(periodo_ms / 2u);
  }
}

static void Erro_Config_Laco(uint32_t codigo) /* trava mostrando o codigo */
{
  for (;;)
  {
    LED_Mascara((uint16_t)((codigo & 0xFu) << 12));   /* bit0 = PD12 ... */
    HAL_Delay(800u);
    LED_Mascara(0u);
    HAL_Delay(300u);
  }
}

/* =============================================================================
 *  9) CARTAO SD / PENDRIVE (USB Host MSC + FatFs)
 *     Cada gravacao abre, anexa e fecha o arquivo: tolera remocao do cartao
 *     e queda de energia sem corromper o que ja foi gravado.
 *     Na 1a montagem apos ligar, HARMON.CSV e BRUTO.CSV de um ensaio anterior
 *     sao renomeados (ARQUIVO_NOVO_AO_LIGAR): cada ensaio comeca um arquivo
 *     novo, com o relogio em 00:00:00.
 * ============================================================================= */
static void Midia_Servico(void)
{
  if (Appli_state == APPLICATION_READY)
  {
    if ((midia_ok == 0u) && ((HAL_GetTick() - t_montagem) >= 1000u))
    {
      t_montagem = HAL_GetTick();
      if (f_mount(&USBHFatFS, (TCHAR const *)USBHPath, 1) == FR_OK)
      {
        midia_ok = 1u;
        if ((sessao_iniciada == 0u) && (ARQUIVO_NOVO_AO_LIGAR != 0))   /* 1a montagem */
        {
          Arquivo_Arquivar(ARQ_HARM,  "HARM");
          Arquivo_Arquivar(ARQ_BRUTO, "BRUT");
        }
        sessao_iniciada = 1u;
      }
    }
  }
  else if (midia_ok != 0u)
  {
    (void)f_mount(NULL, (TCHAR const *)USBHPath, 0);
    midia_ok = 0u;
  }
}

static uint8_t Caminho(char *dst, size_t tam, const char *nome)  /* "0:/NOME" */
{
  const size_t n = strlen(USBHPath);

  if ((n + strlen(nome)) >= tam) { return 0u; }
  memcpy(dst, USBHPath, n);
  strcpy(&dst[n], nome);
  return 1u;
}

/* Renomeia o arquivo de um ensaio anterior para PREF_nnn.CSV (001..999) */
static void Arquivo_Arquivar(const char *nome, const char *prefixo)
{
  char     de[24], para[24], novo[13];
  FILINFO  fi;
  uint32_t n;

  if ((Caminho(de, sizeof(de), nome) == 0u) || (f_stat(de, &fi) != FR_OK)) { return; }
  memcpy(novo, prefixo, 4u);                       /* "HARM_001.CSV"         */
  novo[4] = '_';
  memcpy(&novo[8], ".CSV", 5u);
  for (n = 1u; n <= 999u; n++)
  {
    novo[5] = (char)('0' + (n / 100u));
    novo[6] = (char)('0' + ((n / 10u) % 10u));
    novo[7] = (char)('0' + (n % 10u));
    if (Caminho(para, sizeof(para), novo) == 0u) { return; }
    if (f_stat(para, &fi) == FR_NO_FILE)
    {
      (void)f_rename(de, para);
      return;
    }
  }
}

static FRESULT Log_Anexar(const char *nome, void (*cabecalho)(void), void (*linhas)(void))
{
  char caminho[24];
  FRESULT fr, fc;

  if (Caminho(caminho, sizeof(caminho), nome) == 0u) { return FR_INVALID_NAME; }

  fr = f_open(&USBHFile, caminho, FA_OPEN_ALWAYS | FA_WRITE);
  if (fr != FR_OK) { return fr; }

  fr = f_lseek(&USBHFile, f_size(&USBHFile));     /* posiciona no fim      */
  if (fr == FR_OK)
  {
    W_Inicia(&USBHFile);
    if (f_size(&USBHFile) == 0u) { cabecalho(); } /* arquivo novo          */
    linhas();
    W_Descarrega();
    fr = wres;
  }
  fc = f_close(&USBHFile);
  return (fr != FR_OK) ? fr : fc;
}

/* Chamada uma vez por captura aceita. Se a captura ja pertence a outro
 * intervalo, grava antes a linha do intervalo que terminou (HARMON.CSV);
 * depois soma a captura ao intervalo atual e grava a linha do BRUTO.CSV.
 * Devolve 0 se alguma gravacao falhou.                                   */
static uint8_t Registro_Gravar(void)
{
  const uint64_t idx   = t_medicao_ms / (uint64_t)intervalo_ms;
  const uint8_t  fecha = ((acc.n > 0u) &&
                          ((idx != acc.idx) || (intervalo_ms != acc.intervalo_ms))) ? 1u : 0u;
  uint8_t ok = 1u;

  if (fecha != 0u)
  {
    if ((midia_ok != 0u) && (Log_Anexar(ARQ_HARM, Harm_Cabecalho, Harm_Linha) != FR_OK))
    {
      ok = 0u;
    }
    memset(&acc, 0, sizeof(acc));
  }
  if (acc.n == 0u)
  {
    acc.idx          = idx;
    acc.intervalo_ms = intervalo_ms;
  }
  Acc_Adicionar();

  if ((midia_ok != 0u) && (ok != 0u) && ((BRUTO_TODAS_CAPTURAS != 0) || (acc.n == 1u)))
  {
    if (Log_Anexar(ARQ_BRUTO, Bruto_Cabecalho, Bruto_Linha) != FR_OK) { ok = 0u; }
  }
  return ok;
}

/* =============================================================================
 *  10) APLICACAO (maquina de estados chamada no while(1))
 * ============================================================================= */
static void App_Iniciar(void)
{
  uint32_t k;

  LED_Mascara(LED_TODOS);                    /* teste dos LEDs            */
  HAL_Delay(300u);
  for (k = 1u; k <= 4u; k++)
  {
    LED_Barra(k);
    HAL_Delay(120u);
  }
  LED_Mascara(0u);

  /* RM0090 (multi ADC mode): o gatilho externo deve ficar so no mestre;
   * nos escravos ele e desligado para evitar conversoes espurias.        */
  ADC2->CR2 &= ~ADC_CR2_EXTEN;
  ADC3->CR2 &= ~ADC_CR2_EXTEN;

#if SOBREAMOSTRAGEM
  OS_ConfigurarADC();                        /* TIM8, 144 ciclos, DMA circular */
#endif
  erro_config = Config_Verificar();
  if (erro_config != 0u)
  {
    Erro_Config_Laco(erro_config);           /* nao retorna               */
  }

  /* unidade fisica por contagem do ADC */
  ganho_sinal[SIG_VA] = GANHO_VA * (VDDA_V / ADC_FUNDO_ESCALA);
  ganho_sinal[SIG_VB] = GANHO_VB * (VDDA_V / ADC_FUNDO_ESCALA);
  ganho_sinal[SIG_VC] = GANHO_VC * (VDDA_V / ADC_FUNDO_ESCALA);
  ganho_sinal[SIG_IA] = GANHO_IA * (VDDA_V / ADC_FUNDO_ESCALA);
  ganho_sinal[SIG_IB] = GANHO_IB * (VDDA_V / ADC_FUNDO_ESCALA);
  ganho_sinal[SIG_IC] = GANHO_IC * (VDDA_V / ADC_FUNDO_ESCALA);

  fs_uso = Gatilho_ProgramarTaxa((float)N_FFT * F_NOMINAL_HZ); /* 7680 Hz */
  pll.f = F_NOMINAL_HZ;
  SogiPll_Reinicia(&pll, 0u);
  estado = EST_AGUARDA_MIDIA;
}

static void App_Tarefa(void)
{
  (void)Tempo_ms();                 /* mantem a contagem de voltas do tick */
  Midia_Servico();

  switch (estado)
  {
    case EST_AGUARDA_MIDIA:
      LED_Mascara((((HAL_GetTick() / 250u) & 1u) != 0u) ? LED_LARANJA : 0u);
      if ((midia_ok != 0u) || (MEDIR_SEM_PENDRIVE != 0))
      {
        estado = EST_INICIA_AQUISICAO;
      }
      break;

    case EST_INICIA_AQUISICAO:
      LED_Barra(1u);
      if (Aquisicao_Iniciar() == HAL_OK)
      {
        if (relogio_iniciado == 0u)         /* 1a captura = 00:00:00 do dia 1 */
        {
          t_zero_ms        = t_inicio_acq64;
          relogio_iniciado = 1u;
        }
        estado = EST_AQUISICAO;
      }
      else
      {
        (void)Aquisicao_Encerrar();
        LED_Piscar(LED_VERMELHO | LED_LARANJA, 3u, 300u);
      }
      break;

    case EST_AQUISICAO:
    {
      /* barra de progresso pelo contador do DMA (amostras ja transferidas) */
#if SOBREAMOSTRAGEM
      const uint32_t feitas = os_saidas * N_CANAIS;
#else
      const uint32_t feitas = ADC_BUF_LEN - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
#endif
      LED_Barra(1u + (4u * feitas) / ADC_BUF_LEN);

      if ((acq_concluida != 0u) || (acq_erro != 0u))
      {
        if (Aquisicao_Encerrar() != 0u)
        {
          t_medicao_ms = t_inicio_acq64 - t_zero_ms;
          estado = EST_PROCESSA;
        }
        else
        {
          LED_Piscar(LED_VERMELHO | LED_LARANJA, 3u, 300u);
          estado = EST_INICIA_AQUISICAO;
        }
      }
      else if ((HAL_GetTick() - t_inicio_acq) > TIMEOUT_AQUISICAO_MS)
      {
        (void)Aquisicao_Encerrar();                 /* sem gatilho do TIM3? */
        LED_Piscar(LED_VERMELHO | LED_LARANJA, 3u, 300u);
        estado = EST_INICIA_AQUISICAO;
      }
      break;
    }

    case EST_PROCESSA:
    {
      const float fs_capturada = fs_uso;

      LED_Mascara(LED_TODOS);
      Estatisticas_Calcular();      /* offsets, RMS, max/min, P, S, linha */
      Pll_Processar();              /* SOGI-PLL -> f_medida, pll_travado  */
      Harmonicos_Calcular();        /* 5 janelas x 3 FFTs de 128 pontos    */
      IEC_Avaliar();                /* THD, PWHD, limites                  */
      Grandezas_Calcular();         /* angulos, desequilibrio, DHT, totais */
      cap.fs = fs_capturada;

      /* sincronismo IEC 61000-4-7: fs deve ser 128*f com erro <= 0,03 % */
      if (pll_travado != 0u)
      {
        erro_sinc = fabsf(fs_capturada / ((float)N_FFT * f_medida) - 1.0f);
        sinc_ok   = (erro_sinc <= IEC_ERRO_SINC_MAX) ? 1u : 0u;
        fs_uso    = Gatilho_ProgramarTaxa((float)N_FFT * f_medida);   /* proxima */
      }
      else
      {
        erro_sinc = 0.0f;
        sinc_ok   = 0u;
        fs_uso    = Gatilho_ProgramarTaxa((float)N_FFT * F_NOMINAL_HZ);
      }

      if ((pll_travado != 0u) && (sinc_ok == 0u) && (tentativas_sinc < MAX_TENTATIVAS_SINC))
      {
        tentativas_sinc++;                  /* descarta e recaptura ja */
        estado = EST_INICIA_AQUISICAO;      /* com a taxa corrigida    */
      }
      else
      {
        recapturas      = tentativas_sinc;
        tentativas_sinc = 0u;
        seq_medicao++;
        estado = EST_GRAVA;
      }
      break;
    }

    case EST_GRAVA:
      if (midia_ok != 0u)
      {
        LED_Mascara(LED_AZUL);
      }
      if (Registro_Gravar() == 0u)          /* acumula e grava BRUTO/HARMON */
      {
        (void)f_mount(NULL, (TCHAR const *)USBHPath, 0);
        midia_ok = 0u;
        LED_Piscar(LED_VERMELHO, 3u, 300u);
      }
      LED_Mascara(0u);
      estado = ((midia_ok != 0u) || (MEDIR_SEM_PENDRIVE != 0)) ? EST_INICIA_AQUISICAO
                                                              : EST_AGUARDA_MIDIA;
      break;

    default:
      estado = EST_AGUARDA_MIDIA;
      break;
  }
}

/* =============================================================================
 *  11) CALLBACKS DO HAL
 * ============================================================================= */
#if SOBREAMOSTRAGEM
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (acq_concluida == 0u) && (acq_erro == 0u))
  {                                    /* 1a metade do buffer circular     */
    OS_ProcessarMeia(&os_dma[0]);
    OS_ConfereAtraso(0u);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (acq_concluida == 0u) && (acq_erro == 0u))
  {                                    /* 2a metade do buffer circular     */
    OS_ProcessarMeia(&os_dma[OS_MEIA_LEN]);
    OS_ConfereAtraso(1u);
  }
}
#else
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)          /* DMA transferiu as 46080 amostras */
  {
    TIM3->CR1 &= ~TIM_CR1_CEN;         /* para os gatilhos imediatamente   */
    acq_concluida = 1u;
  }
}
#endif

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (acq_concluida == 0u))  /* erro DURANTE a captura */
  {
#if SOBREAMOSTRAGEM
    TIM8->CR1 &= ~TIM_CR1_CEN;
#else
    TIM3->CR1 &= ~TIM_CR1_CEN;
#endif
    acq_erro = 1u;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Falha de inicializacao do HAL: os 4 LEDs piscam rapidamente */
  __disable_irq();
  while (1)
  {
    volatile uint32_t t;
    GPIOD->ODR ^= LED_TODOS;
    for (t = 0u; t < 1500000u; t++) { }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
