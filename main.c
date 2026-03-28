#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"


/*
 *
 * #define ADC0_BASE ADCA_BASE é o pino AA0
 * #define DAC0_BASE DACB_BASE é o pino AA1
 *
 *
 */

// Parte de compartilhamento de memória

#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;

#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;

#pragma DATA_SECTION(adcVoltage,"Cla1ToCpuMsgRAM");
volatile float adcVoltage;

#pragma DATA_SECTION(REF,"Cla1ToCpuMsgRAM");
float REF = 8.0f;


// VREF é a tensão de referência do DAC/ADC

#define norm_DAC 4095.0f/12.0f

#define norm_DAC_il 4095.0f/1.2f
//#define norm_ADC  18.0f/4095.0F

// varaveis criadas para  PWM
uint32_t ePwm_TimeBase;
uint32_t ePwm_MinDuty;
uint32_t ePwm_MaxDuty;
uint32_t ePwm_curDuty;

volatile uint32_t cmp_Value;
//
// Definições de Constantes
//
#define F_PWM                  10000.0f     // Frequência de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Período de chaveamento (s)
#define DT_SIM                 0.000001f    // Passo de simulação (5 µs)
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / DT_SIM) // Passos por ciclo PWM

// Parâmetros do Conversor Buck
#define VIN                    12.0f       // Tensão de entrada (V)
#define L                      0.001f      // Indutância (H)
#define C                      0.00001f    // Capacitância (F)
#define R_LOAD                 10.0f       // Carga resistiva (Ohm)

// Constantes auxiliares (evita divisões repetidas no loop)
#define INV_L                  (DT_SIM / L)
#define INV_C                  (DT_SIM / C)
#define INV_R_LOAD             (1.0f / R_LOAD)

volatile float32_t g_vout_sim = 0.0f;        // Tensão de saída simulada
volatile float32_t g_il_sim = 0.0f;          // Corrente no indutor simulada
volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM
volatile bool g_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g_new_step_ready = false;      // Flag para novo passo de simulação
volatile float g_duty_cycle = 0.5f;          // Razão cíclica (entre 0 e 1)

uint16_t dacVal,dacVal_il;

void main(void)
{
   // uint16_t dacVal,dacVal_il;
    float32_t v_l, i_c;
    // Inicialização dos periféricos

    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    ePwm_TimeBase = EPWM_getTimeBasePeriod(EPWM0_BASE);
    ePwm_MinDuty = (uint32_t) (0.95f * (float) ePwm_TimeBase);
    ePwm_MaxDuty = (uint32_t) (0.05f * (float) ePwm_TimeBase);

    EINT;
    ERTM;

    while (1)
    {

     //  cmp_Value = (uint32_t) (g_duty_cycle * ePwm_TimeBase);
     //  EPWM_setCounterCompareValue(EPWM0_BASE, EPWM_COUNTER_COMPARE_A, cmp_Value);
     //   ePwm_curDuty = EPWM_getCounterCompareValue(EPWM0_BASE, EPWM_COUNTER_COMPARE_A);

        if (g_new_step_ready)
        {
            g_new_step_ready = false;

            // Tensão no indutor
            v_l = g_switch_on ? (VIN - g_vout_sim) : (-g_vout_sim);

            // Corrente do capacitor
            i_c = g_il_sim - (g_vout_sim * INV_R_LOAD);

            // Atualização via método de Euler
            g_il_sim += INV_L * v_l;
            g_vout_sim += INV_C * i_c;

            if (g_vout_sim < 0.0f)
                g_vout_sim = 0.0f;

            if (g_vout_sim > VIN)
                g_vout_sim = VIN;

           dacVal = (uint16_t) ((g_vout_sim * norm_DAC));

           dacVal = (dacVal > 4095) ? 4095 :  dacVal;

           DAC_setShadowValue(DAC0_BASE, dacVal);



            dacVal_il = (uint16_t) ((g_il_sim * norm_DAC_il));

          // dacVal_il = (uint16_t) ((dacVal*INV_R_LOAD));


            dacVal_il = (dacVal_il > 4095) ? 4095 :  dacVal_il;


            DAC_setShadowValue(DAC1_BASE, dacVal_il);


        }
    }
}

// Interrupção externa (XINT1 ou outro XINT ligado ao GPIO que recebe o PWM)
__interrupt void INT_myGPIO0_XINT_ISR(void)
{
    g_switch_on = GPIO_readPin(myGPIO0);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE)
        g_step_counter = 0;

    // Sinaliza para o loop principal que deve simular o próximo passo
    g_new_step_ready = true;

    // Libera nova interrupção
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}
/// Falta desligar o trip zone
