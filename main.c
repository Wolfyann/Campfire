//******************************************************************************
//   MSP430G2xx3 Campfire with  Microcontroler
//
//   Description: LED Controled by PWM to simulate a campfire
//   Built with CCS V12.x - ESIGELEC 07 Nov. 2022 - Yann DUCHEMIN
// R. 02 / 12 / 2022
//******************************************************************************
/*                       _____________
 *                      / MSP430G2553 \
 *                 |  1 |Vcc  |_| Vss | 20 |
 *           P1.0  |  2 |             | 19 | P2.6
 *           P1.1  |  3 |             | 18 | P2.7
 * TA0.1-(Y) P1.2  |  4 |             | 17 | Tst
 * [button]>-P1.3  |  5 | +Rt         | 16 | !RST
 *           P1.4  |  6 |             | 15 | P1.7-<[LDR-A7]
 *           P1.5  |  7 |             | 14 | P1.6
 *           P2.0  |  8 |             | 13 | P2.5  (R)-TA1.2
 * TA1.1-(Y) P2.1  |  9 |             | 12 | P2.4
 *           P2.2  | 10 |             | 11 | P2.3  (f)-TA1.0
 *                      \_____________/
 * ---------------------------------------------------------------------------
 */

#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>

/*
 * LED on Timer TA1.x
 */
#define     YELLOW          BIT1        // R16
#define     RED             BIT5        // R24
#define     SPARK           BIT2

/*
 * Application
 */
#define     CAMPFIRE       (RED | YELLOW)


#define     PERIOD          2048
#define     PERIOD_Y        6144
#define     PERIOD_R        1024

#define     STEP            16

#define     PERIOD2         (PERIOD<<1)
#define     STEP2           (STEP<<2)

#define     LDR             0x07
/*
 * Generic defines
 */
#define     TRUE            1
#define     FALSE           0

/*
 * Prototypes
 */
void init_BOARD(void);
void initTimer(void);
void init_ADC( void );
unsigned int ReadADC( unsigned int );
unsigned int LowPassReadADC( unsigned int );


/*
 * Global variables
 */
uint8_t  sensY      = TRUE;
uint8_t  sensY2     = TRUE;
uint8_t  sensR      = TRUE;
uint8_t  mode       = TRUE;
uint8_t  start      = TRUE;
uint16_t randNumber = FALSE;
uint16_t dutyCycle  = STEP;

/* ----------------------------------------------------------------------------
 * Description: Main board initialisation
 * Input: /
 * Output: /
 */
void init_BOARD( void )
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = 0 | (WDTPW | WDTHOLD);

    if( (CALBC1_1MHZ == 0xFF) || (CALDCO_1MHZ == 0xFF) )
    {
        __bis_SR_register(LPM4_bits);  // #trap
    }
    else
    {
        // Factory Set.
        BCSCTL1 = 0 | (CALBC1_1MHZ);
        DCOCTL  = 0 | (CALDCO_1MHZ);
    }

    //---------------------------------------------------- Safety
    P1SEL  = 0x00;        // GPIO
    P1SEL2 = 0x00;        // GPIO
    P2SEL  = 0x00;        // GPIO
    P2SEL2 = 0x00;        // GPIO
    P3SEL  = 0x00;        // GPIO
    P3SEL2 = 0x00;        // GPIO
    P1DIR = 0x00;         // IN
    P2DIR = 0x00;         // IN
    P3DIR = 0x00;         // IN

    //---------------------------------------------------- Timer Output Initialisation
    P2SEL  |=  CAMPFIRE;
    P2SEL2 &= ~CAMPFIRE;
    P2DIR  |=  CAMPFIRE;

    P1SEL |= SPARK;
    P1SEL2 &= ~SPARK;
    P1DIR |= SPARK;

    //---------------------------------------------------- Button S3 (GPIO P1.3)
    P1SEL  &= ~BIT3;
    P1SEL2 &= ~BIT3;
    P1DIR  &=~BIT3;      // P1.3 as input
    P1REN  |= BIT3;      // Pulling resistor request
    P1OUT  |= BIT3;      // Pull-Up activation
    P1IES  &=~BIT3;      // Edge Detection (rise)
    P1IE   |= BIT3;      // Interrupt request on P1.3
    P1IFG &= ~BIT3;
}

/* ----------------------------------------------------------------------------
 * Description: MSP430G2553' Timer init
 * Input: /
 * Output: /
 */
void init_Timer(void)
{
    TA0CTL &= ~MC_0;
    TA0CTL = 0 | (TASSEL_2 | ID_3 | MC_1);      // source SMCLK pour TimerA , mode comptage Up 2 CCR0
    TA0CCTL1 = 0 | (OUTMOD_6 | CCIE);           // activation mode de sortie n°2
    TA0CCR0 = PERIOD;                           // determine la periode du signal
    TA0CCR1 = dutyCycle;                        // determine pas pour le rapport cyclique du signal
    TA0CTL |= TAIE;                             // activation interruption Timer

    TA1CTL &= ~MC_0;                        // Arrêt Timer
    TA1CTL = 0 | (TASSEL_2 | ID_3 | MC_3);  // source SMCLK pour TimerA , mode comptage Continu

    TA1CCTL1 = 0 | (OUTMOD_6 | CCIE);       // activation mode de sortie n°6 Toggle/Set
    TA1CCTL2 = 0 | (OUTMOD_2 | CCIE);       // activation mode de sortie n°2 Toggle/Reset

    TA1CCR0 = PERIOD;                       // determine la periode du signal
    TA1CCR1 = PERIOD_Y;                     // determine pas pour le rapport cyclique du signal
    TA1CCR2 = PERIOD_R;                     // determine pas pour le rapport cyclique du signal

    TA1CTL |= TAIE;                         //activation interruption Timer
}

/* ----------------------------------------------------------------------------
 * Description: Fonction d'initialisation de l'ADC
 * Entrees : -
 * Sorties: -
 */
void init_ADC( void )
{
  ADC10CTL0 = 0x00;
  ADC10CTL1 = 0x00;  // ADC10DISABLE
  ADC10CTL0 |= ( SREF_0 | ADC10SHT_2 | REFON | ADC10ON );
  ADC10CTL0 &= ~(ADC10SR | REF2_5V);      // 200ksps
  ADC10CTL1 |= ( ADC10DIV_0 | ADC10SSEL_3 | SHS_0 | CONSEQ_0 );
}

/* ----------------------------------------------------------------------------
 * Fonction lecture de la valeur ADC convertie
 * Entrees : numero de voie
 * Sorties: valeur convertie
 */
unsigned int ReadADC(unsigned int analog)
{
  ADC10CTL1 |= (analog * 0x1000u );
  ADC10CTL0 |= ENC | ADC10SC;
  while( !(ADC10CTL0 & ADC10IFG) || (ADC10CTL1 & ADC10BUSY) );
  ADC10CTL0 &= ~( ENC | ADC10IFG);
  ADC10CTL1 &= ~(analog * 0x1000u);
  return ADC10MEM;
}

/* ----------------------------------------------------------------------------
 * Fonction lecture de la valeur ADC convertie avec filtre passe bas
 * Entrees : numero de voie
 * Sorties: valeur convertie
 */
unsigned int LowPassReadADC(unsigned int analog)
{
  unsigned int lp = 0;
  unsigned int i = 0;
  for( i=63; i <= 1; i--)   //changement decrementation 05XI2015 ULP Recommandations
    lp += ReadADC(analog);
  return(lp>>6);
}

/* ----------------------------------------------------------------------------
 * Description: Fonction Principale
 * Input: /
 * Output: /
 */
void main(void)
{
    init_BOARD();                       // initialisation de entrées/sorties
    init_Timer();                       // initialisation des timers
//    init_ADC();                         // initialisation de l'ADC pour detection JOUR/NUIT par photoresistance
    __no_operation();                   // sync
    __enable_interrupt();               // autorisation generale des interruptions
    __bis_SR_register(LPM1_bits | GIE); // Low Power Mode
/*
   while(1){
        start = LowPassReadADC(LDR);
        (start <= 10 ) ? (start = TRUE) : (start = FALSE);
    }
*/
}

/*
 * ====================================================================
 * VECTEUR INTERRUPTION PORT 1
 *
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if( mode )         // desactivation du mode auto
    {
        mode=0;
    }
    randNumber = (TA1CCR0 & 0x0F)>>2;
    if( dutyCycle >= ( (STEP & 0x0F) << (randNumber>>1) ) )
    {
        dutyCycle >>= 1;
    }
    else
    {
        dutyCycle = PERIOD;
        (TA0CCTL1 & OUTMOD_2) ? (TA0CCTL1 |= OUTMOD_6) : (TA0CCTL1 |= OUTMOD_2);
        (TA1CCTL1 & OUTMOD_2) ? (TA1CCTL1 |= OUTMOD_6) : (TA1CCTL1 |= OUTMOD_2);
    }
    P1IFG &= ~BIT3;
}

/*
 * ====================================================================
 * VECTEUR INTERRUPTION TIMER 1
 *
 */

#pragma vector = TIMER0_A1_VECTOR                                   // Timer A Interrupt Service Routine
__interrupt void CCR_1_2_ISR_TIMER0(void)
{
  switch(TAIV)
  {
      case TA0IV_TAIFG:
        if (sensY2)                                                   // intensité positive
        {
            TA0CCR1 += dutyCycle >> randNumber;
            if( TA0CCR1 >= (PERIOD - (dutyCycle>>randNumber)) )
            {
                sensY2 = FALSE;
                (TA0CCTL1 & OUTMOD_2) ? (TA0CCTL1 |= OUTMOD_6) : (TA0CCTL1 |= OUTMOD_2);
            }
        }
        else                                                        // intensité négative
        {
            TA0CCR1 -= dutyCycle >> randNumber;
            if( TA0CCR1 <= (STEP + (dutyCycle>>randNumber)) )
               {
                   sensY2 = TRUE;
                   (TA0CCTL1 & OUTMOD_2) ? (TA0CCTL1 |= OUTMOD_6) : (TA0CCTL1 |= OUTMOD_2);
               }
        }
        TA0CTL &= ~TAIFG;
      break;

    case TA0IV_TACCR1:
          randNumber = (TA0CCR0 & 0x0F)>>2;
          TA0CCTL1 &= ~CCIFG;
      break;

    case TA0IV_TACCR2:
           TA0CCTL2 &= ~CCIFG;
      break;
  }
}

/*
 * ====================================================================
 * VECTEUR INTERRUPTION TIMER 2
 *
 */
#pragma vector = TIMER1_A1_VECTOR // Timer A Interrupt Service Routine
__interrupt void CCR_1_2_ISR_TIMER1(void)
{
  switch(TA1IV)
  {
    case TA1IV_TAIFG:

        if(sensY)
        {
            TA1CCR1 += dutyCycle; //(dutyCycle>>randNumber);
        }
        else
        {
            TA1CCR1 -= dutyCycle; //(dutyCycle>>randNumber);
        }

        if(sensR)
        {
            TA1CCR2 += STEP>>randNumber;
        }
        else
        {
            TA1CCR2 -= STEP>>randNumber;
        }

        if( (TA1CCR1 >= (PERIOD_Y - (dutyCycle+randNumber))) && (sensY) )
        {
            sensY = FALSE;
        }
        else if( (TA1CCR1 <= ( (PERIOD_Y>>3) - STEP + (dutyCycle+randNumber) )) && (!sensY) )
        {
            sensY = TRUE;
        }

        if( (TA1CCR2 >= PERIOD_R) )
        {
            sensR = FALSE;
        }
        else if( TA1CCR2 <= (PERIOD_R>>2)  )
        {
            sensR = TRUE;
        }

        TA1CTL &= ~TAIFG;
      break;

    case TA1IV_TACCR1:
          randNumber = (TA1CCR0 & 0x0F)>>2;
          if( dutyCycle >= (STEP<<1) )
          {
              dutyCycle-=((STEP>>randNumber)>>1);
          }
          else
          {
              dutyCycle = ((PERIOD>>randNumber)>>1);
          }
          (TA1CCTL1 & OUTMOD_7) ? (TA1CCTL1 |= OUTMOD_3) : (TA1CCTL1 |= OUTMOD_3);
          TA1CCTL1 &= ~CCIFG;
      break;

    case TA1IV_TACCR2:
           randNumber = (TA1CCR0 & 0x0F)>>2;
           dutyCycle += randNumber;
           (TA1CCTL1 & OUTMOD_2) ? (TA1CCTL1 |= OUTMOD_6) : (TA1CCTL1 |= OUTMOD_2); // change Yellow when Red reached his end
           TA1CCTL2 &= ~CCIFG;
      break;
  }
}
