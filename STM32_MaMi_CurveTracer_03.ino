
// 2018.11.18 Changed enc2 PINs from 19,20  ...> 15,16, because 20 is no isr-pin digitalWrite(BTN32, LOW);      
// 2018.11.26 new from Curve_MapleMi_08_TFT_CS
// 2018.11.27 ADC-Pin-names changed, ILI9341_BASE
// 2018.11.27 s_comp for compare DUT's (or cmplements) on screen
// 2018.11.28 test switch over relais              ?? ok, ADC4 free
// 2018.11.29 test switch over photo-relais AQY212 ?? ok, ADC4 free
// 2018.11.30 test current-limit with ADC3 and ADC2/ADC1 and R32/R37 = 100 Ohm, cleaning program 
// 2018.12.01 EEProm read in setup ( test only, deactivated on 2018.12.05)  
// 2018.12.05 test n-JFET with neg Vgs with ICL7662 and inv. opamp TL062 (input from MCP4822 DAC-A)
// 2018.12.12 changed TL062 with LT1013, because he works with maximal 44V (preparing for p-FET and perhaps Zener:
//            LT1013 : -13V (ICL7662) and + 26V (13V supply + ICL7662 as doppler))        
// 2018.12.14 Dacbase auf 0...2.048V wegen LT1013 an -13...+26V (other divider on + output (100k/20k -> gain = 6)
// 2018.12.14 Curve_MapleMi_14 intgrrate zener over LT1013 (PIN7) 26V and ADC4 with Rz to ground
// 2018.12.18 Curve_MapleMi_15 : test p-JFET with ScanAllNeg_pJFET and Base = 26V .... 13V
//                               integrate ScanAllNeg_PJFET in ScanAllNeg
// 2018.12.22 test diodes 0V...24V and pJFET > 50mA
// 2018.12.26 STM32_MaMi_PinTest_00: chnnge BTN1 to BTN0 (Pin0), SPI1_NSS_PIN (to Pin1), TFT_DC (to Pin2),Pin15,16 (I2C) free
//            test -3.3V on pin5 TCA0372 (instead of 0V) better in Zero-area, test spikes / cntv  
// 2019.01.05 STM32_MaMi_CurveTracer_00: copy from STM32_MaMi_PinTest_00 without scanzener ...
// 2018.01.10 STM32_MaMi_CurveTracer_01: copy from ..._ , but gain TCA0372 for I-DAC from 3 to 6 (all devices with DAC 0...2.047V , only for pJFET with 2.047...4.096 for 12 ...24V gate-Voltage)...
// 2018.01.10                            V_bat reduce to 12.5V (beacause 13.6V was to much for the ADC (with divider 100K33k) problems with pn-diodes (Uz to small) 
// 2018.01.16                            LT1013 on separate MT3608 with 26V, neg supply with LM317 (+12V) and ICL7662-board (ca -12V) 
// 2018.01.23 STM32:MaMi_CurveTracer_03: Parameters from EEPROM , possibility to save the three x 4  parameters there.
// 2018.01.25                            New Menu with better separation start buttons "P" and "N" and more space for the parameters and EEPROM-Savings. 
#include <EEPROM.h>

#include <SPI.h>
//
// ### MapleMini Pin definitions   ###############################################

#define BTN0              0   // PB11 PWM// switch gain to 6, bat-voltage from 12 to 24V and divider from 100/33 to 100/(33||20) ###
#define SPI1_NSS_PIN      1   // PB10 PWM// SPI1: TFT_CS, SPI1 : d   
#define TFT_DC            2   // PB2     // SPI1: switch data/control , default: BOOT1  
// boot1                  2   // PB2     // for boot with RS232
#define pin_ADC5_Bat_3V   3   // PB0  PWM// ADC5 read battery-voltage fo alarm ??? <-- open 
//      TFT_MOSI          4   // PA7  PWM// SPI1 intern for TFT ILI9341 
//      TFT_MISO          5   // PA6  PWM// SPI1 intern, not used, for instance SD-Read 
//      TFT_SCK           6   // PA5     // SPI1 intern for TFT ILI9341  
#define pin_ADC4          7   // PA4     // ADC4 --- free ### normal: SPI1_SS_PIN  to PIN12 / PC15 
#define pin_ADC3_Bat_12V  8   // PA3  PWM// ADC3
#define pin_ADC2_Vc_PNP   9   // PA2  PWM// ADC2
#define pin_ADC1_Vc_NPN  10   // PA1  PWM// ADC1
#define pin_ADC0_Vo_OPV  11   // PA0     // ADC0  // PIN1 LM358
#define BTN12            12   // PC15    // BTN12 Switch for changing voltage +/- LT1013
//                       13   // PC14    // ENC2 Inp A     
//                       14   // PC13    // ENC2 Inp B 
// boot0                BUT   // PB8     // BOOT0
//                       15   // PB7     // --- yet not used ----  default:  I2C_1 SDA
//                       16   // PB6     // --- yet not used ----  default:  I2C_1 SCL
//                       17   // PB5  PWM// ENC1 Inp A
//                       18   // PB4  PWM// ENC1 Inp B
#define BTN19            19   // PB3  PWM// Switch for relais to activate pnp,pMOS,pDiode,p-jFET ---> Scan_p_Dev
#define BTN20            20   // PA15 PWM// Switch for relais to activate npn,nMOS,nDiode,n-jFET ---> Scan_n_Dev
//TOUCH IRQ              21   // PA14    // Touch irq
//TOUCH DOUT             22   // PA13    // Touch dout 
//                       23   // PA12    // --- intern USB_DP
//                       24   // PA11 PWM// --- intern USB_DM 
//Touch DIN              25   // PA10 PWM// Touch din
//Touch CS               26   // PA9  PWM// Touch cs
//Touch CLK              27   // PA8  PWM// Touch clk
//SPI2_MOSI              28   // PB15 PWM// SPI2_MOSI intern for DAC MCP48x2
//SPI2_MISO              29   // PB14 PWM// SPI2_MISO intern, not used
//SPI2_SCK               30   // PB13 PWM// SPI2_SCK  intern for DAC MCP48x2
#define SPI2_NSS_PIN     31   // PB12    // SPI2_CS   ( You can change it to the STM32 pin you want).
#define BTN32            32   // PB8     // switch gain to 3 ,bat-voltage to 12V (normal), default : BOOT0 for boot RS232
#define BOARD_LED        33   // PB1     // Pin33 / PB1 on MapleMini not outside 
#define TFT_RST         112   // ??      // formal, in reality on 3.3V 

SPIClass SPI_2(2);        // Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port+
byte data;

/* // MCP48x2 DAC PIN-Map ##########################
//     
//Vdd + 1      8 Vout A  ("Base")
//CS    2      7 Vss  -
//SCK   3      6 Vout B  ("Vcc");
//SDI   4      5 LDAC ( on Vss/-)
//
*/

#include <Adafruit_GFX_AS.h>
#include <Adafruit_ILI9341_STM.h>

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(SPI1_NSS_PIN, TFT_DC);//, TFT_RST); // Using hardware SPI (SPI_1

#include <URTouch.h>

//URTouch(      clk,cs,din,dout,irq); 
URTouch  myTouch( 27,26,25, 22, 21);        //  <<-   24,  23 reserved for USB !!
//URTouch  myTouch(PA8,PA9,PA10,PA13,PA14); //  <<- PA11,PA12 reserved for USB !!

//### from multiencoder / madias ##################  // STM32duino
#define MAXENCODERS 2
volatile int encstate[MAXENCODERS];
volatile int encflag[MAXENCODERS];
boolean A_set[MAXENCODERS];
boolean B_set[MAXENCODERS];
volatile int16_t encoderpos[MAXENCODERS];
volatile int  encodertimer = millis(); // acceleration measurement

int encoderpinA[MAXENCODERS] = {18,14}; // pin array of all encoder A inputs encoder 0,1  // PB4,PC14
int encoderpinB[MAXENCODERS] = {17,13}; // pin array of all encoder B inputs encoder 0,1  // PB5,PC13

//int encoderpinA[MAXENCODERS] = {17,15}; // pin array of all encoder A inputs encoder 0,1  // PB5,PB7
//int encoderpinB[MAXENCODERS] = {18,16}; // pin array of all encoder B inputs encoder 0,1  // PB4,PB6
unsigned int lastEncoderPos[MAXENCODERS];

#include <Wire.h>     // Use the Wire Library for I2C ###########################  not used, only tested

//uint8_t enc0dir = 0;  // Drehrichtung 0=left, 1=right
//uint8_t enc1dir = 0;  // Drehrichtung

uint8_t s_func = 0,s_diode=0,s_npn=0,s_pnp=0,s_fet=1,s_depl=0,s_dio=0;
volatile uint8_t s_comp=0;
uint8_t s_first=1,s_endgr=0,s_touch=0,s_save=0,s_trace=0,s_start=1,s_auto=0;
uint8_t s_pinch = 0,s_jfet=0,s_vgate0=0,s_tend=0,zen_v=0;     // Z??
int8_t s_secy = 0,s_secx = 0,s_exec_npn=0,s_exec_pnp=0,s_graph0;
int s_yfactor = 1,s_xfactor = 1; 
int pinch_v=0,cntv=0;
int n = 0;

byte a=0;                // Variable to store the Byte from I2C
//char buffer0[9];         //the ASCII of the integer will be stored in this char array
//char buffer1[9];         //the ASCII of the integer will be stored in this char array

#define TFT_WIDTH     320
#define TFT_HEIGHT    240
#define GRID_WIDTH    300
#define GRID_HEIGHT   210


const int TextLeft  = 40;
const int TextTop   = 56 + 10 + 4;
const int ValLeft   = 148;
const int BoxTop    = 48;
const int BoxLeft   = 176;
const int RowHeight = 50;

const int OKWidth   = 87;
const int OKHeight  = 34;
const int OKLeft    = (TFT_WIDTH - OKWidth);// 233
const int OKTop     = 1; //5 ## displ TFT_HEIGHT - OKHeight - 8; // #?

const int DiodeLeft   =   0; // 20;  // ### di
const int BIPOLLeft   =  80; // 10;
const int MOSFETLeft  = 160; //114;
const int JFETLeft    = 240; //218;

const int ADC_MAX = 4096; // maple mini // 1024; // arduino NANO
const int DAC_MAX = 4096; // MCP4822=4096, MCP4812=1024, MCP4802 = 256
int amax,dmax,dhalf; 

const int R1 = 33;  // 3.3V Ref // 1.1V Ref  22; // 33; //ADC input potential divider lower resistor k-ohms  ###
const int R2 = 100; // 3.3V Ref // 1.1V Ref 258; // 58; // 58; for 3.6V ; 47; // ADC input potential divider upper resistor k-ohms  ###
const int R3 = 100; // collector resistor ohms
const int R4 = 68;  //50;  // 68; ### // DAC op amp feedback upper resistor k-ohms ###############################################################
const int R5 = 33;  //26;  // 33; ### // DAC op amp feedback lower resistor k-ohms ###############################################################
const int R6 = 10;  // 3.3V Ref // 1.1V Ref 85;  // 10; // measure battery volts potential divider upper resistor k-ohms
const int R7 = 33;  // 3.3V Ref // 1.1V Ref 22;  // 33; // measure battery volts potential divider lower resistor k-ohms

const int DacVref = 40; // DAC Vref in 100s of mV
const int AdcVInt = 33; // ADC Vref in 100s of mV (Arduino UNO/NANO: INTERNAL 1.1 V)
const int AdcVref = 5;  // ADC Vref in V  ### test ???
const int mAmax   = 50; // Ic for top of screen
const unsigned long TimeoutPeriod = 60000;

volatile int16_t count2=0,count3=0,count2_a=999,count3_a=999;// ### e
volatile uint32_t lastEnc3=0,currMill3=0;                // ### e
volatile uint32_t lastEnc2=0,currMill2=0;                // ### e


int minNPNa=0,incNPNa=20,maxNPNa;//,minNPNbasea=10,incNPNbasea=10;  // ### e
int minFETa=10,incFETa=1,maxFETa;//,minFETvolta=10,incFETvolta=10;  // ### e
int minMOSa=10,incMOSa=1,maxMOSa;//,minMOSvolta=10,incMOSvolta=10;  // ### e
int maxDIOa=5,incDIOa=0,minDIOa; //.minAnodea=5,incAnodea=1; 
int16_t c2_BIP,c2_DIO,c2_FET,c2_MOS=0;     
int16_t c3_BIP,c3_DIO,c3_FET,c3_MOS=0;
int16_t c4_BIP,c4_DIO,c4_FET,c4_MOS=0;

//int Adc_12V  = 0;  // test 
int MinVgate = 10;
int MaxVgate = 120;  // 12 ###
int IncVgate = 1;   // 100mV ? ### e 
int MinV_FET = 10;
int MaxV_FET = 120;  // 12 ###
int IncV_FET = 1;   // 100mV ? ### e 
int MinV_MOS = 10;
int MaxV_MOS = 120;  // 12 ###
int IncV_MOS = 1;   // 100mV ? ### e 
int MinIbase = 0;  //20;#############################################
int MaxIbase = 0; // 400 350; ### e µA #############################
int IncIbase = 20;  // ### e
int MinIanode = 0;  //5 ############################################### 
int IncIanode = 0; // ### di max-voltage
int MaxIanode = 5;  // ### di test 
long ngJFET = dmax; // 12 bit 4095
 
int minYposGain, maxYposGain, minBaseGain, maxBaseGain; // used when calc gain

int iaz=0,iu0=0,iu1=0,iu2=0,iu3=0,iuz,val=0;  // ### di
int16_t xt=0,yt=0;
int prev_x = 0, prev_y = 0;
float izf=0,uz=0,fro=0,f_thrn=0,f_thrp=0; 

// FLASH memory address defines ####### EEPROM ########
#define PREAMBLE_VALUE  2859

#define PARAM_PREAMBLE  0
#define PARAM_C2_BIP    1
#define PARAM_C3_BIP    2
#define PARAM_C4_BIP    3
#define PARAM_C2_MOS    4
#define PARAM_C3_MOS    5
#define PARAM_C4_MOS    6
#define PARAM_C2_FET    7
#define PARAM_C3_FET    8
#define PARAM_C4_FET    9
#define PARAM_C2_DIO   10
#define PARAM_C3_DIO   11
#define PARAM_C4_DIO   12

#define PARAM_ZERO1    13
#define PARAM_ZERO2    14
#define PARAM_ZERO3    15
#define PARAM_ZERO4    16
#define PARAM_ZERO5    17
#define PARAM_ZERO6    18


// TFT display constants
#define PORTRAIT      0
#define LANDSCAPE     1

// display colours

#define ILI9341_AQUA   0x07FF
#define ILI9341_PURPLE 0x780F 
#define ILI9341_LIME   0x07E0
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

uint16_t ILI9341_TraceCol = 0;
uint16_t ILI9341_ErrorCol = 0;
uint16_t ILI9341_KindCol  = 0;
uint16_t ILI9341_BASE     = 0;
uint16_t ivxa = 0;

#define LED_ON  digitalWrite(BOARD_LED, LOW)
#define LED_OFF digitalWrite(BOARD_LED, HIGH)

// number of pixels waveform moves left/right or up/down
#define XCURSOR_STEP  25
#define YCURSOR_STEP  10

#define BTN0_DEBOUNCE_TIME 250 // 200###############################

//### from multiencoder / madias ############nn#######
// timer
#define ENCODER_RATE 400    // in microseconds; 
HardwareTimer timer(1);



#define CHANNEL_A 0  // dac MCP48x2
#define CHANNEL_B 1  // dac MCP48x2


enum TkindDUT {tkNothing, tkPNP, tkNPN, tkPMOS, tkNMOS, tkNJFET, tkPJFET, tkPDIODE, tkNDIODE};  // ### di
     //          0,        1       2      3           4         5        6        7        8
volatile TkindDUT curkind,lastkind;


enum TclassDUT {tcxxx, tcBIPOL, tcMOSFET, tcJFET, tcDIODE};  // ### di 0,1,2,3,4
volatile TclassDUT CurDUTclass = tcDIODE;//tcMOSFET; //tcBIPOL;

volatile uint8_t class_new=0,class_old=4,class_cur=4; // 1 = BIPOL, 2 = MOS, 3 = FET, 4 = DIODE

static int volt_dac0=0,volt_dac1=0;
static int diff_v0= 50,diff_v1=50;

volatile uint8_t s_t      = 1;   // ##### touch
volatile int32_t actValue = 0;   // ##### touch
volatile uint8_t actFunc  = 1;  
volatile int16_t encPos_f = 1;
volatile int16_t encPos_p = 0;
volatile int16_t actPosP  = 0; 
volatile int8_t curp[9] = {0, 0, 1, 6, 0, 0, 7, 1, 1};  // current switch position
static uint8_t s_touched  = 0;   // ##### touch

long i_o=0,i_j=0,i_n=0,j_o=0;
int16_t touchY, touchX;          // ##### touch
volatile int x,y;                // ##### touch
int  stCurrentLen=0;             // ##### touch
char stCurrent[25]="";           // ##### touch
char stLast[25]="";              // ##### touch
volatile long lastBtnPress = 0;

uint8_t byte_t[5];
uint16_t cfx=0,cfy=224,cfw=20,cfh=16;

int16_t xCursor;
//int diff_v=50;
int32_t delta = 0; 

uint8_t tx[100], ty[100];
uint8_t tl=0;








/***************************************************************************************
** Function name:           rgb
** Description:             convert three 8 bit RGB levels to a 16 bit colour value
***************************************************************************************/
uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}



// ------------------------
void formatSaveConfig()  {
// ------------------------
 
  EEPROM.format();               // Formatting EEPROM
  
  //  Saving all config params....
  
  SaveParEEPROM(PARAM_PREAMBLE, PREAMBLE_VALUE);
  
  SaveParEEPROM(PARAM_C2_BIP, c2_BIP);
  SaveParEEPROM(PARAM_C3_BIP, c3_BIP);
  SaveParEEPROM(PARAM_C4_BIP, c4_BIP);
  
  SaveParEEPROM(PARAM_C2_MOS, c2_MOS);
  SaveParEEPROM(PARAM_C3_MOS, c3_MOS);
  SaveParEEPROM(PARAM_C4_MOS, c4_MOS);

  SaveParEEPROM(PARAM_C2_FET, c2_FET);
  SaveParEEPROM(PARAM_C3_FET, c3_FET);
  SaveParEEPROM(PARAM_C3_FET, c4_MOS);

  SaveParEEPROM(PARAM_C2_DIO, c2_DIO);
  SaveParEEPROM(PARAM_C3_DIO, c3_DIO);
  SaveParEEPROM(PARAM_C4_DIO, c4_DIO);
  
}



// ------------------------
void SaveParEEPROM(uint16_t param, uint16_t data)  {
// ------------------------
  uint16 status = EEPROM.write(param, data);
  if(status != EEPROM_OK) {
    // ("Unable to save param in EEPROM, code: ")
    tft.fillRect(1,1,70,12,ILI9341_BLACK);
    tft.setTextColor(ILI9341_ORANGE);
    tft.drawString("EEP-ST: ",1,1,2);
    tft.drawNumber(status,60,1,2);
    delay(1000);
    return;
  }
  tft.fillRect(0,1,70,12,ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.drawString("EEP-ST: ",2,1,2);
  tft.drawNumber(status,60,1,2);
  delay(200);
}



// ------------------------
void loadDefaults()  {
// ------------------------
            // Loading defaults

  
  maxDIOa = MaxIanode; c2_DIO = 5;     // 5-ma
  incDIOa = IncIanode; c3_DIO = 6;     // 12V (x-scale: 1,2,3,4,6,12V,24V,0V)
  minDIOa = MinIanode; c4_DIO = 1;     // 1-ma
  
  minNPNa = MinIbase;  c2_BIP = 0;    // 0;
  incNPNa = IncIbase;  c3_BIP = 20;   // 20;
  maxNPNa = MaxIbase;  c4_BIP = 400;  // 400;

  incMOSa = IncV_MOS;  c2_MOS = 10;   // 1. komma 
  minMOSa = MinV_MOS;  c3_MOS = 2;    // 1. komma
  maxMOSa = MaxV_MOS;  c4_MOS = 60;   // 1. komma
  
  minFETa = MinV_FET;  c2_FET = 30;   // 3.0 V
  incFETa = IncV_FET;  c3_FET = 2;    // 0.2 V 
  maxFETa = MaxV_FET;  c4_FET = 10;   // 1.0 V

  
}


// ------------------------
void loadConfEEPROM(boolean res_EEPROM, boolean display_Data)  {
// ------------------------
  
  if(EEPROM.init() != EEPROM_OK)  {
    loadDefaults();   // set c2_BIP,C3_BIPO etc with start-values f.i. 5
    return;
  }
  
  // read preamble
  if(res_EEPROM ||                                            // reset 
    (EEPROM.read(PARAM_PREAMBLE) != PREAMBLE_VALUE))  {  // EEPROM leer
    loadDefaults();
    formatSaveConfig();  // write to EEPROM
    return;
  } 
  
  // read all the parameters from EEPROM
  uint16_t data;
  setParamEEPROM();
  /*
  data = EEPROM.read(PARAM_C2_BIP);  c2_BIP = data;
  data = EEPROM.read(PARAM_C3_BIP);  c3_BIP = data;

  data = EEPROM.read(PARAM_C2_MOS);  c2_MOS = data;
  data = EEPROM.read(PARAM_C3_MOS);  c2_MOS = data;

  data = EEPROM.read(PARAM_C2_FET);  c2_FET = data;
  data = EEPROM.read(PARAM_C3_FET);  c3_FET = data;
  
  data = EEPROM.read(PARAM_C2_DIO);  c2_DIO = data;
  data = EEPROM.read(PARAM_C3_DIO);  c3_DIO = data;
  */
  if (display_Data == true)  displayDataEEPROM(ILI9341_ORANGE);
   
 
}

void setParamEEPROM() {
  
  c2_BIP = EEPROM.read(PARAM_C2_BIP); 
  c3_BIP = EEPROM.read(PARAM_C3_BIP); 
  c4_BIP = EEPROM.read(PARAM_C4_BIP); 
  c2_MOS = EEPROM.read(PARAM_C2_MOS); 
  c3_MOS = EEPROM.read(PARAM_C3_MOS); 
  c4_MOS = EEPROM.read(PARAM_C4_MOS); 
  c2_FET = EEPROM.read(PARAM_C2_FET); 
  c3_FET = EEPROM.read(PARAM_C3_FET); 
  c4_FET = EEPROM.read(PARAM_C4_FET); 
  c2_DIO = EEPROM.read(PARAM_C2_DIO); 
  c3_DIO = EEPROM.read(PARAM_C3_DIO); 
  c4_DIO = EEPROM.read(PARAM_C4_DIO); 
  
}

void updateEEPROM(TclassDUT class_c) {
 
  if (class_c == tcBIPOL) {
 // c4_BIP = 400; 
   SaveParEEPROM(PARAM_C2_BIP, c2_BIP);
   SaveParEEPROM(PARAM_C3_BIP, c3_BIP);
   SaveParEEPROM(PARAM_C4_BIP, c4_BIP);
  }
  else if (class_c == tcMOSFET) {
//   c4_MOS = 60; 
   SaveParEEPROM(PARAM_C2_MOS, c2_MOS);
   SaveParEEPROM(PARAM_C3_MOS, c3_MOS);
   SaveParEEPROM(PARAM_C4_MOS, c4_MOS);
  }
  else if (class_c == tcJFET) {
//    c4_FET = 20;
   SaveParEEPROM(PARAM_C2_FET, c2_FET);
   SaveParEEPROM(PARAM_C3_FET, c3_FET);
   SaveParEEPROM(PARAM_C4_FET, c4_FET);
  }
  else if (class_c == tcDIODE) {
 //  c4_DIO = 50; 
   SaveParEEPROM(PARAM_C2_DIO, c2_DIO);
   SaveParEEPROM(PARAM_C3_DIO, c3_DIO);
   SaveParEEPROM(PARAM_C4_DIO, c4_DIO);
  }
}


void displayEEPROM(TclassDUT class_c) {
  
  uint16_t dat_c2,dat_c3,dat_c4=29;
  long cnt2_d=0,cnt2_s,cnt2_l;
  int16_t cnt4_d = 0;
  int time_a,time_n;
  uint8_t end_w=0;
 
  tft.setTextColor(ILI9341_AQUA);

  if (class_c == tcDIODE) {
    tft.fillRect(0,1,70,32,ILI9341_BLACK); 
    dat_c2 = EEPROM.read(PARAM_C2_DIO); 
    dat_c3 = EEPROM.read(PARAM_C3_DIO);
    dat_c4 = EEPROM.read(PARAM_C4_DIO);
    tft.drawNumber(dat_c2,8,1,2); 
    tft.drawNumber(dat_c3,31,1,2); 
    tft.drawNumber(dat_c4,55,1,2); 
    tft.drawNumber(c2_DIO,8,15,2); 
    tft.drawNumber(c3_DIO,31,15,2); 
    tft.drawNumber(c4_DIO,55,15,2); 
  }  
  else if (class_c == tcBIPOL) {
   
    tft.fillRect(0,31,70,32,ILI9341_BLACK); 
    dat_c2 = EEPROM.read(PARAM_C2_BIP); 
    dat_c3 = EEPROM.read(PARAM_C3_BIP);
    dat_c4 = EEPROM.read(PARAM_C4_BIP);
    tft.drawNumber(dat_c2,8,31,2); 
    tft.drawNumber(dat_c3,31,31,2); 
    tft.drawNumber(dat_c4,55,31,2); 
    tft.drawNumber(c2_BIP,8,45,2); 
    tft.drawNumber(c3_BIP,31,45,2); 
    tft.drawNumber(c4_BIP,55,45,2); 
    delay(500);
    cnt2_s = count2;
    encoderpos[0];
    count2 = dat_c4; //c4_BIP;
    encoderpos[0] = count2;
    cnt2_l = count2;
    time_a = millis();
    while (end_w == 0) {
      time_n = millis();
      if (time_n - time_a > 1000) end_w = 1;
      if (count2 != cnt2_l) {  // count2 canged
        cnt2_l = count2;
        c4_BIP = count2;
        if (c4_BIP >=0) {
          SaveParEEPROM(PARAM_C4_BIP,c4_BIP); 
          tft.fillRect(55,31,24,30,ILI9341_BLACK); 
          tft.drawNumber(c4_BIP,55,31,2); 
          tft.drawNumber(c4_BIP,55,45,2); 
          tft.setTextColor(ILI9341_CYAN);
          tft.fillRect(179,106,44,17,ILI9341_DARKGREY);   // ###
          tft.drawNumber(c4_BIP,180,106,4); 
        }  
        time_a = time_n;
      }
    }    
    count2 = cnt2_s;
    encoderpos[0] = cnt2_s;
       
  }  
  else if (class_c == tcMOSFET) {
    tft.fillRect(0,61,70,32,ILI9341_BLACK); 
    dat_c2 = EEPROM.read(PARAM_C2_MOS); 
    dat_c3 = EEPROM.read(PARAM_C3_MOS);
    dat_c4 = EEPROM.read(PARAM_C4_MOS);
    tft.drawNumber(dat_c2,8,61,2); 
    tft.drawNumber(dat_c3,31,61,2); 
    tft.drawNumber(dat_c4,55,61,2); 
    tft.drawNumber(c2_MOS,8,75,2); 
    tft.drawNumber(c3_MOS,31,75,2); 
    tft.drawNumber(c4_MOS,55,75,2); 
    delay(500);
    cnt2_s = count2;
    encoderpos[0];
    count2 = dat_c4; //c4_BIP;
    encoderpos[0] = count2;
    cnt2_l = count2;
    time_a = millis();
    while (end_w == 0) {
      time_n = millis();
      if (time_n - time_a > 1000) end_w = 1;
      if (count2 != cnt2_l) {  // count2 canged
        cnt2_l = count2;
        c4_MOS = count2;
        if (c4_MOS >=0) {
          SaveParEEPROM(PARAM_C4_MOS,c4_MOS); 
          tft.fillRect(55,61,24,30,ILI9341_BLACK); 
          tft.drawNumber(c4_MOS,55,61,2); 
          tft.drawNumber(c4_MOS,55,75,2); 
          tft.fillRect(179,106,44,17,ILI9341_DARKGREY);   // ###
          drawDec1(c4_MOS,180,106,4,ILI9341_CYAN);
       
        }  
        time_a = time_n;
      }
    }    
    count2 = cnt2_s;
    encoderpos[0] = cnt2_s;
  }  
  else if (class_c == tcJFET) {
    tft.fillRect(0,91,70,32,ILI9341_BLACK); 
    dat_c2 = EEPROM.read(PARAM_C2_FET); 
    dat_c3 = EEPROM.read(PARAM_C3_FET);
    dat_c4 = EEPROM.read(PARAM_C4_FET);
    tft.drawNumber(dat_c2,8,91,2); 
    tft.drawNumber(dat_c3,31,91,2); 
    tft.drawNumber(dat_c4,55,91,2); 
    tft.drawNumber(c2_FET,8,105,2); 
    tft.drawNumber(c3_FET,31,105,2); 
    tft.drawNumber(c4_FET,55,105,2); 
    
    delay(500);
    cnt2_s = count2;
    encoderpos[0];
    count2 = dat_c4; //c4_BIP;
    encoderpos[0] = count2;
    cnt2_l = count2;
    time_a = millis();
    while (end_w == 0) {
      time_n = millis();
      if (time_n - time_a > 1000) end_w = 1;
      if (count2 != cnt2_l) {  // count2 canged
        cnt2_l = count2;
        c4_FET = count2;
        if (c4_FET >=0) {
          SaveParEEPROM(PARAM_C4_FET,c4_FET); 
          tft.fillRect(55,91,24,30,ILI9341_BLACK); 
          tft.drawNumber(c4_FET,55,91,2); 
          tft.drawNumber(c4_FET,55,105,2); 
          tft.fillRect(179,106,44,17,ILI9341_DARKGREY);   // ###    
          drawDec1(c4_FET,180,106,4,ILI9341_CYAN);
        }  
        time_a = time_n;
      }
    }    
    count2 = cnt2_s;
    encoderpos[0] = cnt2_s;
  }  
          
  delay(1000);
  
}


void displayDataEEPROM(uint16_t tx_col) {
   // read all the parameters from EEPROM
  uint16_t data;
 
  tft.setTextColor(tx_col);
  tft.drawString("npn/pnp  MOSFET   JFET   DIODE ",10,30,2);
  
  data = EEPROM.read(PARAM_C2_BIP);  tft.drawNumber(data,50,60,2);
  data = EEPROM.read(PARAM_C3_BIP);  tft.drawNumber(data,50,76,2);

  data = EEPROM.read(PARAM_C2_MOS);  tft.drawNumber(data,100,60,2);
  data = EEPROM.read(PARAM_C3_MOS);  tft.drawNumber(data,100,76,2);

  data = EEPROM.read(PARAM_C2_FET);  tft.drawNumber(data,150,60,2);
  data = EEPROM.read(PARAM_C3_FET);  tft.drawNumber(data,150,76,2);

  data = EEPROM.read(PARAM_C2_DIO);  tft.drawNumber(data,200,60,2);
  data = EEPROM.read(PARAM_C3_DIO);  tft.drawNumber(data,200,76,2);
  
}

void dac10(uint8_t chan, uint16_t value)    {  // MCP4812 10 bit

#define DAC_A_WRITE 0b0000000000000000
#define DAC_B_WRITE 0b1000000000000000
#define GAIN_2X     0b0000000000000000
#define GAIN_1X     0b0010000000000000
//#define SHUTDOWN    0b0000000000000000
#define ACTIVE      0b0001000000000000

 uint8_t byte0;
 uint8_t byte1;
 uint16_t cmdWrd = 0;
 uint16_t data = 0;
 data = value;
 data = data << 2;   // 10bit

 if (chan == 0) cmdWrd = DAC_A_WRITE;
 else           cmdWrd = DAC_B_WRITE;
 cmdWrd |= GAIN_2X;
 cmdWrd |= ACTIVE;
 //cmdWrd |= (0b0000111111111100 & data);  // 10 bit 8 bit ??
 cmdWrd |= data;
 byte0 = cmdWrd >> 8;
 byte1 = cmdWrd;// << 8; //& 0b0000000011111100;
 SPI_2.setDataMode(SPI_MODE0);
 SPI_2.setBitOrder(MSBFIRST);
 digitalWrite(SPI2_NSS_PIN, LOW);
 SPI_2.transfer(byte0);
 SPI_2.transfer(byte1);
 digitalWrite(SPI2_NSS_PIN, HIGH);
/*
 if (chan == 0) {
   tft.fillRect(158,99,50,65,ILI9341_BLACK);
   tft.drawNumber(value,160,100,4);
   tft.drawNumber(byte0,160,120,4);
   tft.drawNumber(byte1,160,140,4);
 }
 else {
   tft.fillRect(229,99,50,65,ILI9341_BLACK);
   tft.drawNumber(value,230,100,4);
   tft.drawNumber(byte0,230,120,4);
   tft.drawNumber(byte1,230,140,4);
}
*/
}



void dac12( uint16_t vald,uint8_t chan,uint8_t uout)    {  // MCP4822 12 bit

#define DAC_A_WRITE 0b0000000000000000
#define DAC_B_WRITE 0b1000000000000000
#define GAIN_2X     0b0000000000000000
#define GAIN_1X     0b0010000000000000
//#define SHUTDOWN    0b0000000000000000
#define ACTIVE      0b0001000000000000

 uint8_t byte0;
 uint8_t byte1;
 uint16_t cmdWrd = 0;
 //uint16_t data = 0;
 //data = value;
 //data = data << 2;   // 10bit
 //if (chan == 0)      cmdWrd = 0x0000;
 //else if (chan == 1) cmdWrd = 0x8000;
 if (chan == 0) cmdWrd = DAC_A_WRITE;  // bit 15 : 0=channel A
 else           cmdWrd = DAC_B_WRITE;  // bit 15 : 1=channel B
 //                                    // bit 14 : don't care
 if (uout == 2) cmdWrd |= GAIN_1X;     // bit 13 : 1=1*Vref*D/4096, Vref=2.048V  0 ... 2.047 V  
 else           cmdWrd |= GAIN_2X;     // bit 13 : 0=2*Vref*D/4096; Vref=2.048V  0 ... 4.095 V
 cmdWrd |= ACTIVE;                     // bit 12 : 1=active,0=shutdown the selected DAC
 cmdWrd |= (0b1111111111111111 & vald);   //
 byte0   = cmdWrd >> 8;                   // most-sign.bit
 byte1   = (0b0000000011111111 & vald);

 SPI_2.setDataMode(SPI_MODE0);
 SPI_2.setBitOrder(MSBFIRST);
 digitalWrite(SPI2_NSS_PIN, LOW);
 
 SPI_2.transfer(byte0);
 SPI_2.transfer(byte1);

 digitalWrite(SPI2_NSS_PIN, HIGH);
 delayMicroseconds(5);  //  50 #####
 //itoa(byte0,buffer0,2); //(integer, yourBuffer, base) for test transfer data
 //itoa(byte1,buffer1,2); //(integer, yourBuffer, base) for test transfer data

}



//-------------------------------------------------------------------------
// SetDacBase
//   sets the base DAC output
//-------------------------------------------------------------------------

void SetDacBase(uint16_t val_a,  int tDelay) {   // 10,12 bit    
 
  
  dac12(val_a,CHANNEL_A,2);  // 0... 2.074V  --> 0... 12.282 on output PIN7 (gain = +6) LT1013
 // dac12(val_a,CHANNEL_A,4);  // 1... 4.095V onlky for zener and p-JFET 
  if (tDelay > 0)   delayMicroseconds(tDelay);
}

void SetDacVcc(uint16_t val_b, int tDelay) {   // 10,12 bit    
 
 // dac10(CHANNEL_B,val_b);
 // dac12(val_b,CHANNEL_B,2);  // 1... 2.047V
  dac12(val_b,CHANNEL_B,4);  // 1... 4.095V 
  if (tDelay > 0)   delayMicroseconds(tDelay);
}



//-------------------------------------------------------------------------
// GetTouch
//   is the user touching the screen anywhere
//-------------------------------------------------------------------------
bool GetTouch(void) { 
int16_t x, y;
    
  while (myTouch.dataAvailable() == true)
  {
    myTouch.read();
    x = myTouch.getX();  
    y = myTouch.getY();
  
    if ((x!=-1) and (y!=-1))
    {
      xt = 320 - x; // 180° rotation  ########################################
      yt = 240 - y; // 180° rotation  ########################################
//     tft.fillRect(0,0,70,20,ILI9341_BLACK);
//     tft.drawNumber(xt,0,1,2);
//     tft.drawNumber(yt,40,1,2);
     
      s_touch = 1;
  //   count2  = 0;
 //    count3  = 0;
 //    encoderpos[0] = 0;  // ?? ##################################################
 //    encoderpos[1] = 0;  // ??
 //   delay(2000);
      return(true);
    }  // if
    
  }   // while
  s_touch = 0;
  return(false);
}    


//-------------------------------------------------------------------------
// Havetouch
//   is the user touching the screen anywhere
//-------------------------------------------------------------------------

bool HaveTouch() {
  int16_t x, y;
 // bool touched;
 
 // if(s_comp > 0)  return true;
  
 
  return GetTouch();  //####################################<
  
}


//-------------------------------------------------------------------------
// TurnOffLoad
//   sets load current to zero
//-------------------------------------------------------------------------

void TurnOffLoad(TkindDUT kind) {

  switch (kind) {
 //   case tkNothing:
   //   digitalWrite(BTN19, LOW);   // rel  // stop  Scan_p_Dev  
   //    digitalWrite(BTN20, LOW);   // rel  // stop  Scan_n_Dev  
  //   break; 
    case tkPJFET:
      SetDacBase(dhalf,0);  // +12...+24V  ??JF 
      SetDacVcc(dmax, 0);
     break;  
    case tkPMOS:
    case tkPNP:
      SetDacBase(0, 0);
      SetDacVcc(0, 0);  // 1023
     break;
    case tkPDIODE:
      SetDacBase(0, 0);
      SetDacVcc(dhalf, 0);  // 1023
     break; 
      
    case tkNJFET: 
      SetDacBase(dhalf,0);  // -0...-12VV  ??JF 
      SetDacVcc(dmax, 0);
     break;
    case tkNMOS:
    case tkNPN:
      SetDacBase(0,0);
      SetDacVcc(dmax, 0);
     break;
    case tkNDIODE: 
      SetDacBase(0, 0);
      SetDacVcc(dhalf, 0);  // 1023
     break;   
 
  }
}



//-------------------------------------------------------------------------
// DrawCheckCross   line 1655
//   draw a CheckCross in the CheckBox or delete it
//-------------------------------------------------------------------------

void drawCheckCross(uint8_t dut_class, uint8_t paint) {
  
   int BoxLeft = 20;
   int BoxTop  = 52;
   uint16_t class_xt[] = {0,80,160,240,0};     // checkbox x
   uint16_t class_yt[] = {0,135,135,135,135};  // checkbox y
   int Left    = class_xt[dut_class];  // Left:  0,92,164,236,20 0  10 114 218 20
   int Top     = class_yt[dut_class];  // Top:  0 135 135 135 27  

   if (dut_class == 0) return;         // start without old
   Left = Left + 1;
   if (paint == 0) {                    // erase cross
 
     tft.drawRect(Left + BoxLeft+3, Top + BoxTop+3, 33 - 6, 33 - 6, ILI9341_WHITE);  
     tft.drawRect(Left + BoxLeft+4, Top + BoxTop+4, 33 - 8, 33 - 8, ILI9341_WHITE);  
     tft.drawRect(Left + BoxLeft+5, Top + BoxTop+5, 33 - 10, 33 - 10, ILI9341_WHITE); 
     tft.drawLine(Left + BoxLeft, Top + BoxTop, Left + BoxLeft + 32, Top + 52 + 32, ILI9341_WHITE);
     tft.drawLine(Left + BoxLeft, Top + BoxTop + 32, Left + BoxLeft + 32, Top + BoxTop, ILI9341_WHITE);
     return;
   }
  
   tft.drawRect(Left + BoxLeft+3, Top + BoxTop+3, 33 - 6, 33 - 6, ILI9341_RED);  
   tft.drawRect(Left + BoxLeft+4, Top + BoxTop+4, 33 - 8, 33 - 8, ILI9341_RED);  
   tft.drawRect(Left + BoxLeft+5, Top + BoxTop+5, 33 - 10, 33 - 10, ILI9341_RED); 
   tft.drawLine(Left + BoxLeft, Top + BoxTop, Left + BoxLeft + 32, Top + 52 + 32, ILI9341_BLACK);
   tft.drawLine(Left + BoxLeft, Top + BoxTop + 32, Left + BoxLeft + 32, Top + BoxTop, ILI9341_BLACK);
    
}


//-------------------------------------------------------------------------
// drawDec1
//   draw i/10 with one d.p.
//-------------------------------------------------------------------------

void drawDec1(int i, int16_t xp, int16_t yp, uint8_t f_size, uint16_t color) {
  int id;
 // f_size = 2;
  int16_t fw = 7;  // f_size=2
  if (f_size == 4) fw = 11;
  tft.setTextColor(color);
  if (i < 0) {
    tft.drawChar('-', xp,yp,f_size);
    i = -i;
    xp = xp + fw;
  }
  id = i/10;
  tft.drawNumber(id,xp,yp,f_size); //DrawInt(i / 10, Font, color);
  if (id > 9) xp = xp + 2*fw;
  else        xp = xp + fw;
  tft.drawString(". ",xp+3,yp,f_size); //DrawString(".", Font, color);
  xp = xp + fw + fw/2;
  tft.drawNumber(i % 10,xp,yp,f_size);
}



void saveParam(int minVal,int incVal) {
     
   if (s_func == 1) drawFunc(1,1,60,"SavePar ",ILI9341_WHITE);  //####
   s_save = 1;
 //  tft.fillRect(25,0,60,20,ILI9341_BLACK);
 //  tft.drawNumber(class_old,26,1,2);
 
   if      (class_old == 1)   {  // BIPOL/npn-pnp
     minNPNa  = minVal;
     incNPNa  = incVal;
     MinIbase = minVal;
     IncIbase = incVal;
     c2_BIP   = count2;
     c3_BIP   = count3;
     CurDUTclass = tcBIPOL;
  //   tft.drawNumber(c2_BIP,46,1,2);
  //   tft.drawNumber(c3_BIP,67,1,2);
   }
   else if (class_old == 2)   {  // MOS-Fet
     minMOSa  = minVal;
     incMOSa  = incVal;
     MinV_MOS = minVal;
     IncV_MOS = incVal;
     c2_MOS   = count2;
     c3_MOS   = count3;
     CurDUTclass = tcMOSFET;
   //  tft.drawNumber(c2_MOS,46,1,2);
   //  tft.drawNumber(c3_MOS,67,1,2);
   }
   else if (class_old == 3)   {  // j-FET
     minFETa  = minVal;
     incFETa  = incVal;
     MinV_FET = minVal;
     IncV_FET = incVal;
     c2_FET   = count2;
     c3_FET   = count3;
     CurDUTclass = tcJFET;
  //   tft.drawNumber(c2_FET,46,1,2);
  //   tft.drawNumber(c3_FET,67,1,2);
   }
   else if (class_old == 4)   {  // Diode
     maxDIOa   = minVal;     // ???
     incDIOa   = incVal;
     MaxIanode = minVal;
     IncIanode = incVal;
     c2_DIO    = count2;
     c3_DIO    = count3;
     if (count3 == 0) {
       zen_v = 24;  // Z??######################################################################################
       s_secx = 1;
     }
     else s_secx = 0;  
     CurDUTclass = tcDIODE;
  //   tft.drawNumber(c2_DIO,46,1,2);
  //   tft.drawNumber(c3_DIO,67,1,2);
   } 
   
   if (class_old == class_new) return; 
 
   count2 = 0;
   count3 = 0;
   encoderpos[0] = 0;
   encoderpos[1] = 0;
   count2_a = count2; //9999;
   count3_a = count3; //9999;
   
 }



void saveCurClass(int class_nr) {
    
   class_cur = class_nr;
   class_old = class_nr;
     
   if      (class_nr == 1) curkind = tkNPN;
   else if (class_nr == 2) curkind = tkNMOS;
   else if (class_nr == 3) curkind = tkNJFET;
   else if (class_nr == 4) curkind = tkNDIODE;
   
   if      (class_nr == 1) CurDUTclass = tcBIPOL;
   else if (class_nr == 2) CurDUTclass = tcMOSFET;
   else if (class_nr == 3) CurDUTclass = tcJFET;
   else if (class_nr == 4) CurDUTclass = tcDIODE;
  // s_save = 1;  //######################################################
 }



void drawFunc(uint16_t x, uint16_t y, uint16_t xl, char *func, uint16_t col) 
{
   tft.fillRect(x,y,xl,12,ILI9341_BLACK);
   tft.setCursor(x,y);
   tft.setTextColor(col);
   tft.drawString(func,x,y,2);
   delay(1000);
}



//-------------------------------------------------------------------------
// InitGraph
//   draws the grid background of the graph
//-------------------------------------------------------------------------

void InitGraph()         {
  long ix, x, iy, y;
  long ixm,ixw;      // Z??
  int  iyi = 5, iys = 5, iym = mAmax;    // 50mA / 5mA "ZENER"
 
  if (s_func == 1) drawFunc(1,1,60,"InitGraph",ILI9341_WHITE);  //####

  s_trace  = 1;
  s_endgr  = 0;

  if (s_comp <= 2) {
    ILI9341_TraceCol = ILI9341_WHITE;
    tft.fillScreen(ILI9341_BLACK); // ClearDisplay(ILI9341_BLACK);
  }  
             //   compare with other DUT ##co##
// else  { ILI9341_TraceCol = ILI9341_LIME;  tft.fillRect(300,80,20,20,ILI9341_BLACK);   tft.setTextColor(ILI9341_LIME);     tft.drawNumber(s_comp,300,80,4);  return; }
 
  tft.setTextColor(ILI9341_DARKGREY);  tft.drawString("0",2,TFT_HEIGHT + 4,1);
  tft.drawLine(0,0,0,TFT_HEIGHT-10,ILI9341_DARKGREY);
  tft.drawLine(0, TFT_HEIGHT - 1, TFT_WIDTH, TFT_HEIGHT - 1, ILI9341_DARKGREY);
     
  for (ix = 1; ix <= 12; ix++) {
    x = 25 * ix ; // 26 #################
    tft.drawLine(x,0,x,TFT_HEIGHT-13,ILI9341_DARKGREY); //DrawLine(x, 0, x, TFT_HEIGHT, ILI9341_DARKGREY);
  }
  
 // x - scala #######################################################################
  if (s_xfactor == 99)     s_xfactor = 1;       // even open #####################################################
  if  ((zen_v > 0) && (zen_v < 12)) s_xfactor = 12/zen_v;    
  else s_xfactor = 1; 
  if (zen_v > 0)   ixm = zen_v;         // up to 12 V for zener  // Z?? 
  else             ixm = 12/s_xfactor;  // s_xfactor special scala for diodes 
 // Serial.print("IG iuz ");Serial.print(iuz); Serial.print(" zen_v "); Serial.print(zen_v);Serial.print(" x_fac "); Serial.print(s_xfactor);Serial.print(" ixm ");Serial.println(ixm);       //#########################################################################
  for (ix = 1; ix <= ixm; ix++) {
      
  //  x = TFT_WIDTH * ix * R1 / (R2 + R1) / AdcVref; // 320*12*22/(22+58)/5=0.275 //   =ix*64*R1/R1+R2 = ix*64*33/80=26
  //  x = TFT_WIDTH * ix * R1 / (R2 + R1) / AdcVInt; // 320*12*22/(22+258)/11=27.4 //  =ix*64*R1/R1+R2 = ix*64*33/80=26  ###
  //  x = TFT_WIDTH * ix * R1 / (R2 + R1) / AdcVref; // 320*12*33.2/(33.2+100)/33=24.2
  //  =ix*64*R1/R1+R2 = ix*64*33/80=26  ###

    x = 25 * ix * s_xfactor;   // 26 #################
    ixw = ix;
    if (zen_v == 24) ixw = ixw +ixw;
   
      
    //tft.setCursor(x + 2, TFT_HEIGHT - 20);  //    ILI9341SetCursor(x + 2, TFT_HEIGHT - 3);
    tft.setTextColor(ILI9341_LIGHTGREY); 
    if (s_exec_pnp == 1)   {    // kind == tkPNP || kind == tkPMOS || kind == tkPJFET || kind == tkPDIODE) {
       if (ixw <= 9) tft.drawChar('-',x-11,TFT_HEIGHT - 14,2);    //DrawString("-", SmallFont, ILI9341_DARKGREY);
       else          tft.drawChar('-',x-18,TFT_HEIGHT - 14,2);    //DrawString("-", SmallFont, ILI9341_DARKGREY);
    }
//  else  tft.drawChar('+',x-5,TFT_HEIGHT - 15,2);     
   
    if    (ixw <= 9){
      tft.drawNumber(ixw,x-6,TFT_HEIGHT - 15,2);      // Z??
      tft.drawChar('V',x+2,TFT_HEIGHT - 15,2);        
    }
    else {
      tft.drawNumber(ixw,x-14,TFT_HEIGHT - 15,2);     // Z??
      tft.drawChar('V',x+2,TFT_HEIGHT - 15,2);   
    }  
  
  }    // end: for ...
  x = 0;
  // y - scala #######################################################################
    
  for (iy = iys; iy <= iym; iy += iyi) {  // 5 ... 50
    y = TFT_HEIGHT - 1 - TFT_HEIGHT * iy / mAmax;
    tft.setCursor(2, y + 8);  //   ILI9341SetCursor(2, y + 8);
    tft.setTextColor(ILI9341_LIGHTGREY);
    if ((iy > 0) && (s_exec_pnp == 1))   {    // 1(kind == tkPNP || kind == tkPMOS || kind == tkPJFET)) ...
      tft.drawString("-",x+2,y,2);// DrawString("-", SmallFont, ILI9341_DARKGREY);
     
      if (s_yfactor <=5) tft.drawNumber(iy/s_yfactor, x+8,y,2);         //DrawInt(iy, SmallFont, ILI9341_DARKGREY);
      else                    tft.drawNumber(iy + iy, x+8,y,2); // max 100mA   
    }  
    else {
    
      if (s_yfactor <=5) tft.drawNumber(iy/s_yfactor, x+3,y,2);         //DrawInt(iy, SmallFont, ILI9341_DARKGREY);
      else                    tft.drawNumber(iy + iy, x+3,y,2); // max 100mA  
    }
  
    tft.drawLine(0,y,TFT_WIDTH,y,ILI9341_DARKGREY); //DrawLine(0, y, TFT_WIDTH, y, ILI9341_DARKGREY);
    
  }
  x = x + 30;                    // 10/50mA
  if (s_yfactor > 5) x = x + 10; // 100mA 
  tft.drawString("mA", x,y,2);
 
}




//-------------------------------------------------------------------------
// DrawKindStr  line 237
//   draws the kind of the DUT at the top of the screen
//-------------------------------------------------------------------------

void DrawKindStr(TkindDUT kind) {
  if (s_func == 1) drawFunc(1,1,60,"DrawKind",ILI9341_WHITE);  //####
  uint16_t xvn = 230,xvi=240;
  
  switch (kind) {
    case tkPNP:
      tft.setTextColor(ILI9341_YELLOW); 
      tft.drawString("PNP",xvn,5,4);                      //196
      drawTransistor('p',185,0,ILI9341_AQUA);  // test
     break;
    case tkNPN:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("NPN",xvn,5,4);                      // 196
      drawTransistor('n',185,0,ILI9341_PINK);  // test
     break;
    case tkPMOS:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("p-MOS",xvn,3,4);                        // 196
      drawMosFet('p',185,0,ILI9341_AQUA);
     break;
    case tkNMOS:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("n-MOS",xvn,3,4);                        // 196
      drawMosFet('n',185,0,ILI9341_ORANGE);
     break;
    case tkPJFET:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("p-JFET",xvn,3,4);
      drawJFet('p',190,0,ILI9341_AQUA);
     break;
    case tkNJFET:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("n-JFET",xvn,3,4);
      drawJFet('n',190,0,ILI9341_ORANGE);
     break;
    case tkPDIODE:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("pn-Diode",xvn-20,3,4);
      drawDiode('p',xvn-55,0,ILI9341_PINK);   
     break;
    case tkNDIODE:
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("np-Diode",xvn-20,3,4);
      drawDiode('n',xvn-55,0,ILI9341_AQUA);//MAGENTA);   
     break;
    default: // unknown
      tft.setTextColor(ILI9341_YELLOW);
      tft.drawString("Curve Tracer",(TFT_WIDTH - 117) / 2,3,4);
   }
}

void DrawFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  
  tft.drawFastHLine(x, y, w, color);
  tft.drawFastHLine(x, y + h - 1, w, color);
  tft.drawFastVLine(x, y, h, color);
  tft.drawFastVLine(x + w - 1, y, h, color);
}


//-----------------------------------------------------------------------------
// DrawCheckBox  line 1602
//   draw a CheckBox on the main menu screen
//-------------------------------------------------------------------------

void DrawCheckBox(uint8_t class_nr, int Left, char *str, bool checked, uint16_t color) {

  const int Height    =  97;
  const int BoxWidth  =  33;
  const int BoxHeight =  33;
  const int Top       = 135;                         
  const int Width     =  75;                        
  const int BoxLeft = (Width - BoxWidth) / 2;  
  const int BoxTop = 52;
  if (s_func == 1) drawFunc(1,1,60,"DrawCheckBox",ILI9341_WHITE);  //####
 
 
  DrawFrame(Left,Top,Width,Height,ILI9341_WHITE);  // ### di left,top,width,height,color

  tft.fillRect(Left + 2, Top + 2, Width - 4, Height - 4, color); // DrawBox
 
  DrawFrame(Left, Top, Width, Height, ILI9341_WHITE);
 
  tft.fillRect(Left + 2, Top + 2, Width - 4, Height - 4, color);

  DrawFrame(Left + BoxLeft, Top + BoxTop, BoxWidth, BoxHeight, ILI9341_BLACK);
 
  tft.fillRect(Left + BoxLeft + 2, Top + BoxTop + 2, BoxWidth - 4, BoxHeight - 4, ILI9341_WHITE);

  tft.setTextColor(ILI9341_BLACK); 
  tft.drawString(str,Left + 14,Top ,2); // 18  
  if      (class_nr == 4)        { // "DIODE " 
         drawDiode('p',10,148,ILI9341_RED);
         drawDiode('n',30,148,ILI9341_BLUE);
  }  
  else if (class_nr == 1)        { // "PNP NPN" 
    drawTransistor('p',82,148,ILI9341_RED); 
    drawTransistor('n',114,148,ILI9341_BLUE); 
  }
  else if (class_nr == 2)       {  // "MOSFET" 
         drawMosFet('p',163,148,ILI9341_RED);
         drawMosFet('n',195,148,ILI9341_BLUE);
  }  
  else if (class_nr == 3)       {  // "JFET " 
         drawJFet('p',250,148,ILI9341_RED);
         drawJFet('n',280,148,ILI9341_BLUE);
  }  
  
   
  if (checked) {
    if      (CurDUTclass == tcBIPOL)  {
      class_cur = 1;
      drawCheckCross(class_old,0); 
      drawCheckCross(class_cur,1); 
      class_old = 1;     
      if (s_first == 0) ExecSetupMenuBIPOL();  // ### test #################################
    }   
    else if (CurDUTclass == tcMOSFET) {
      class_cur = 2;
      drawCheckCross(class_old,0); 
      drawCheckCross(class_cur,1); 
      class_old = 2;
      if (s_first == 0) ExecSetupMenuFET();      // ### test
    }  
    else if (CurDUTclass == tcJFET)  {
      class_cur = 3;
      drawCheckCross(class_old,0); 
      drawCheckCross(class_cur,1); 
      class_old = 3;
      if (s_first == 0)  ExecSetupMenuFET();      // ### test
    }  
    else if (CurDUTclass == tcDIODE) {
      class_cur = 4;
      drawCheckCross(class_old,0); 
      drawCheckCross(class_cur,1); 
      class_old = 4;
      if (s_first == 0)  ExecSetupMenuDiode();    // ### test ######################
    }  
  } 
  
  
}


//-------------------------------------------------------------------------
// drawSocket   line 1407
//   draw socket depending of DUT
//-------------------------------------------------------------------------

void drawSocket(void) {
  
 
    if (s_func == 1) drawFunc(1,1,60,"DrawSocket",ILI9341_WHITE);  //####
     tft.fillRect(132,4,112,24,ILI9341_BLACK);   // ### 155
  
    tft.fillRect(80,50,236,82,ILI9341_BLACK);   // ### 155
  
    tft.setTextColor(ILI9341_AQUA);
    
    if (CurDUTclass == tcBIPOL) {
     
      tft.drawString("PNP/NPN",97+35,6,4);
  
      tft.setTextColor(ILI9341_WHITE);
      tft.drawString("Min I-Base",82/*140*/,56/*40*/,2);
      tft.drawString("Inc I-Base",82/*140*/,81/*80*/,2);
      tft.drawString("Max I-Base",82/*140*/,106/*80*/,2);
      tft.drawString("Emitter  ",242,50,2);
      tft.drawString("Collector",242,70,2);
      tft.drawString("Base     ",242,90,2); 
      tft.drawString("Emitter  ",242,110,2);
      tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###
      tft.setTextColor(ILI9341_CYAN);
  //    tft.drawNumber(minNPNa,180,56,4);   
  //    tft.drawNumber(incNPNa,180,81,4); 
      tft.drawNumber(c2_BIP,180,56,4);   
      tft.drawNumber(c3_BIP,180,81,4); 
      tft.drawNumber(c4_BIP,180,106,4); 
      tft.setTextColor(ILI9341_WHITE);
    }
     
    else if (CurDUTclass == tcDIODE) { 
      tft.drawString("    DIODE",35+97,6,4);
      tft.setTextColor(ILI9341_WHITE);
      tft.drawString("Max I-Anode",82/*140*/,56,2);
      tft.drawString("Max U-Diode",82/*140*/,81,2);
      tft.drawString("Cathode",242,50,2);
      tft.drawString("Anode  ",242,70,2);
      tft.drawString("       ",242,90,2); 
      tft.drawString("Cathode",242,110,2);
      tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###
      tft.setTextColor(ILI9341_CYAN);
  //    tft.drawNumber(maxDIOa, 180,56,4);   
  //    tft.drawNumber(incDIOa, 180,81,4); 
      tft.drawNumber(c2_DIO,180,56,4);   
      tft.drawNumber(c3_DIO,180,81,4); 
 //     if (incDIOa == 0) s_secx = 1;
 //     if (c3_DIO  == 0) s_secx = 1;
      tft.setTextColor(ILI9341_WHITE);
   
    }
    else {                                           // MOSFET / jFET
      tft.setTextColor(ILI9341_WHITE);
      if (CurDUTclass == tcMOSFET)                   // MOSFET 
        tft.drawString("Min V-gs",82/*140*/,56,2);         // mod. from 2018.11.16
      else                                           // jFET 
        tft.drawString("Max V-gs",82/*140*/,56,2);         // mod. from 2018.11.16  
      tft.drawString("Inc V-gs",82/*140*/,81,2);
      if (CurDUTclass == tcMOSFET)                   // MOSFET 
        tft.drawString("Max V-gs",82/*140*/,106,2);
      else  
        tft.drawString("Min V-gs",82/*140*/,106,2);     
      if (CurDUTclass == tcMOSFET)  {                // MOSFET 
        tft.setTextColor(ILI9341_AQUA);
    
        tft.drawString(" Mos-FET",97+35,6,4);
        tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###
   //     drawDec1(minMOSa,180,56,4,ILI9341_CYAN);   
   //     drawDec1(incMOSa,180,81,4,ILI9341_CYAN);    
        drawDec1(c2_MOS,180,56,4,ILI9341_CYAN);   
        drawDec1(c3_MOS,180,81,4,ILI9341_CYAN);
        drawDec1(c4_MOS,180,106,4,ILI9341_CYAN);
        tft.setTextColor(ILI9341_WHITE);
     
      }  
      else {                                         // jFET 
        tft.setTextColor(ILI9341_AQUA);
        tft.drawString("    J-FET",97+35,6,4);
        tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###
    //    drawDec1(minFETa,180,56,4,ILI9341_CYAN);  
    //    drawDec1(incFETa,180,81,4,ILI9341_CYAN);  
        drawDec1(c2_FET,180,56,4,ILI9341_CYAN);   
        drawDec1(c3_FET,180,81,4,ILI9341_CYAN);
        drawDec1(c4_FET,180,106,4,ILI9341_CYAN);
        tft.setTextColor(ILI9341_WHITE);
    
      }
      tft.drawString("Source",242,50,2);
      tft.drawString("Drain ",242,70,2);
      tft.drawString("Gate  ",242,90,2); 
      tft.drawString("Source",242,110,2);
    //  ILI9341SetCursor(230,yp);    DrawString("Source", 4, ILI9341_WHITE); 
    //  ILI9341SetCursor(230,yp+20); DrawString("Drain ", 4, ILI9341_WHITE); 
    //  ILI9341SetCursor(230,yp+40); DrawString("Gate  ", 4, ILI9341_WHITE); 
    //  ILI9341SetCursor(230,yp+60); DrawString("Source", 4, ILI9341_WHITE); 
    }
    
     tft.setTextColor(ILI9341_YELLOW);
     
    if (CurDUTclass == tcDIODE) {
      tft.drawNumber(4,306,50,2);
      tft.drawNumber(3,306,70,2);
      tft.drawString("  ",306,90,2);
      tft.drawNumber(1,306,110,2);
     /*
      ILI9341SetCursor(306,yp+6);  DrawInt(4, 4, ILI9341_YELLOW);  // ### e
      ILI9341SetCursor(306,yp+26); DrawInt(3, 4, ILI9341_YELLOW);  // ### e 
      ILI9341SetCursor(306,yp+46); DrawString("     ", 4, ILI9341_WHITE); 
      ILI9341SetCursor(306,yp+66); DrawInt(1, 4, ILI9341_YELLOW);  // ### e
      */
    }  
    else {
      tft.drawNumber(4,306,50,2);
      tft.drawNumber(3,306,70,2);
      tft.drawNumber(2,306,90,2);
      tft.drawNumber(1,306,110,2);
    
      /*
      ILI9341SetCursor(306,yp);    DrawInt(4, 4, ILI9341_YELLOW);  // ### e
      ILI9341SetCursor(306,yp+20); DrawInt(3, 4, ILI9341_YELLOW);  // ### e
      ILI9341SetCursor(306,yp+40); DrawInt(2, 4, ILI9341_YELLOW);  // ### e
      ILI9341SetCursor(306,yp+60); DrawInt(1, 4, ILI9341_YELLOW);  // ### e 
      */
    }
    tft.setTextColor(ILI9341_WHITE); 
}



//-------------------------------------------------------------------------
// DrawMenuScreen  line 1766
//   draw the main menu screen
//-------------------------------------------------------------------------

void DrawMenuScreen(void) {
  if (s_func == 1) drawFunc(1,1,80,"DrawMenuScreen",ILI9341_WHITE);  //####
  //DrawBox(1,1,90,12, ILI9341_BLACK);  DrawStringAt(4,8,"drawSocket", SmallFont, RGB(28, 128, 255));  //##########
  s_trace = 0;
  if (s_first == 1) {
    tft.fillScreen(ILI9341_BLACK); // ClearDisplay(ILI9341_BLACK);
    
  // DrawKindStr(tkNothing);
    tft.setTextColor(ILI9341_YELLOW);
    tft.drawString("Curve Tracer",(TFT_WIDTH - 117) / 2,5,1);
   
    DrawCheckBox(4,DiodeLeft,  " DIODE ",  CurDUTclass == tcDIODE,   tft.color565(0xFF, 0x80, 0x10));   // orange
       
    DrawCheckBox(1,BIPOLLeft, "PNP NPN",   CurDUTclass == tcBIPOL,   tft.color565(0xFF, 0xFF, 0x80));   // gelb
   
    DrawCheckBox(2,MOSFETLeft, "MOSFET",   CurDUTclass == tcMOSFET,  tft.color565(0x80, 0xFF, 0xFF));   // hellblau
   
    DrawCheckBox(3,JFETLeft,   " J-FET",   CurDUTclass == tcJFET,    tft.color565(0xFF, 0x80, 0xFF));   // violett
    drawSocket();     // dependent from CurDUTclass
/*
  //===================================================================================================
    DrawFrame(OKLeft, OKTop, OKWidth, OKHeight, ILI9341_WHITE);         // paint "START"-box
    tft.fillRect(OKLeft + 2, OKTop + 2, OKWidth - 4, OKHeight - 4, ILI9341_ORANGE);
    tft.drawString("START",OKLeft + 4,OKTop+8,4);  // "SAVE"
    
  //====================================================================================================
    */
    int x_a = 80; // 215
    tft.setTextColor(ILI9341_RED);
    DrawFrame(x_a, 0, 47, 47, ILI9341_WHITE);         // paint "P"-"START"-box
    tft.fillRect(x_a+2, 0 + 2, 47-4, 47 - 4, ILI9341_ORANGE);
    tft.drawString(" P ",x_a + 9/*13*/,12,4);  //
    tft.setTextColor(ILI9341_BLUE); 
    x_a = 250; // 270
    DrawFrame(x_a, 0, 47, 47, ILI9341_WHITE);         // paint "N"-"START"-box
    tft.fillRect(x_a+2, 0 + 2, 47-4, 47 - 4, ILI9341_ORANGE);
    tft.drawString(" N ",x_a + 8/*13*/,12,4);  // 
    s_first = 0;
    tft.setTextColor(ILI9341_WHITE); 
   
    
  /* 
    if      (CurDUTclass == tcBIPOL) ExecSetupMenuBIPOL();
    else if (CurDUTclass == tcDIODE)   ExecSetupMenuDiode();
    else if (CurDUTclass == tcMOSFET)  ExecSetupMenuFET();
    else if (CurDUTclass == tcJFET)    ExecSetupMenuFET();
    s_first = 0;
    }
  */    
  } 
  else {
    if (class_old > 0) drawCheckCross(class_old,0); // erase cross for old_class
    drawCheckCross(class_cur,1);                    // paint cross for cur_class
    tft.drawRect(95,61,225,71,ILI9341_BLACK);       // erase old data 36,,96
    drawSocket();     // dependent from CurDUTclass
    /*
    if      (CurDUTclass == tcBIPOL)   ExecSetupMenuBIPOL();
    else if (CurDUTclass == tcDIODE)   ExecSetupMenuDiode();
    else if (CurDUTclass == tcMOSFET)  ExecSetupMenuFET();
    else if (CurDUTclass == tcJFET)    ExecSetupMenuFET();
    */
  }  
 //delay(5000);
// draw Battery Voltages ( but battery is not connected to A6-ADC6 !!! #################
//  DrawStringAt(4,  15, "Bat ", 4, RGB(128, 128, 255));
//  DrawStringAt(4,  15, "    ", 4, RGB(128, 128, 255));
// DrawDecimal(BattVolts(), 4, RGB(128, 128, 255));
}




//-------------------------------------------------------------------------
// TestDevNeg
//    is there a DUT inserted?    returns kind of device
//    
//-------------------------------------------------------------------------

TkindDUT TestDevNeg(TclassDUT clasx) {
  TkindDUT kind;
  kind = tkNothing;
 // s_xfactor = 1;
  switch (clasx) {
    case tcBIPOL:     kind = tkPNP;        SetDacBase(dmax, 0);      SetDacVcc(dmax, 0);   break; 
    case tcMOSFET:    kind = tkPMOS;       SetDacBase(dmax, 0);      SetDacVcc(0, 0);      break;   // dhalf ???
    case tcJFET:      kind = tkPJFET;      SetDacBase(dmax, 0);      SetDacVcc(0, 0);      break;   // ??JF base +12...< +24V
    case tcDIODE:     kind = tkPDIODE;     SetDacBase(0, 0);         SetDacVcc(0, 0);      break; 
  // return kind; 
  }
  return kind; 

}



//-------------------------------------------------------------------------
// TestDevPos
//    is there a DUT inserted?    returns kind of device
//    
//-------------------------------------------------------------------------

TkindDUT TestDevPos(TclassDUT clasx) {
  TkindDUT kind;
  kind = tkNothing;
 
  switch (clasx) {
    case tcBIPOL:     kind = tkNPN;        SetDacBase(dmax, 1);       SetDacVcc(0, 1);     break; 
    case tcMOSFET:    kind = tkNMOS;       SetDacBase(dmax, 0);      SetDacVcc(0, 0);      break;   // dhalf??
    case tcJFET:      kind = tkNJFET;      SetDacBase(dhalf, 0);      SetDacVcc(0, 0);     break;   // ??JF base -0...-12V
    case tcDIODE:     kind = tkNDIODE;     SetDacBase(0, 0);          SetDacVcc(dmax, 0);    break; 
//   return kind; 
  }
  return kind; 

}





//-----------------------------------------------------------
void initEncoders()    // madias
//-----------------------------------------------------------
{
  encodertimer = millis(); // acceleration measurement
  for (byte counter = 0; counter < MAXENCODERS; counter++)
  {
    encstate[counter] = HIGH;
    encflag[counter] = HIGH;
    A_set[counter] = false;
    B_set[counter] = false;
    encoderpos[counter] = 0;
    pinMode(encoderpinA[counter], INPUT_PULLUP);
    pinMode(encoderpinB[counter], INPUT_PULLUP);
    lastEncoderPos[counter] = 1; // test
  }
   lastEncoderPos[1] = 0;  // test for frame timeBase
  // timer setup for encoder
  timer.pause();
  timer.setPeriod(ENCODER_RATE); // in microseconds
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(readEncoders);
  timer.refresh();
  timer.resume();
  
}



//-----------------------------------------------------------
void readEncoders() {      // ********encoder function
//-----------------------------------------------------------  
  for (byte counter = 0; counter < MAXENCODERS; counter++)
  {
    if ( (gpio_read_bit(PIN_MAP[encoderpinA[counter]].gpio_device, PIN_MAP[encoderpinA[counter]].gpio_bit) ? HIGH : LOW) != A_set[counter] )
    {
      A_set[counter] = !A_set[counter];
      if ( A_set[counter] && !B_set[counter] )
      {
        if (millis() - encodertimer > 3)
           encoderpos[counter] += 1;
    
        else
          encoderpos[counter] += 1;//5;without accelleration
      }
      encodertimer = millis();
    }
    if ( (gpio_read_bit(PIN_MAP[encoderpinB[counter]].gpio_device, PIN_MAP[encoderpinB[counter]].gpio_bit) ? HIGH : LOW) != B_set[counter] )
    {
      B_set[counter] = !B_set[counter];
      if ( B_set[counter] && !A_set[counter] )
        if (millis() - encodertimer > 3)
          encoderpos[counter] -= 1;
     
        else
         encoderpos[counter] -= 1;//5;
      encodertimer = millis();
    }
  }
  if (encoderpos[0] < 0)   encoderpos[0] = 0;    
  if (encoderpos[1] < 0)   encoderpos[1] = 0;     
  count2 = encoderpos[0];
  count3 = encoderpos[1]; 
 }

// ------------------------
 void btn0ISR()   {     // 2. run with same kind or complementaer
// ------------------------
  // debounce
  //static long pressedTime = 0;
  if(millis() - lastBtnPress < BTN0_DEBOUNCE_TIME)
    return;
  if(millis() - lastBtnPress < 1000) {  // 1000 // is it a short press  
   
    s_comp++;
    if (s_comp > 1) s_comp = 0;         // only 0 , 1 and 2
  
 //   if (s_comp == 0) {
 //     tft.fillRect(300,0,20,20,ILI9341_BLACK);
 //     return;
 //   }  
    tft.fillRect(300,0,20,20,ILI9341_WHITE); 
    tft.setTextColor(ILI9341_GREEN); 
    tft.drawNumber(s_comp,303,1,4);
    tft.setTextColor(ILI9341_WHITE);  
    lastBtnPress = millis();
    return;
  }                                 
 // else {   // select different parameters to change
 // else resetParam();      // long press reset parameter to default
  lastBtnPress = millis();
 
}



// ------------------------
void changeXCursor(int16_t xPos)  {
// ------------------------
  /*
  tft.drawRect(x_p[3],y_p[3],x_w[3],y_w[3], ILI9341_AQUA);   
  tft.fillRect(x_p[3]+1,2, x_w[3]-2,14, ILI9341_BLACK); // ################## test #######
   tft.setCursor(x_p[3]+1,4);  tft.print(xPos);
   */
 
}


//-------------------------------------------------------------------------
// Aver4_ADC
// Smoot  mean of N readings of ADC
//-------------------------------------------------------------------------

inline int Aver4_ADC(int pin) {
 
  
  int sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return sum = sum >> 2;  // /4  
}


//-------------------------------------------------------------------------
// Aver8_ADC
//   mean of N readings of ADC
//-------------------------------------------------------------------------

int Aver8_ADC(int pin) {
   
  const int n   = 8; // 4 x ,8 x
  const int ip3 = 3; // 2 exp 3 = 8
  uint8_t i;
  int sum=0;
  for (i = 1; i <= n; i++) {
    sum += analogRead(pin);
  }
  return sum = sum >> ip3;  // /8 
}




void pinchJFET(long ixu,long iyi) { // ixu=..1...319 = 12V
  static float fra=0.0,fro=0.0;
  static int ia = 0;
  float fr,fu,fi;
  int ir,iu;
  if (s_tend == 1)  return;
  //fu = 25.806*(float)ixu;                      // U in mV
  fu = 12.8*(float)ixu;                        // U in mV 4095/320
  if (s_xfactor > 1) fu = fu/(float)s_xfactor;
 // fi = 0.2833*(float)iyi;    // I in mA
  fi = 0.20833*(float)iyi;   // I in mA
  if (s_yfactor > 1) fi = fi/(float) s_yfactor;
  fr = fu/fi;                                  // R in Ohm;
  /*
  tft.setTextColor(ILI9341_ORANGE);
  tft.fillRect(260,100,70,50,ILI9341_BLACK);
  tft.drawFloat(fra,1,260,100,2);  // 1.
  tft.drawFloat(fro,1,260,115,2);  // pre
  tft.drawFloat(fr,1,260,130,2);   // act
  tft.drawNumber(pinch_v,260,1453,2);
  delay(200);
  tft.setTextColor(ILI9341_WHITE);
  */
// tft.drawNumber((int)fr,80,0,2);    // resistance
  if (ixu <= 10)  fra = fr;           // start-resistance
   
  if (fr > fra) {                              // trace more flat , resistance bigger
    /*
    tft.fillRect(40,20,160,13,ILI9341_BLACK);
    tft.drawFloat(fi,1,40,20,2);
    tft.drawFloat(fu,1,80,20,2);
    tft.drawFloat(fra,1,125,20,2);
    tft.drawFloat(fr,1,160,20,2);
    delay(100);
   */ 
    if (fr >(5 * fra)) {              
      s_pinch  = 1;
      s_vgate0 = 1;
      s_tend   = 1;
      pinch_v = (int)(5.0+fu*120.0/4095.0);  // 100mV
      ILI9341_TraceCol = ILI9341_AQUA;
      return;
    }
  }  
     
}   


 

//-------------------------------------------------------------------------
// Graph_P
//   Vcc, Vce in ADC counts
//   base in uA
//-------------------------------------------------------------------------

void Graph_P( bool isNPN, int Vcc, int Vce, int base, int Adc_12V) {    // true,iu0,iu1
  long i, j,j0,j1,j2,j3,jy0,jy1,jy2,jy3;
  static int px, py;
  
 // if (s_func == 1) drawFunc(1,1,60,"Graph",ILI9341_WHITE);  //#### to slow
  tft.setTextColor(ILI9341_GREEN);
  
  if (isNPN)     {   // Scan_n_Dev: npn, nDiode, n_mos, n_FET
    i = Vce;              // A1
  /*  
    if (s_dio == 1) {
      if (i >= ivxa) ivxa = i;
      else i = ivxa;
    }
    */
    j = Vcc - Vce;        // A0 - A1     // U at R37
  } 
  else              {   // Scan_p_Dev: pnp, pDiode, p_mos, p_FET 
    i = Adc_12V - Vce;    // A3 - A2
    if (s_dio == 1) {
      if (i >= ivxa) ivxa = i;
      else i = ivxa;
    }
    j = Vce - Vcc;        // A2 - A0 // U at R32
  }
  j0 = j;
      // U / x-Achse   
  i = (TFT_WIDTH ) * i / ADC_MAX;         // U / x-Achse    1 ... 320

  if (zen_v == 24) i = i * 30/25;  // voltage divider max. 29.7V, display 25V 

       

  //if (j == 0) j = 239;     // ??? ##########################################################################
 // else if (j > 239) j = 0; //239;    // max
 // else if (j < 0) return;
 if (j < 0) return;

 // Serial.print("x ");  Serial.print(i);Serial.print(" y ");  Serial.print(j); Serial.print(" j0 ");  Serial.print(j0);  Serial.print(" A1 ");  Serial.println(Vce);   


 

  if (i == i_o) {  // same x-point  // ### new test
    i_n++;
    i_j = i_j + j0;
    return;
  }
  if (i_n > 0) {
    i_o = i;
    j = i_j/i_n;
    i_n = 0;
    i_j = 0;
  }  

  if ( j > 0 )   {   // Z 
   
 // j = volt diff. on R3 (R32/R37): Uout/A0 -- A1/Vce_npn bzw. A2/Vce_pnp , 0...4095 12-bit-DAC//0...1023 10-bit-DAC//0...255 8-bitDAC
  
 // j1 = j * (R2 + R1) * 33000 / R3 / R1 / ADC_MAX; // amax?? convert j to 100s of uA   // j=volt on R3=100 Ohm, R1=33k,R2=100k  // ### test   ???
    j1 = j * 0.3248;   // 0.03284
    if (zen_v == 24) j1 = j1 * 2.3;
    j2 = j1;
    if (s_secy == 2)  j2 = j1 * s_yfactor;                 // ############################### idss <= 10mA factor=5
    if (s_secy == 3)  j2 = (int)(0.5 + float(j1)/2.0);     // ############################### idss > 100mA factor=0.5=1/2
    j3 = TFT_HEIGHT - 1 - TFT_HEIGHT * j2 / (mAmax * 10);  // mAmax = 50
    j = j3;
   
  }
  else j = 239; 
  if (s_xfactor > 1) i = i * s_xfactor;                   // else ?? Z??
    
//  Serial.print("x ");  Serial.print(i);Serial.print(" y ");  Serial.print(j); Serial.print(" j0 ");  Serial.print(j0);  Serial.print(" A1 ");  Serial.println(Vce);   
  if (isNPN == true)  {
    if (( i >= i_o) && (cntv > 10))      {                // 100for pJFET ntv > 3 !!
       tft.drawPixel( i, j,ILI9341_TraceCol);
          
    }     
  }
  else  {
    if (( i >= i_o) && (cntv > 90))      {                // 100for pJFET ntv > 3 !!
       tft.drawPixel( i, j,ILI9341_TraceCol);
 
    }     
  }
 
  i_o = i;
  j_o = j;
  if ((prev_x <= TFT_WIDTH / 4) and (i > TFT_WIDTH / 4) and (j < TFT_HEIGHT - TFT_HEIGHT / 20)) {
    if (maxYposGain < 0) {
      minYposGain = j;
      minBaseGain = base;
    }
    maxYposGain = j;
    maxBaseGain = base;
  }
  
  if (s_jfet == 1)     {
   
    if (s_vgate0 == 0)   {       // Vgate = 0 
      if (s_tend == 0)  pinchJFET(i,j);          // pinch_off-vol
    }
  }  
  if (j == prev_y)  {
    if (s_secy == 0) {
      if ((j > TFT_HEIGHT - 48) && (i > 260))  {   // 192
        s_secy = 1;
      }  
      else {
        if (j <= 1)        {   // 4 : IDss > 50 mA
          s_secy = -1;     
          return;
        }
      }     
    }
  }
  prev_x = i;
  prev_y = j;
}







//-------------------------------------------------------------------------
// GraphZ
//   Vcc, Vce in ADC counts
//   base in uA
//-------------------------------------------------------------------------

void GraphZ( bool isNPN, int Vcc, int Vce, int base, int Adc_12V) {    // true,iu0,iu1
  long i, j,j0,j1,j2,j3,jy0,jy1,jy2,jy3;
  static int px, py;

 // if (s_func == 1) drawFunc(1,1,60,"Graph",ILI9341_WHITE);  //#### to slow
  tft.setTextColor(ILI9341_GREEN);
  /*
  tft.fillRect(260,120,80,53,ILI9341_BLACK);
  tft.drawNumber(Vcc,260,120,2);
  tft.drawNumber(Vce,260,133,2);
  tft.drawNumber(base,260,146,2);
  tft.drawNumber(Adc_12V,260,159,2);
  delay(10);
  */
 
  if (isNPN)     {   // Scan_n_Dev: npn, nDiode, n_mos, n_FET
    i = Vce;              // A1
    
    if (s_dio == 1) {
      if (i >= ivxa) ivxa = i;
      else i = ivxa;
    }
    
    j = Vcc - Vce;        // A0 - A1     // U at R37
  } 
  else              {   // Scan_p_Dev: pnp, pDiode, p_mos, p_FET 
    i = Adc_12V - Vce;    // A3 - A2
    if (s_dio == 1) {
      if (i >= ivxa) ivxa = i;
      else i = ivxa;
    }
    j = Vce - Vcc;        // A2 - A0 // U at R32
  }
      // U / x-Achse   
  i = (TFT_WIDTH ) * i / ADC_MAX;         // U / x-Achse    1 ... 320

  if (zen_v == 24) i = i * 30/25;  // voltage divider max. 29.7V, display 25V 

//  j2 = j * 0.03248;
//  tft.drawFloat(j2,3,80,135,2);      // I / y-Achse 
  
 
       
  if ( j > 0 )   {   // Z 
   
 // j = volt diff. on R3 (R32/R37): Uout/A0 -- A1/Vce_npn bzw. A2/Vce_pnp , 0...4095 12-bit-DAC//0...1023 10-bit-DAC//0...255 8-bitDAC
  
 // j1 = j * (R2 + R1) * 33000 / R3 / R1 / ADC_MAX; // amax?? convert j to 100s of uA   // j=volt on R3=100 Ohm, R1=33k,R2=100k  // ### test   ???
    j1 = j * 0.3248;   // 0.03284
    if (zen_v == 24) j1 = j1 * 2.3;
    j2 = j1;
    if (s_secy == 2)  j2 = j1 * s_yfactor;                 // ############################### idss <= 10mA factor=5
    if (s_secy == 3)  j2 = (int)(0.5 + float(j1)/2.0);     // ############################### idss > 100mA factor=0.5=1/2
    j3 = TFT_HEIGHT - 1 - TFT_HEIGHT * j2 / (mAmax * 10);  // mAmax = 50
    j = j3;
   
  }

  else if (j == 0) j = 239;     // ground-line  ################
  else if (j  < 0) return;
  if (j > 239) j = 239;    // max
// Serial.print("x ");  Serial.print(i);Serial.print(" y ");  Serial.print(j); Serial.print(" A0 ");  Serial.print(Vcc);  Serial.print(" A1 ");  Serial.println(Vce);   

  if (i == i_o) {  // same x-point  // ### new test
    i_n++;
    i_j = i_j + j;
    return;
  }
  if (i_n > 0) {
    i_o = i;
    j = i_j/i_n;
    i_n = 0;
    i_j = 0;
  }  



  px >> px;     //   px = px / 2 + i;
  px = px + i;
 
  py >> py;     //   py = py / 2 + j; 
  py = py + j;
 
  i >> px;      //  i = px / 2;
  
  j >> py;      //  j = py / 2;

    
 // tft.fillRect(260,75,60,12,ILI9341_BLACK);  
 // tft.drawNumber(i,260,75,2);       // source   
 // tft.drawNumber(j,290,75,2);       //  
    
  if (s_xfactor > 1) i = i * s_xfactor;                   // else ?? Z??
  if (isNPN == true) {
    if (( i >= prev_x) && (cntv > 3))                     // cntv > 3 !!
      tft.drawLine(prev_x, prev_y, i, j,ILI9341_TraceCol);   
  }
  else 
    if (( i >= prev_x) && (cntv > 90))                    // 100for pJFET ntv > 3 !!
      tft.drawLine(prev_x, prev_y, i, j,ILI9341_TraceCol);  
 
 
/*
  if ( cntv < 50) {
    tft.fillRect(80,1,220,30,ILI9341_BLACK);
    tft.drawNumber(prev_x,80,1,2);
    tft.drawNumber(prev_y,80,14,2);
    tft.drawNumber(i,130,1,2);
    tft.drawNumber(j,130,14,2); 
    delay(500);
  }
  */
  if ((prev_x <= TFT_WIDTH / 4) and (i > TFT_WIDTH / 4) and (j < TFT_HEIGHT - TFT_HEIGHT / 20)) {
    if (maxYposGain < 0) {
      minYposGain = j;
      minBaseGain = base;
    }
    maxYposGain = j;
    maxBaseGain = base;
  }
  
  if (s_jfet == 1)     {
   
    if (s_vgate0 == 0)   {       // Vgate = 0 
      if (s_tend == 0)  pinchJFET(i,j);          // pinch_off-vol
    }
  }  
  if (j == prev_y)  {
    if (s_secy == 0) {
      if ((j > TFT_HEIGHT - 48) && (i > 260))  {   // 192
        s_secy = 1;
      }  
      else {
        if (j <= 1)        {   // 4 : IDss > 50 mA
          s_secy = -1;     
          return;
        }
      }     
    }
  }
  prev_x = i;
  prev_y = j;
}








//-------------------------------------------------------------------------
// ScanKind
//   draw curves for a component, not for p-JFET
//-------------------------------------------------------------------------
void ScanKind(TkindDUT kind) {
  int minBase, maxBase, incBase, Adc_12V;
  //TkindDUT lastkind;  // from outside
  TkindDUT kind_s = kind;  // save kind !!!!!!!!! ############################## look 10 lines forward ###################
  
  if (s_func == 1) drawFunc(1,1,60,"ScanKind",ILI9341_MAGENTA);  //####

  Adc_12V = Aver4_ADC(pin_ADC3_Bat_12V);  //#######################
  
 // tft.fillRect(90,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE);tft.drawNumber(kind,90,0,2);   delay(2000);    
     
  InitGraph();

  kind = kind_s;  // ##############################################################################
  
  // tft.fillRect(100,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_GREEN);tft.drawNumber(kind,100,0,2);  
  DrawKindStr(kind); 

  //* tft.fillRect(110,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_YELLOW);tft.drawNumber(kind,110,0,2);  
  ILI9341_BASE = ILI9341_WHITE; // first run 
  
  s_save  = 0;

 comp2:
 
  switch (kind) {
    case tkPNP:
    case tkNPN:
      minBase = MinIbase;  // 10 : 50/5
      maxBase = c4_BIP;       // 400 ##################MaxIbase * 10 / 50;  // 70 : 350/5
      incBase = IncIbase;
     break;

    case tkNMOS:
    case tkPMOS:
      minBase = MinV_MOS;  // MinVgate; // * 10; ###
      incBase = IncV_MOS;  // IncVgate; // * 10; ###
      maxBase = c4_MOS;   // 120;       // * 10; ###  ??
     break;
  
    case tkNJFET:
    case tkPJFET:
      minBase = MinV_FET;  // MinVgate; // * 10;
      incBase = IncV_FET;  // IncVgate; // * 10; ###
      maxBase = c4_FET;  // 120;       // MaxVgate * 10;
     break;
     
    case tkNDIODE:         // ## di ???
    case tkPDIODE:         // ## di ???
      minBase = MinIanode; // ## 5 di ??
      maxBase = MaxIanode; // ## di ?? 
      incBase = IncIanode; // ## di ??
     break; 
  }
    
comp1:
  switch (kind) {
    case tkPNP:     
      Scan_p_Dev(kind, minBase, maxBase, incBase,  Adc_12V);
     break;
    case tkNPN: 
      Scan_n_Dev(kind, minBase, maxBase, incBase); 
     break;

    case tkPMOS: 
      Scan_p_Dev(kind, minBase, maxBase, incBase,  Adc_12V);
     break;
    case tkNMOS:   
      Scan_n_Dev(kind, minBase, maxBase, incBase); 
     break;
  
    case tkNJFET:  
      Scan_n_Dev(kind, minBase, maxBase, incBase);   
     break;
    case tkPJFET: 
      Scan_p_Dev(kind, minBase, maxBase, incBase, Adc_12V); 
     break;

    case tkPDIODE:  
      Scan_p_Dev(kind, minBase, maxBase, incBase,  Adc_12V);
     break;
      
    case tkNDIODE: 
      Scan_n_Dev(kind, minBase, maxBase, incBase);  
     break; 
      
  }

   
  
//  tft.fillRect(0,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE);tft.drawNumber(0,0,0,2); 
 
  long count3a = count3;
  while (1 == 1)      {  // wait here in the in ScanKind(kind) until touched or compare 
    if (HaveTouch())  {
      digitalWrite(BTN19, LOW); 
      digitalWrite(BTN20, LOW); 
      return; 
    }
    if (count3 != count3a) {
      if (count3 > count3a) {
        ILI9341_TraceCol = ILI9341_GREEN;
        ILI9341_BASE     = ILI9341_GREEN; 
    //    kind = lastkind; 
      // s_yfactor = 1;
        s_endgr = 0;
        s_trace = 1;
      //  s_save  = 1;
     //   s_save  = 0;
  //   tft.fillRect(0,13,60,40,ILI9341_BLACK); tft.setTextColor(ILI9341_GREEN);
  //   tft.drawNumber(count3a,0,13,2); 
  //   tft.drawNumber(count3,0,26,2); 
  /*   tft.drawNumber(lastkind,20,13,2);*/  //tft.drawNumber(kind,40,13,2);
  //   tft.drawNumber(curkind,20,26,2);
  //delay(2000);
        goto comp1; 
      }
      else {
        complemKind(kind);   //  complementaer kind ########################################################
        kind    = curkind;     // compl. kind
     //   curkind = lastkind;
        s_yfactor = 1;
        s_trace   = 1;
        s_endgr   = 0;
        //   s_save  = 0;
        ILI9341_TraceCol = ILI9341_ORANGE;
        ILI9341_BASE     = ILI9341_ORANGE;
     //   tft.fillRect(300,0,20,40,ILI9341_BLACK); tft.setTextColor(ILI9341_ORANGE); 
     //   tft.drawNumber(count3a,0,13,2); 
     //   tft.drawNumber(count3,0,26,2); 
     /*   tft.drawNumber(lastkind,20,13,2);*/tft.drawNumber(kind,40,13,2);
     //   tft.drawNumber(curkind,20,26,2);
     //    delay(2000);
        goto comp2;
      }
    }
  }   
}

void complemKind(TkindDUT kind_o)  {     // change kind to complementaer

 TkindDUT kind;
  
  //enum TkindDUT {tkNothing, tkPNP, tkNPN, tkPMOS, tkNMOS, tkNJFET, tkPJFET, tkPDIODE, tkNDIODE};  // ### di
  //#############       0,        1       2      3       4         5        6        7         8

 //   if (s_exec_npn == 1) kind = kind_o - 1;  
 //   if (s_exec_pnp == 1) kind = kind_o + 1;
    
  if      (kind_o == tkPNP)    kind = tkNPN;
  else if (kind_o == tkNPN)    kind = tkPNP;
  else if (kind_o == tkPMOS)   kind = tkNMOS;
  else if (kind_o == tkNMOS)   kind = tkPMOS;
  else if (kind_o == tkPJFET)  kind = tkNJFET;
  else if (kind_o == tkNJFET)  kind = tkPJFET;
  else if (kind_o == tkPDIODE) kind = tkNDIODE;
  else if (kind_o == tkNDIODE) kind = tkPDIODE;
 
  curkind  = kind;                         //###########################
  //lastkind = kind;                         //###########################
  if (s_exec_npn == 1)  {       // last: Scan_n_Dev: npn,nMOS      
   
    s_exec_npn = 0;    // rel##
    s_exec_pnp = 1;    // rel##
 
   
    digitalWrite(BTN20, LOW);   // rel  // stop  Scan_n_Dev  
    delay(20);
    digitalWrite(BTN19, HIGH);  // rel  // start Scan_p_Dev : p-Diode,pnp,p_MOS,pJFET 
    delay(20);
    delay(20);
    return;
  }   
  if (s_exec_pnp == 1)  {       //  last: pnp == Scan_p_Dev 
   
    s_exec_npn = 1;    // rel##
    s_exec_pnp = 0;    // rel##
      
    digitalWrite(BTN19, LOW);   // rel  // stop  Scan_p_Dev: p-Diode, pnp, p_MOS, pJFET
    delay(20);
    digitalWrite(BTN20, HIGH);  // rel  // start Scan_n_Dev: n-Diode, npn, n_MOS, nJFET 
    delay(20);
    delay(20);
    return;
  } 
      
  
                 
}


void curLimit()  {
  
  for (uint8_t idr = 0;idr < 6;idr++)  {
    tft.fillRect(25,30,122,16,ILI9341_ORANGE);
    tft.setTextColor(ILI9341_RED);
    tft.drawString("max-current limit !",30,30,2); 
    delay(180);
    tft.fillRect(25,30,122,16,ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.drawString("max-current limit !",30,30,2); 
    delay(120);
  }
  
}




//-------------------------------------------------------------------------
// Scan_p_Dev
//   draw curves for a component on the PNP side of the DUT socket : PN-Diode,PNP,P-MOS,p-JFET
// 
//   if user touches the screen at the end of a line, the scan is terminated and the function returns true
//-------------------------------------------------------------------------

void Scan_p_Dev(TkindDUT kind, int minBase, int maxBase, int incBase, int Adc_12V) {
  int DacVcc;      // collector voltage PNP ADC 2
  int incv = 1;    // 1 increment DACVcc  // 2??
  int base,delay_d,zen_a=0;
  int id,idr,zw_maxb,zw_minb,zw_incb;     // max 4095  ### d
  uint8_t s_blink = 0,s_kindcur=0,s_zencur=0;
  float fim=0,fimax,fu0,fu2,fu3,fud,fum,fim_pmos=0.5;
  float idy1,idy2,idy3,idy4,idy5;
 // tft.fillRect(0,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE);tft.drawNumber(1,0,0,2);  
 
  if (s_func == 1) drawFunc(1,1,60,"Scan_p_Dev",ILI9341_WHITE);  //####
  digitalWrite(BTN0, LOW);         // 
  digitalWrite(BTN12, LOW);        // switch on positive base voltage (non-inverting amplifier) gain = +6  standard
  digitalWrite(BTN32, HIGH);       // 12V standard
        // 
  delay(1);
  i_o=0;
  i_j=0;
  i_n=0;
  j_o=0;
  fimax = 5.0001;        // 50 mA on 100 Ohm    // tft.drawFloat(fimax,1,70,0,2);
  f_thrp = 0;
  zw_maxb = maxBase; 
  zw_minb = minBase; // <<-----------------------------------------------------------------
  zw_incb = incBase;
  if (kind == tkPDIODE)   {       // current limit DIODE

    fim = 0.1 * (float)maxBase;   // tft.drawFloat(fim,1,120,0,2);
    if  ( maxBase <= 10 )  {  
      s_yfactor = 5;
      s_secy    = 2;
    }  
       
  }   
  if (kind == tkPJFET)   { 
    zw_maxb = minBase;   // change min - max 
    zw_minb = maxBase;   // change  max - min
    zw_incb = incBase;
    s_vgate0 = 0;
    s_jfet   = 1;
    minBase = 0;
    
  }
 
// s_secx = 0;
 
second_scan_pDev:
  s_kindcur = 0;
  s_zencur = 0; 
  s_blink  = 0;
  ivxa = 0;
  s_dio = 0;
  maxBase = zw_maxb;
  minBase = zw_minb;
  prev_x = 0;
  prev_y = 239;
 //tft.drawNumber(minBase,80,0,2);   // v-gate start
 // tft.drawNumber(maxBase,120,0,2);  // vgate (input) 120
  //tft.drawNumber(incBase,160,0,2);  // vgate (input) 120  
  iu3 = Aver4_ADC(pin_ADC3_Bat_12V);   // A3  <<<<< 12.240 V = 4095
  DacVcc = dmax;          //4095; // DAC_MAX - 1;       // max index dacdmax;  // 4095 1023 255;  <<< in setup 
  maxYposGain = -1;
  minBaseGain = 0;
  maxBaseGain = 0;
  
  if (kind == tkPDIODE) {
    incBase = 1;
    minBase = 1;
    if      (zen_v <=  6) {
      digitalWrite(BTN0,  LOW);  // HIGH: 24V 
   //   digitalWrite(BTN32, LOW);  // HIGH: 12V   // change U_DAC etc.
      digitalWrite(BTN32, HIGH);  // HIGH: 12V  ### test 
    }  
    else if (zen_v <= 12) {
      digitalWrite(BTN0,  LOW);  // HIGH: 24V 
      digitalWrite(BTN32, HIGH); // 12V standard
    }  
    else if (zen_v == 24)  { 
      digitalWrite(BTN0,  HIGH); // 24V for zener > 12V 
      digitalWrite(BTN32, LOW);  // HIGH: 12V
    }  
    delay(20);  // needs time to stabilize
    iuz = 0;
  }  
  if (kind == tkPJFET)   { 
    incBase = zw_incb;
    minBase = zw_minb; 
    maxBase = zw_maxb; 
    digitalWrite(BTN0,  LOW);   // HIGH: 24V only for diodes/zener TCA0372
    digitalWrite(BTN32, HIGH);  // HIGH: 12V standard for TCA0372

  }
/*
  tft.fillRect(1,1,110,12,ILI9341_BLACK); 
  tft.drawNumber(minBase,1,1,2);  //########## im
  tft.drawNumber(maxBase,1,40,2);  //########## im
  tft.drawNumber(incBase,1,80,2);  //########## im
  if (kind == tkPDIODE) incBase = 1;    // maxBase;
*/

 //  Serial.print("zen_v ");Serial.print(zen_v);Serial.print(" sec_x ");Serial.println(s_secx);
  
  for (base = minBase; base <= maxBase; base += incBase)   {
  //  tft.fillRect(70,0,100,13,ILI9341_BLACK);   // v-gate start
  //  tft.drawNumber(iu3,80,0,2);   // v-gate start
  //  tft.drawNumber(base,120,0,2);  // vgate (input) 120
  //  tft.drawNumber(maxBase,160,0,2);  // vgate (input) 120  
    cntv = 0;
    if (base >= 0)  {     //   
      if (kind == tkPJFET) { 
        id = 2067 +  base * 16.667;  // 2067 = 12.4 V
      }
      
      else if (kind == tkPNP)     {
        /*
        iu3  = Aver4_ADC(pin_ADC3_Bat_12V);   // A3
        idy3 = (float)iu3;                    // 4095.0  A3  
        idy3 = idy3 * 3.2331;                 // u_bat mV  
        idy2 = idy3 - 680.0 - (float)base * 53.85;   // Voltage mV on Uout LT1013
        idy4 = 0.500 + idy2 / 6.00;           // gain 6
        id = (int)idy4;
        tft.fillRect(90,70,50,80,ILI9341_BLACK);   
        tft.drawNumber((int)idy3,90, 70,2);  // base *100  (vgs in mv) 1 ### test   
        tft.drawNumber((int)idy2,90, 85,2);  
        tft.drawNumber((int)idy4,90, 100,2);  
        tft.drawNumber(base,90,115,2);
        delay(500);  
        */
     // idy1 = base *  1.0;                   // 1 ### 135 = 5 * 27k;  // voltage r mv 10*27k = 270  -> U DAC = 13,23/3.3;  350 * 27 = 9450;     
        idy2 = base * 26.85;           // 2 ### U on 27k 9400 mV on R 2.7V/100µA  11,79V/400µA  26.85 = 27k ??
        iu3  = Aver4_ADC(pin_ADC3_Bat_12V);   // A3
        idy3 = (float)iu3;                    // 4095.0  A3  
        idy3 = idy3 * 3.2331;                 // u_bat mV  
      //  tft.fillRect(90,70,50,80,ILI9341_BLACK);   
      //  tft.drawNumber((int)idy3,90, 70,2);  // base *100  (vgs in mv) 1 ### test  
     // idy3 = (12180.0 - 680.0) - idy2;      // 3 ### 4050  mV on OPA NE5532 = 12180 mV(OP PIN7) - 680mV Ube #########
        idy3 = (idy3 - 630.0) - idy2;         // 3 ### 12.282 = Uout(max) PIN7 OPA LT1013 (=2047 *6)    = 24570 mV(OP PIN7) - 680mV Ube #########
        idy4 = idy3 / 2.923;                  // 4 ### Voltage  Pin8 DAC 1227 mV on MCP4802/MCP4812
     //   idy4 = idy3 / 1.4615;                  // i-DAC : 0...2.047V
     // idy5 = idy4 / 16.00;                  // 5 ### set bit DAC 16 =  256/4096 MCP4802  8 bit
     // idy5 = idy4 / 4.00;                   // 5 ### set bit DAC  4 = 1024/4096 MCP4812 10 bit
        idy5 = idy4;                          // 5 ### set bit DAC  1 = 4096/4096 MCP4822 12 bit
        idy5 = idy5 + 0.500;                  // 6 ### rounded
        id = (int)idy5;
        
      }
      else if (kind == tkPMOS)  {
        idy1 = (int)base * 100.0;               // in mV 
     // idy3 = 12180.0 - idy1;                  // U-12V - n* 100mV
        iu3 = Aver4_ADC(pin_ADC3_Bat_12V);   // A3
        idy3 = (float)iu3;                      // 4095.0  A3
     // idy3 = idy3 * 0.80586;//3300.0/float(amax);// 4095.0;          // mV at A3
     // idy3 = idy3 * 4.012;//133.2/33.2;       // mV at U-12V
        idy3 = idy3 * 3.2331;                   // max 12.9712
      //  idy3 = idy3 * 1.6165;                   // i-DAC : 0...2.047V
        idy3 = idy3 - idy1;                     // U-12V[in mv] - n* 100mV
     // idy5 = idy3  / 12.495;                  // set Dac_base for voltage  // bei R4=68.1k,R5=33.2k  : 12.495mV  at R4=50k,R5=26k: 11.97 mV,  4mV pro Dac_Count
     // idy5 = idy3  / 3.0542;                  // set Dac_base for voltage  // bei R4=68.1k,R5=33.2k  :  3.0542mV at R4=50k,R5=26k: 2.923 mV,  1mV pro Dac_Count
        idy5 = idy3  / 3.003;                   // set Dac_base for voltage  // bei R4=66.4k,R5=33.14k :  3.003mV  at R4=50k,R5=26k: 2.923 mV,  1mV pro Dac_Count
        id = (int)idy5;
        /*
        tft.fillRect(90,70,50,80,ILI9341_BLACK);   
        tft.drawNumber((int)idy1,90, 70,2);  // base *100  (vgs in mv) 1 ### test       
        tft.drawNumber(base,     90, 83,2);      // 1 ### test 
        tft.drawNumber(incBase,  90, 96,2); 
        tft.drawNumber(iu3,      90,109,2); 
        tft.drawFloat(idy3,    1,90,122,2);   // 12,104V
        tft.drawFloat(idy5,    1,90,135,2);   //   4030  
        delay(100); 
        */
      }
      else if (kind == tkPDIODE)  {
        id    = 1;
        s_dio = 1;
      }
     
        
      if ( id < 0 )    break;
      if ( id > dmax ) id = dmax;

      if (kind == tkPJFET) {     // gate-voltage > 12V  ###########################################
        dac12(id,0,4);           // channel A ,0... 4095 >> 0...+24V, gain-I = +6
        delayMicroseconds(400);  // 400 
        
     //   Serial.print("pJFET id ");Serial.println(id);

      }
      else if (kind == tkPDIODE)  {    //    SetDacBase(0,5); //????????????????????? 
        idy5  = 0.0;     // ?? copy from  tkNDIODE
        id    = 0;          // ?? copy from  tkNDIODE
        s_dio = 1;       // ?? copy from  tkNDIODE
     
        dac12(0,0,2);            // channel A ,0... 2047 >> 0...+12V
   //     Serial.print("pDIODE id ");Serial.println(id);
        delayMicroseconds(50);   // 400 
      }  
      
      else                 {     //         SetDacBase(id, 5);  
        dac12(id,0,2);           // channel A ,0... 2047 >> 0...+12V, gain-I = +6
     //   dac12(id,0,4);           // channel A ,0... 4047 >> 0...+12V gain=3 ####################################################
        delayMicroseconds(400);  // 400 
      }  
     
      
    } // if (base >= 0)
 
 
   
    for (DacVcc = 4095; DacVcc >= 0; DacVcc -= incv)   { // pDev
      cntv++;
   
      if (kind == tkPDIODE)  { 
        if      (cntv <  10) delay_d =  500;  // 3 700
        else if (cntv <  100) delay_d =  100;  // 550
   //     else if (cntv < 150) delay_d =  10;  // 6
        else                 delay_d =   2;  // 2 3
   //   Serial.print("count ");Serial.print(cntv);Serial.print(" delay ");Serial.println(delay_d);
      }  
      else {
        if (cntv < 100) delay_d =  20; // 20 10 50
        else           delay_d =  3;  // 2
      }  
  //    SetDacVcc(DacVcc,2);                  // 5 50 5 µs
   //   if (s_secx == 3)  dac12(DacVcc,1,2);   // factor 2 :  0...6V   
      dac12(DacVcc,1,4);                    // set DAC-B voltage 0...4095  // SetDacVcc(DacVcc,0);  // 2047/ 12 bit, 512, 10 bit  ca 6V ### 
      delayMicroseconds(delay_d);           // if (DacVcc == 4095)   delayMicroseconds(250);   else delayMicroseconds(10); 
      if (s_secy != 2)       {     //  normal: 50mA /100mA
        iu0 = analogRead(pin_ADC0_Vo_OPV);  // A0 Output TCA0372
        iu2 = analogRead(pin_ADC2_Vc_PNP);  // A2 Collector PNP
        iu3 = analogRead(pin_ADC3_Bat_12V); // A3 Bat-Voltage
      }
      else {  
        if (kind == tkPJFET)  {
          iu0 = Aver4_ADC(pin_ADC0_Vo_OPV);  // average over 4 reads A0
          iu2 = Aver4_ADC(pin_ADC2_Vc_PNP);  // average over 4 reads A2 Collector PNP
          iu3 = Aver4_ADC(pin_ADC3_Bat_12V); // average over 4 reads A3
        }  
        else {
          iu0 = analogRead(pin_ADC0_Vo_OPV);  // A0 Output TCA0372
          iu2 = analogRead(pin_ADC2_Vc_PNP);  // A2 Collector PNP
          iu3 = analogRead(pin_ADC3_Bat_12V); // A3 Bat-Voltage
        }  
      }
    
      Adc_12V = iu3;
    
      if (zen_v < 24) { // <= 12V: divider : 100k/33.2k
        fu0 = (float)iu0*0.0032331; //*13.234/4095.0;//(float)amax;
        fu2 = (float)iu2*0.0032331; //*13.234/4095.0;//(float)amax;
        fu3 = (float)iu3*0.0032331; //*13.234/(float)amax;
      }
      else {            // 24V: divider: 100k/(33.2||20k) = 100k/12.481k
        fu0 = (float)iu0*0.007263; //*29.742/4095.0;//(float)amax;
        fu2 = (float)iu2*0.007263; //*29.742/4095.0;//(float)amax;
        fu3 = (float)iu3*0.007263; //*29.742/4095.0;//(float)amax;
      }
      
      fud = fu2 - fu0;   // V at R32
    // Serial.print("count ");Serial.print(cntv);Serial.print(" iu0 ");Serial.print(iu0);Serial.print(" iu2 ");Serial.print(iu2);Serial.print(" iu3 ");Serial.print(iu3);
    // Serial.print(" fud ");Serial.println(fud);
       
      if ( fud >= fimax )      {              // fimax = 50mA * 100 Ohm = 5.0 (=R32)  #########
        s_kindcur = 1;
        dac12(4095,1,4);               //############################################## 
        if (kind == tkPDIODE)  goto end_pDev;
        else                   goto end_base_pDev;  // next base ?? 
      }
      if (kind == tkPMOS) {
        if (fud >= fim_pmos) {
           if (f_thrp == 0) f_thrp = fud;
         }
      }
      if (kind == tkPDIODE) {   // between A3 and A2
                       
        iuz = (int)(0.5 + 10.0*(fu3-fu2));     // 100m V zener-voltage ?
      //  if (fud > 0.1) delayMicroseconds(250);  //######################################################### 
        if (fud >= fim)                    {   // fim = maxBase * 100 Ohm (=R37)
          s_zencur = 1;  // ??test
          goto end_pDev;
        }   
      //  if (fud >= 0.2) delayMicroseconds(10);   ???
        if (s_diode == 1)    {
          tft.fillRect(240,91,38,72, ILI9341_BLACK);    // ### di
          tft.setTextColor(ILI9341_AQUA);
          tft.drawNumber(DacVcc,  240, 92,2);  // ### test di -1    // 4095
          tft.drawNumber(iu3,     240,104,2);  // ### u-bat 
          tft.drawNumber(iu2,     240,116,2);  // ### test di 3607 // 3369 Collector pnp/R32
          tft.drawNumber(iu0,     240,128,2);  // ### test di 4095  // 4074  from Ubat (13.2V) --> 0V
          tft.drawNumber(base,    240,140,2);  // ### test di 267  // 2410      UB OPamp/R32
          tft.drawFloat(fim,  1,  240,152,2);  // ### test di 4095  // fu2 - fu0 3369-2410 =   ---> iz
          delay(2);
        }  
      }  
  //    Serial.print("DacV ");Serial.print(DacVcc);Serial.print(" u3 ");Serial.print(iu3);Serial.print(" u2 ");Serial.print(iu2);Serial.print(" u0 ");Serial.print(iu0);Serial.print(" ib ");Serial.println(base);
  /*    if (kind == tkPJFET)   { 
    
        tft.fillRect(240,91,38,72, ILI9341_BLACK);    // ### di
        tft.setTextColor(ILI9341_AQUA);
        tft.drawNumber(DacVcc,  240, 92,2);  // ### test di -1    // 4095
        tft.drawNumber(iu3,     240,104,2);  // ### u-bat 
        tft.drawNumber(iu2,     240,116,2);  // ### test di 3607 // 3369 Collector pnp/R32
        tft.drawNumber(iu0,     240,128,2);  // ### test di 4095  // 4074  from Ubat (13.2V) --> 0V
        tft.drawNumber(base,    240,140,2);  // ### test di 267  // 2410      UB OPamp/R32
        tft.drawFloat(fim,  1,  240,152,2);  // ### test di 4095  // fu2 - fu0 3369-2410 =   ---> iz
        delay(2);
      }  
      */
      /*  
      tft.fillRect(90,150,50,60,ILI9341_BLACK); 
      tft.drawNumber(iu3,90,150,2);  // bat_v  /AC_12V    // 13.1 V   _> iu = 4052;
      tft.drawNumber(iu2,90,163,2);  // coll./drain // 12.5 V
      tft.drawNumber(iu0,90,176,2);  // TCA0372 out  // 7.6 V
      tft.drawNumber(base,90,189,2);  // TCA0372 out  // 7.6 V
      delay(20);
      */
      if (kind == tkPJFET)
        Graph_P( false, iu0, iu2, base, Adc_12V);   // for Scan_p_Dev : pnp,p-mos,p-jfet,pDiode 1.parameter is_npn=false <==
      else
        GraphZ( false, iu0, iu2, base, Adc_12V);   // for Scan_p_Dev : pnp,p-mos,p-jfet,pDiode 1.parameter is_npn=false <==
      if (prev_y < 0)    DacVcc = -1;
      if (s_endgr == 1)  goto end_pDev;   
   
      if (kind == tkPDIODE) {   // between A3 and A2
                       
    //    iuz = (int)(0.5 + 10.0*(fu3-fu2));  // 100m V
    //    if (fud >= 0.2) delayMicroseconds(10);
        if (s_diode == 1)    { 
       // tft.drawNumber(iuz ,      282,80,2);   // ### test di 4095  // iuz ---> uz
          tft.fillRect(280,91,38,80, ILI9341_BLACK);    // ### di
          tft.setTextColor(ILI9341_YELLOW);
          tft.drawNumber(DacVcc,    282, 92,2);  // ### test di -1    // 4095
          tft.drawFloat(fu3,     1, 282,104,2);  // ### test di 4095  // 4074            -----> Ubat = 13.2V
          tft.drawFloat(fu2,     1, 282,116,2);  // ### test di 3607 // 3369 Collector pnp/R32
          tft.drawFloat(fu0,     1, 282,128,2);  // ### test di 267  // 2410      UB OPamp/R32
          tft.drawFloat(fud,     2, 282,140,2);  // ### test di 4095  // fu2 - fu0 V-R32 =   ---> iz
          tft.drawFloat(fu3-fu2, 1, 282,152,2);  // ### test di 4095  // Uec pnp2410     =   ---> uz
          delay(1);
        } // if (s_diode == 1
      }   // diode
    }     // for DacVcc  // for DacVcc
    
    if ((base >= 0)  &&      // > 0
        (base <= maxBase))    {
  
      tft.setTextColor(ILI9341_BASE);
      switch (kind) {
        case tkPNP:
          tft.drawNumber(base,prev_x + 4,prev_y - 6,2);//  DrawInt(base , SmallFont, ILI9341_WHITE);
         break;
        case tkPMOS:
          if (prev_x > 270) prev_x = 264;
          tft.fillRect(prev_x,prev_y-6,30,13,ILI9341_BLACK);
          drawDec1(base,prev_x + 4,prev_y - 6,2,ILI9341_BASE);   // 1 Nachkomma-Stelle
          tft.drawString(" V",prev_x + 36,prev_y - 6,2);  //DrawString("V", SmallFont, ILI9341_WHITE);
         break;
        case tkPJFET:
          if (prev_x > 270) prev_x = 264;
          tft.fillRect(prev_x,prev_y-6,30,13,ILI9341_BLACK);
          drawDec1(base ,prev_x + 4,prev_y - 6,2,ILI9341_BASE);   // 1 Nachkomma-Stelle
          tft.drawString(" V",prev_x + 36,prev_y - 6,2);  //DrawString("V", SmallFont, ILI9341_WHITE);
         break;
      }  // switch
    }    // base > 0

    if (HaveTouch())return;
 
 end_base_pDev: 
    delay(100); 
  }  // for (base 
 end_pDev:

  if (s_kindcur == 1) {   // current-limit diodes and other kind 
    if (s_blink == 0) {  
      curLimit();
      s_blink = 1;
      delay(500);
    }
    if ((kind == tkPJFET) ||  // small Rds  ########################### 
        (kind == tkPMOS))    { 
      if (s_secy != 3) s_secy = -1;   // second run with 100mA max. scale
      else s_secy = 4;  // ??test ?4?
    }
  }

  if (s_zencur == 1) {    // zener current-limit
    if (iuz > 0)  {
      tft.setTextColor(ILI9341_CYAN);
      if (prev_x > 260) prev_x = 260;
      if (iuz >= 33)  tft.drawString("Uz=",prev_x+5,prev_y,2);   // DrawString(" Vd ", LargeFont, ILI9341_CYAN);
      else            tft.drawString("Ud=",prev_x+5,prev_y,2);   // DrawString(" Vd ", LargeFont, ILI9341_CYAN);
      drawDec1(iuz,prev_x+28,prev_y,2,ILI9341_CYAN);            // DrawDecimal(iuz, LargeFont, ILI9341_CYAN); 
      s_xfactor =  1; 
      zen_a = zen_v;
      if (s_secx == 1) {    // uz = 0: search optimal x - scale
        if      (iuz <  10) s_xfactor = 12;  //  1V=10*100mV
        else if (iuz <  20) s_xfactor =  6;  //  2V
        else if (iuz <  30) s_xfactor =  4;  //  3V
        else if (iuz <  40) s_xfactor =  3;  //  4V
        else if (iuz <  60) s_xfactor =  2;  //  6V
        else if (iuz < 120) s_xfactor =  1;  // 12V 
        
        s_secx = 2;                          // second run
        zen_v = 12/s_xfactor; // 1 + (iuz + 1) / 10;
        if (zen_v == zen_a) s_secx = 3;      // end
      
        if (iuz  >= 120)    s_secx = 3;      // end
      }  
      else s_secx = 3; 
    }  // iuz > 0
    else  tft.drawString("Uz?? ",180,100,2);
  }
  EndScan(kind);
  if (s_secx == 2)  {
    InitGraph();
    DrawKindStr(kind);
    s_secx = 3;     // 2. scan  ??
    goto second_scan_pDev; 
  }
  if ((s_secy == 1)  &&       // 2. scan because IDss < 10mA
      (kind ==  tkPJFET))  {  // IDss < 10mA
    s_yfactor = 5;
    s_secy = 2;         // 2. scan with idss < 10mA 
    InitGraph();
    DrawKindStr(kind);
  
    delay(1500);
    goto second_scan_pDev; 
  } 
  if ((s_secy == -1) &&       // IDss > 50mA -1=nFET, -2=pFET ??
    ((kind ==  tkPJFET) || (kind == tkPMOS))) {  // IDss > 50mA  ??test
    s_yfactor = 9;
    s_secy = 3;         // 2. scan with idss 100mA 
    InitGraph();
    DrawKindStr(kind);
    delay(1500);
    fimax = fimax + fimax; /// ??test ?????????????????????????????????
    goto second_scan_pDev; 
  }  
  while (1 == 1) {   if (HaveTouch()) return; }
 
}



//-------------------------------------------------------------------------
// Scan_n_Dev
//   draw curves for a component on the NPN side of the DUT socket: np-Diode,npn,n-MOS (also depletion),n-JFET
//    
//   if user touches the screen at the end of a line, the scan is terminated and the function returns true
//-------------------------------------------------------------------------

void Scan_n_Dev(TkindDUT kind, int minBase, int maxBase, int incBase) {

  int DacVcc; // collector
  int zw_maxb,zw_minb,zw_incb,base,delay_d=10,zen_a=0;
  int id,iu3,iu4,iud;    // max 4095  ### d
  int idr,incv = 1;      // increment DACVcc 
  uint8_t s_blink = 0,s_kindcur=0,s_zencur=0;   // warning current blinking
  float idy1,idy2,idy3,idy4,idy5,fim=5.0001,fimax,fu0,fu1,fud,fum,fim_nmos=0.5;

  //tft.fillRect(0,0,10,12,ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE);tft.drawNumber(2,0,0,2); 
  if (s_func == 1)  drawFunc(1,1,60,"Scan_n_Dev",ILI9341_WHITE);  //####
  digitalWrite(BTN0, LOW);         // 
  digitalWrite(BTN12, LOW);        // switch on positive base voltage (non-inverting amplifier) gain = +6  standard
  digitalWrite(BTN32, HIGH);       // 12V standard
  delay(1);
  i_o=0;
  i_j=0;
  i_n=0;
  j_o=0;
  zw_maxb = maxBase;   //##########################
  zw_minb = minBase;   //##########################
  zw_incb = incBase;   //##########################
  s_depl = 0;             // enhancement ??
  fimax  = 5.0001;        // 50 mA on 100 Ohm = 5.0   //tft.drawFloat(fimax,1,70,0,2);
  f_thrn = 0;
  if (kind == tkNDIODE)   {       // current limit diode 

    fim = 0.1 * (float)maxBase;   //    tft.drawFloat(fim,1,120,0,2);
    if ( maxBase <= 10 )  {  
      s_yfactor = 5;
      s_secy    = 2;
    }  
  
  } 
  
  if (kind == tkNMOS)    {   // maxBase = 120 
 //   maxBase = 60;
  }
  if (kind == tkNJFET)    {   // maxBase = 120
  
    zw_maxb = minBase; // change min - max ###################
    zw_minb = maxBase; // change max - min ###################
    zw_incb = incBase;
    s_vgate0 = 0;
    s_jfet   = 1;
    s_pinch = 0;
    s_tend  = 0;
 
    digitalWrite(BTN12, HIGH);  // switch on negative base voltage (inverting amplifier) gain = -6 ########################################
    delay(10);
  }

  else {
    digitalWrite(BTN12, LOW);  // switch on positiv base voltage (noninverting amplifier) gain = +6 ########################################
    delay(10);
  }

//  s_secx = 0;
   
 second_scan_nDev: 
  s_kindcur = 0;
  s_zencur = 0; 
  s_blink  = 0;
  maxBase = zw_maxb;  // ??########
  minBase = zw_minb;  // ??#######
  prev_x = 0;
  prev_y = 239;
  ivxa   = 0;
  
  DacVcc = 0;
  maxYposGain = -1;
  minBaseGain = 0;
  maxBaseGain = 0;
 

  if (kind == tkNDIODE)   {
    incBase = 1;
    minBase = 1;
    iuz     = 0;
    if      (zen_v < 24) {
      digitalWrite(BTN0,  LOW);  // HIGH: 24V 
      digitalWrite(BTN32, HIGH); // 12V standard
    }  
    else                 {  // zen_v == 24) 
      digitalWrite(BTN0,  HIGH); // 24V for zener > 12V 
      digitalWrite(BTN32, LOW);  // HIGH: 12V
    }  
    delay(10);  // needs time to stabilize
  
  }  
  else if (kind == tkNJFET)   {   // maxBase = 120
    DacVcc = 0;
 
 
    incBase = zw_incb;
    minBase = zw_minb; 
    maxBase = zw_maxb; 
  } 
  else if (kind == tkNMOS)   {   // maxBase = 120
  //  maxBase = 120;
  } 
  else {   
    if (incBase <=0) incBase = 1;       // ### ?? 
    id = (maxBase - minBase)/ incBase;  // ### di maxBase = 50  minBase = Ianode-inc incBase = 5
    if (id > 20) maxBase = incBase * 20;
  }  
  delay_d = 50;

  //Serial.print("min: ");Serial.print(minBase);Serial.print(" max: ");Serial.print(maxBase);Serial.print(" inc: ");Serial.println(incBase);
  
  for (base = minBase; base <= maxBase; base = base + incBase) {
 //   tft.fillRect(80,0,90,13,ILI9341_BLACK);   // v-gate start
 //   tft.drawNumber(base,80,0,2);   // v-gate start
 //   tft.drawNumber(maxBase,120,0,2);  // vgate (input) 120
 //   tft.drawNumber(incBase,160,0,2);  // vgate (input) 120  
  //  delay(500);
    cntv = 0;
      
    ILI9341_TraceCol = ILI9341_WHITE;
    if ( base >= 0)       {  // 
  
      if (kind == tkNPN) {
     
       
     // idy1 = base *  1.0;                   // 1 ### 135 = 5 * 27;  // voltage r mv 10*27k = 270  -> U DAC = 13,23/3.3;  350 * 27 = 9450; 
        idy2 = base * 26.85;                  // 2 ### 9400 mV on R 26.85 = 27k ??
        idy3 = idy2 + 680;                    // 3 ### 4050  mV on OPA NE5532 12815 = 13455 mV(OP PIN7) - 680mV Ube #########
        idy4 = idy3 / 3.3;                    // 4 ### Voltage  Pin8 DAC 1227 mV on MCP4802/MCP4812
      //  idy4 = idy3 / 1.65;                    // 4 ### Voltage  Pin8 DAC 1227 mV on MCP4802/MCP4812/22 V-max =2.047 V
       // idy5 = idy4 / 16.00;                // 5 ### set bit DAC 16 = 256/4096 MCP4802  8 bit
       // idy5 = idy4 / 4.00;                 // 5 ### set bit DAC 4 = 1024/4096 MCP4812 10 bit
        idy5 = idy4; // /1.0;                 // 5 ### set bit DAC 1 = 4096/4096 MCP4822 12 bit
        idy5 = idy5 + 0.500;                  // 6 ### rounded
        id = (int)idy5;
      }
      else if (kind == tkNMOS)   {
         
        idy1 = base * 100.0;                 // in mV 
        idy3 = idy1;
     // idy5 = idy3  / 12.495;               // set Dac_base for voltage  // bei R4=68.1k,R5=33.2k : 12.495mV bei R4=50k,R5=26k: 11.97  mV, 4mV pro Dac_Count
       idy5 = idy3  / 3.0542;               // set Dac_base for voltage  // bei R4=68.1k,R5=33.2k : 3.0542mV bei R4=50k,R5=26k: 2.923 mV, 1mV pro Dac_Count
     //   idy5 = idy3  / 1.5271;               // i-DAC : 0....2.047V            
     // idy5 = idy3 * 0.08553;
        id = (int)idy5;
  //      Serial.print("id: ");Serial.print(id);Serial.print(" dmax: ");Serial.println(dmax);
      }
      else if (kind == tkNJFET)   {
        id = base * 16.6667;      // gain = -6  #######################################
      }
      else if (kind == tkNDIODE)  {
        idy5  = 0.0;  // ??
        id    = 0;
        s_dio = 1;
      }
      if (id > dmax) break;
      if ( kind == tkNJFET ) {  // ub=-12V 
        dac12(id,0,2);         // 0...-12V gain=-6
        delayMicroseconds(20);
      }   
      else {
        dac12(id,0,2);         // 0...-12V gain +6
    //    SetDacBase(id, 2);     // 0..+24V
        delayMicroseconds(20);
        if (kind == tkNDIODE) base = 1;  //maxBase;  
      }
     
    }   // if (base >=0
    
    for (DacVcc = 0; DacVcc <= dmax; DacVcc +=  incv) {   // +=8 // +=2 ###      // m+=8 Scan_n_Dev###
      cntv++;
   
      if (kind == tkNDIODE) {
        if      (cntv <  10) delay_d = 400; // 200 700  // attention !!
        else if (cntv <  50) delay_d =   4; // 2 10 100
        else                 delay_d =   4; // 1  5
   
      }  
      else if (kind == tkNJFET) {
        if (cntv < 6) delay_d =  500;  // 10 50
        else          delay_d =  2;
      }
      else {
        if (cntv < 10) delay_d =  10;  // 5 10 50
        else           delay_d =  3;  // 2
      }  
       // if (s_secx == 3) dac12(DacVcc,1,2);   // set DAC-B voltage : 0...2.047V , gain TCA0372=3 : 0... 6V
       /* else */dac12(DacVcc,1,4);                // set DAC-B voltage : 0...4.095v , gain TCA0372=3 : 0...12V
       if (delay_d > 0) delayMicroseconds(delay_d);
       if (s_secy != 2) {      // normal: 50mA /100mA
         iu0 = analogRead(pin_ADC0_Vo_OPV);  // A0
         iu1 = analogRead(pin_ADC1_Vc_NPN);  // A1
       }
       else {                  // 2
         if (kind == tkNJFET) {
           iu0 = Aver4_ADC(pin_ADC0_Vo_OPV);  // A0
           iu1 = Aver4_ADC(pin_ADC1_Vc_NPN);  // A1 
         }
         else {
           iu0 = analogRead(pin_ADC0_Vo_OPV);  // A0
           iu1 = analogRead(pin_ADC1_Vc_NPN);  // A1
         }
       }
       if (zen_v < 24) { // <= 12V: divider : 100k/33.2k
         fu0 = (float)iu0*0.0032331; //*13.234/4095.0;//(float)amax;
         fu1 = (float)iu1*0.0032331; //*13.234/4095.0;//(float)amax;
       }
       else {            // 24V: divider: 100k/(33.2||20k) = 100k/12.481k
         fu0 = (float)iu0*0.007263; //*29.742/4095.0;//(float)amax;
         fu1 = (float)iu1*0.007263; //*29.742/4095.0;//(float)amax;
            
       }
      
       fud = fu0 - fu1; 
   // Serial.print("count ");Serial.print(cntv);Serial.print(" iu0 ");Serial.print(iu0);Serial.print(" iu1 ");Serial.print(iu1);Serial.print(" fud ");Serial.println(fud);
        if ( fud >= fimax )      {              // fimax = 50mA * 100 Ohm = 5.0 (=R32)  #########
          s_kindcur = 1;
          if (kind == tkNDIODE)  goto end_nDev;
          else                   goto end_base_nDev;  // next base ?? 
        }
        if (kind == tkNDIODE) {
          iuz = (int)(0.5 + 10.0 *fu1);  // 100m V// uz between A1 and ground  ######## 
          if ( fud >= fim )     {              // fim = maxBase * 100 Ohm (=R32)
            s_zencur = 1;
            goto end_nDev;
          }
        }  // if (kind = tkNDIODE
        if (kind == tkNMOS) {
          if (fud >= fim_nmos) {
            if (f_thrn == 0) f_thrn = fud;
          }
          
        }
        if ((kind == tkNPN) && (s_npn == 1))   {
          tft.setTextColor(ILI9341_YELLOW);
          tft.fillRect(242,80,50,75, ILI9341_BLACK);    // ### di
          tft.drawNumber(DacVcc,242,80,2);      // ### test npn            0
          tft.drawNumber(iu0,242,92,2);         // ### test npn A0/Vout OP 2565 ??
          tft.drawNumber(iu1,242,104,2);        // ### test npn A1/Vc npn  41  
          tft.drawNumber(base, 242,116,2);      // ### test npn            50
        }
        if (kind == tkNJFET)
           Graph_P( true, iu0, iu1, base, 4095);  // nDev ######################################################################
        else  
           GraphZ( true, iu0, iu1, base, 4095);  // other nDev ######################################################################
        if (prev_y > 239)  prev_y = 239;  // ??
        if (prev_y < 0)    DacVcc = dmax + 1;  // 4096; 1024; //256;  // Abbruch ??
       
        if (s_endgr == 1)  goto end_nDev; 
           
        if ((kind == tkNDIODE) && (s_diode == 1))    {
          tft.fillRect(282,80,38,75, ILI9341_BLACK);    // ### di
          tft.setTextColor(ILI9341_YELLOW);
          tft.drawNumber(DacVcc, 282,80,2);  // ### test di 568
          tft.drawNumber(iu0,282,92,2);      // ### test di
          tft.drawNumber(iu1,282,104,2);      // ### test di
          tft.drawNumber(iuz,282,116,2);  //########## im
          tft.drawFloat(fud,2,282,128,2);     // ### test di 4095
          delay(50);
        }
               
      }  
  
     // if ((base >= 0) && (base <= maxBase)) {
      if (base >= 0) {  
     // tft.setCursor(prev_x + 1, prev_y + 2); //ILI9341SetCursor(prev_x + 1, prev_y + 2);
        tft.setTextColor(ILI9341_BASE);
        switch (kind) {
          case tkNPN:
            tft.drawNumber(base,prev_x + 4,prev_y - 6,2);   //DrawInt(base, SmallFont, ILI9341_WHITE);
           break;
          case tkNMOS:
            if (prev_x > 270) prev_x = 270;
            tft.fillRect(prev_x,prev_y-6,32,13,ILI9341_BLACK);
            if (s_depl == 1) {
              tft.setTextColor(ILI9341_YELLOW);                    // complete n-MOS with: depl.
              tft.drawString("depl.",280,33,2);  
              drawDec1(-base,prev_x+4,prev_y -6,2,ILI9341_BASE);     // 1 nachkomma-stelle negativ
              tft.setTextColor(ILI9341_WHITE);
            }  
     
            else drawDec1(base,prev_x+4,prev_y -6,2,ILI9341_BASE);   // 1 nachkomma-stelle
            tft.drawString("V",prev_x + 35,prev_y - 6,2);  //DrawString("V", SmallFont, ILI9341_WHITE);
           break;
          case tkNJFET:
            if (prev_x > 270) prev_x = 270;
            tft.fillRect(prev_x,prev_y-6,46,26,ILI9341_BLACK);
         // drawDec1(-(base-incBase)/10 ,prev_x + 4,prev_y - 6,2,ILI9341_BASE);   // 1 Nachkomma-Stelle
         // drawDec1(-(base - incBase)/10 ,prev_x + 6,prev_y - 6,2,ILI9341_BASE);   // 1 Nachkomma-Stelle
            drawDec1(-base,prev_x+4,prev_y -6,2,ILI9341_BASE);   // 1 nachkomma-stelle
            tft.drawString("V",prev_x + 37,prev_y - 6,2);  //DrawString("V", SmallFont, ILI9341_WHITE);
            delay(200);
           break;   
      
      
         }
      }
 
      if (HaveTouch())  return;
 end_base_nDev: 
      delay(100);
   } // for (base ...
 end_nDev: 

   if (s_kindcur == 1) {   // "normal" current-limit at 50mA 
     if (s_blink == 0) {  
       curLimit();
       s_blink = 1;
       delay(500);
     }
     if (kind == tkNJFET) {   // test only for depletion mode with small Rds  ########################### 
       kind = tkNMOS;  // depletion-mode ??
       s_depl = 1;     // for sign in end_scan...  
       if (s_secy != 3) s_secy = -1;   // second run with 100mA max. scale
       else s_secy = 4;    // ??test ?4
     }
     if (kind == tkNMOS) {   // test only  for small Rds  ########################### 
       if (s_secy != 3) s_secy = -1;   // second run with 100mA max. scale
       else s_secy = 4;    // ??test ?4
     }
   }
   if (s_zencur == 1) {    // zener current-limit
     if (iuz > 0)  {
       dac12(0,1,4);                     // set DAC-B voltage to 0
       delayMicroseconds(100);
       tft.setTextColor(ILI9341_CYAN);
       if (prev_x > 260) prev_x = 260;
       if (iuz >= 33)  tft.drawString("Uz=",prev_x+5,prev_y,2);   // DrawString(" Vd ", LargeFont, ILI9341_CYAN);
       else            tft.drawString("Ud=",prev_x+5,prev_y,2);   // DrawString(" Vd ", LargeFont, ILI9341_CYAN);
       drawDec1(iuz,prev_x+28,prev_y,2,ILI9341_CYAN);            // DrawDecimal(iuz, LargeFont, ILI9341_CYAN); 
       s_xfactor =  1; 
       zen_a = zen_v;
       if (s_secx == 1) {    // uz = 0: search optimal x - scale
         if      (iuz <  10) s_xfactor = 12;  //  1V=10*100mV
         else if (iuz <  20) s_xfactor =  6;  //  2V
         else if (iuz <  30) s_xfactor =  4;  //  3V
         else if (iuz <  40) s_xfactor =  3;  //  4V
         else if (iuz <  60) s_xfactor =  2;  //  6V
         else if (iuz < 120) s_xfactor =  1;  // 12V 
        
         s_secx = 2;                          // second run
         zen_v = 12/s_xfactor; //1 + (iuz + 1) / 10;
         if (zen_v == zen_a) s_secx = 3;      // end
      
         if (iuz  >= 120)    s_secx = 3;      // end
       }  
       else s_secx = 3; 
     }  
   }
  EndScan(kind);
  
  if (s_secx == 2)  {
    InitGraph();
    DrawKindStr(kind);
    s_secx = 3;  // 2. scan  ??
    delay(1500);
    goto second_scan_nDev; 
  }
  
  if ((s_secy == 1)  &&       // 2. scan because IDss < 10mA
      (kind ==  tkNJFET))  {  // IDss < 10mA
    s_yfactor = 5;
    s_secy = 2;         // 2. scan with idss < 10mA 
    InitGraph();
    DrawKindStr(kind);
    delay(1500);
    goto second_scan_nDev; 
  } 
  if (s_secy == -1)  {     // IDss > 50mA -1=nFET, -2=pFET ??
    if ((kind ==  tkNDIODE) || (kind == tkNPN)) return;  // IDss > 50mA
    s_yfactor = 9;
    s_secy = 3;         // 2. scan with idss 100mA 
    InitGraph();
    DrawKindStr(kind);
    delay(1500);
    fimax = fimax + fimax; /// ??test ?????????????????????????????????
    goto second_scan_nDev; 
  }  
 
  while (1 == 1) {   if (HaveTouch()) return; }
 
  
}

 





//-------------------------------------------------------------------------
// EndScan
//   at end of drawing curves - calculates gain
//-------------------------------------------------------------------------

void EndScan(TkindDUT kind) {
  int i = 0;
//  tft.fillRect(0,0,40,12,ILI9341_BLACK); tft.setTextColor(ILI9341_WHITE);tft.drawNumber(4,40,0,2);  // d??
  if (s_func == 1) drawFunc(80,1,60,"EndScan",ILI9341_WHITE);  //####
  s_save  = 0;
  s_first = 1;
  TurnOffLoad(kind);
//  TclassDUT Curclass_x = tcDIODE;//tcMOSFET; //tcBIPOL;
  
  tft.fillRect(250,40,80,14,ILI9341_BLACK);  
  tft.setTextColor(ILI9341_KindCol);  // Orange
  
    switch (kind) {
    case tkPNP:
    case tkNPN:
     // Curclass_x = tcBIPOL;
      if (maxBaseGain > minBaseGain) {
       // DrawKindStr(kind);  //######################################################################
        tft.drawString("gain=", 250,40,2);         
        i = long(minYposGain - maxYposGain) * mAmax * 1000 / (maxBaseGain - minBaseGain) / TFT_HEIGHT ;// / 5;
        tft.drawNumber(i,280,40,2);                
        break; // return;  // ##co
      } 
    
      tft.drawString(" gain = ???", 250,70,2);  
     break; // return;  // ##co
    case tkNMOS:
   
      tft.drawString("Vth=", 250,40,2);  
      tft.drawFloat(f_thrn,1,280,40,2);    
   //   i = GetNMosfetThreshold();
   //   drawDec1(i,280,40,2,ILI9341_KindCol);  
     break; // return;  // ##co
      

    case tkPMOS:
  
      tft.drawString("Vth=", 250,40,2);  
      tft.drawFloat(f_thrp,1,280,40,2);   
  //    i = GetPMosfetThreshold();
  //    drawDec1(i,285,40,2,ILI9341_KindCol);    
     break; // return;  // ##co

    case tkPJFET:
    case tkNJFET:

      if (kind == tkPJFET) tft.drawString("Voff=",  250,40,2);  
      if (kind == tkNJFET) tft.drawString("Voff=-", 250,40,2); 
      i = pinch_v; // GetJfetPinchOff(kind);
      drawDec1(i,292,40,2,ILI9341_KindCol);   
     break; // return;  // ##co
      
    case tkNDIODE:
    case tkPDIODE:
 
      tft.drawString(" Iz =", 240,40,2);   
      i = MaxIanode;
      tft.drawNumber(i,275,40,2);   // I-Diode in mA
      if (i >= 10) tft.drawString("mA", 300,40,2);  
      else         tft.drawString("mA", 288,40,2);  
     break; // return;  // ##co 
  }  
 
 //  while (1 == 1) {   if (HaveTouch()) return; }
  delay(800);     // ### for looking

}


//-------------------------------------------------------------------------
// GetPMosfetThreshold
//   calc threshold V of MOSFET ; result in 100s of mV
//-------------------------------------------------------------------------

int GetPMosfetThreshold() {
  int ii,iv;
  float gate,v12,vd;
  if (s_func == 1) drawFunc(1,1,60,"P_Threshold",ILI9341_WHITE);  //####
  SetDacVcc(dhalf,1);       // 512, 10 bit  ca 6V ### d
  iv = Aver4_ADC(pin_ADC3_Bat_12V);  // battery voltage
  v12 = (float)iv;
  v12 = iv * 3.3/(float)amax;  // U an A3-Pin
  v12 = v12 * 133.2/33.2;      // U-12V 
  v12 = v12 * 10.0;            // 0.1V  
  tft.fillRect(1,1,55,12,ILI9341_BLACK);  //####
  tft.drawFloat(v12,1,1,1,2);  //####
  
  SetDacBase(dmax, 20);  //12V
  gate = (float)(dmax) * 0.03124; // * 0.12495;  // 1mV /DAC_Count// 12 mV, 4mV pro Dac_Count * (68.1+33.2)/33.2  = 12,495mV Dac-Count dmax??
 // tft.drawFloat(gate,1,50,1,2);  //####
 // tft.drawFloat(gate-v12,1,100,1,2);  //####
 
  for (ii = dmax; ii >= dhalf+dmax/10/*2500,600*/; ii--) {   //  3.06*4.095 = 12.495 mV , -12.242mV
   //  tft.drawFloat(gate,1,50,1,2);  //####
  //   tft.drawFloat(gate-v12,1,100,1,2);  //####
     SetDacBase(ii, 20);  // x?   // ### d ca 12mv
 //    gate = (float)(ii) * 0.12495;
     gate = (float)(ii) * 0.03124;   // set Dac_base for voltage  bei R4=68.1k,R5=33.2k : 12.495mV bei R4=50k,R5=26k: 11.97 mV, 1mV pro Dac_Count
  //   tft.fillRect(1,69,55,12,ILI9341_BLACK);  //####
  //   tft.drawFloat(gate,1,60,1,2);  //####
     if (Aver4_ADC(pin_ADC2_Vc_PNP) - Aver4_ADC(pin_ADC0_Vo_OPV) > 10) { // A2 - A0
       vd = 0.5 + (gate - v12); 
       tft.drawFloat(vd,1,100,1,2);  //####
     //   delay(4000);
       //    gate = 0.5 +((float)(ii) * 0.12495 - v12); // set Dac_base for voltage  bei R4=68.1k,R5=33.2k : 12.495mV bei R4=50k,R5=26k: 11.97 mV, 4mV pro Dac_Count
       return (int)(vd);
    }  
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetNMosfetThreshold
//   calc threshold V of MOSFET ; result in 100s of mV
//-------------------------------------------------------------------------

int GetNMosfetThreshold() {
  int ii,i0,i1;
  float gate;
  if (s_func == 1) drawFunc(1,1,60,"N_Threshold",ILI9341_WHITE);  //####
//  SetDacVcc(128, 1);  //  8 bit
  SetDacVcc(dhalf,1);     // 10 bit  ca 6 V ### d
  for (ii = 0; ii <= 1000; ii++) {  // 1mV  // dmax??
    SetDacBase(ii, 20);
    i0 = analogRead(pin_ADC0_Vo_OPV);
    i1 = analogRead(pin_ADC1_Vc_NPN); 
 //   Serial.print("ii: ");Serial.print(ii);Serial.print(" i0: ");Serial.print(i0);Serial.print(" i1: ");Serial.println(i1);
    if ((i0 - i1) > 10)   { // 10 A0 - A1 voltage on R37
//     tft.fillRect(1,1,80,12,ILI9341_BLACK);  //####
 //     tft.drawNumber(ii,1,1,2);              //####
  //    gate = 0.5 + 0.12495*float(ii);     // set Dac_base for voltage  bei R4=68.1k,R5=33.2k : 12.495mV bei R4=50k,R5=26k: 11.97 mV, 4mV pro Dac_Count
       gate = 0.5 + 0.03124*float(ii);      // dmax?? set Dac_base for voltage  bei R4=68.1k,R5=33.2k : 12.495mV bei R4=50k,R5=26k: 11.97 mV, 1mV pro Dac_Count
  //    tft.drawFloat(gate,1,40,1,2);  //####
  //    delay(5000);
   //   Serial.print(" vg: ");Serial.println(gate);
      return (int)gate;
    }
  }
  return 0;
}


/*
//-------------------------------------------------------------------------
// GetJfetPinchOff
//   calc pinch-off V of JFET  ; result in 100s of mV
//-------------------------------------------------------------------------

int GetJfetPinchOff(TkindDUT kind) {
  int gate;
  if (s_func == 1) drawFunc(1,1,60,"JFET_Pinch",ILI9341_WHITE);  //####
  for (gate = 0; gate <= 100; gate++) {   // 100  dmax??
    if (SetJFETvolts(kind, dhalf, gate)) {  // 2047  12 bit, 512 10 bit, 128 8 bit 
 //     ILI9341SetCursor(282,37);  DrawInt(gate, SmallFont, ILI9341_WHITE);  
      if (kind == tkNJFET) {
        if (Aver4_ADC(pin_ADC2_Vc_PNP) - Aver4_ADC(pin_ADC0_Vo_OPV) <= 10)  //  <= 0 // ####  // A2 - A0 R32
          return gate;
      } else {  // tkPJFET??
        if (Aver4_ADC(pin_ADC0_Vo_OPV) - Aver4_ADC(pin_ADC1_Vc_NPN) > 10)   // 10 mV on 100 Ohm   // A0 - A1 R37
          return gate;
      }
    }
  }
  return 0;
}


bool SetJFETvolt(TkindDUT kind, int Vcc, int dDelay) {

 SetDacVcc(Vcc, dDelay);          // DAC-B
 if ((Vcc == 0) || (Vcc == dmax))  delayMicroseconds(80);

 return true;
}



//-------------------------------------------------------------------------
// SetJFETvolts
//   Set Vcc and Vgs for an n-JFET (Vcc measured in DAC counts; Vgs measured in 100s of mV)
//-------------------------------------------------------------------------

bool SetJFETvolts(TkindDUT kind, int Vcc, int Vgs) {
  
  int ADCs, i;
  int ADCu;
  int p1 = -1, p2 = -1, p3 = -1;
  if (s_func == 1) drawFunc(1,1,60,"SetJFetV ",ILI9341_WHITE);  //####
 
  SetDacVcc(Vcc, 1);
  
   
  do {
     SetDacBase(ngJFET, 0);     // start 
    ADCs = Aver4_ADC(pin_ADC2_Vc_PNP);   // ADC2 = A2
  //  const long c1 = long(1000) * AdcVref * 10 * (R2 + R1) / R1 / ADC_MAX; //62 // ### 1000*5*10*80/33/1024 = 118.37

   //                                  3.3(133.2/33.2)/4.095 = 32.33
    const long c1 = 137;    // amax?? long(1000) * AdcVInt * 10 * (R2 + R1) / R1 / ADC_MAX; //126  // ### 1000*1.1*10*280/22/1024 = 136.72 ADC
  //   const long c2 = 466; //long(1000) * DacVref * (R4 + R5) / R5 / 256; // ### 1000 * 40.96 * (49.9 + 26.1)/26.1/256 = 465.90 DAC  // L272 8 bit DAC MCP4802
      const long c2 = 116;  // dmax?? long(1000) * DacVref * (R4 + R5) / R5 / 1024; // ### 1000 * 40.96 * (49.9 + 26.1)/26.1/1024 = 116.475 DAC  // L272 10 bit DAC MCP4812
   // const long c2  = 490; //long(1000) * DacVref * (R4 + R5) / R5 / 256; // ### 1000 * 40.96 * (68 + 33)/33/256 = 489.7 DAC  // NE5532
   //  const long c2  = 544; //long(1000) * DacVref * (R4 + R5) / R5 / 256; // ### 1000 * 40.96 * (68 + 28.33)/28.33/256 = 544.05 DAC  // NE5532 200k||33k=28.33k
    const long c3n =  0; //2;// #####28; // fudge factor  ##################################
    const long c3p =  0; // fudge factor

    if (kind == tkNJFET)
      ngJFET = (ADCs * c1 - (long(Vgs) * 1000 + Vcc * c3n)) / c2;  // amax-dmax??
    else
      ngJFET = ((long(Vgs) * 1000 + Vcc * c3p) - ADCs * c1) / c2;  // amax-dmax??

    if (ngJFET < 0) ngJFET = 0;
    if (ngJFET > dmax) ngJFET = dmax;

    p1 = p2;
    p2 = p3;
    p3 = ngJFET;
   
  } while (p1 != p3);

  return (ngJFET > 0) && (ngJFET < dmax);
}
*/

//-------------------------------------------------------------------------
// ExecSetupMenuBIPOL
//   draw and executes the setup menu screen for a BIPOL
//-------------------------------------------------------------------------

bool ExecSetupMenuBIPOL(void) {
    
  if (s_func == 1) drawFunc(1,1,60,"NPNP",ILI9341_WHITE);  //####
  return ExecSetupMenu("BIPOL Setup", "Min I-base", "Inc I-base", &MinIbase, &IncIbase, 400, 50); // ### e
 // return ExecSetupMenu("BIPOL Setup", "Min I-base", "Max I-base", &MinIbase, &MaxIbase, 350, 50);  // ### e
}



bool ExecSetupMenuFET(void) {  // j-FET and MOS-FET 
 //DrawBox(1,1,90,12, ILI9341_BLACK);  DrawStringAt(4,8,"ExecSetupMenuFet", SmallFont, RGB(28, 128, 255));  //#############
  
  if (CurDUTclass == tcMOSFET)  {
    if (s_func == 1) drawFunc(1,1,60,"MOS ",ILI9341_WHITE);  //####
    return ExecSetupMenu("MOSFET Setup", "Min V-gate", "Inc V-gate", &MinVgate, &IncVgate, 12, 1);  // ### e
  }
  else {
    if (s_func == 1) drawFunc(1,1,60,"JFET",ILI9341_WHITE);  //####
    return ExecSetupMenu(" J-FET Setup", "Min V-gate", "Inc V-gate", &MinVgate, &IncVgate, 12, 1);  // ### e  
  }  
 // return ExecSetupMenu("  FET Setup", "Min V-gate", "Max V-gate", &MinVgate, &MaxVgate, 12, 1); // ### e
}




//-------------------------------------------------------------------------
// ExecSetupMenuDiode       // ### di // #?
//   draw and executes the setup menu screen for a Diode
//-------------------------------------------------------------------------

bool ExecSetupMenuDiode(void) {    // ### di  // #?

  if (s_func == 1) drawFunc(1,1,60,"DIO",ILI9341_WHITE);  //####
  return ExecSetupMenu("Diode Setup", "Max I-Anode", "Inc I-Anode", &MaxIanode, &IncIanode,0,0); //maxDIOa,incDIOa);  // ### e  // &IncIanode

}




//-------------------------------------------------------------------------
// ExecSetupMenu
//   draw and executes the setup menu screen
//   returns true if a DUT is inserted
//-------------------------------------------------------------------------

//bool ExecSetupMenu(char *str1, char *str2, char *str3, int *amin, int *amax, int valMax, int valInc) {
bool ExecSetupMenu(char *str1, char *str2, char *str3, int *amin, int *ainc, int valMax, int valMin) {  // ### e
 
  int16_t x, y;
  int mi,mc;
  static unsigned long time = 0;
  if (s_func == 1) drawFunc(60,1,40,"-ExSM",ILI9341_WHITE);  //####  //###########
  s_save    = 0;
  s_comp    = 0;       // ##co
  s_yfactor = 1;  // y-scale j-fet
  s_secy = 0;     // 1. scan mit s_yfactor = 1
  s_xfactor = 1;                // x-scale Diode
  if (s_auto == 1) s_secx = 1;  // for 2. run 
 start1: 

  count2_a = 0;  // ???
  count3_a = 0;  // ???
 // count2 = 0;
  while (true) {  // 1
   
    tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###

    tft.setTextColor(ILI9341_CYAN);
    if      (CurDUTclass == tcBIPOL)    tft.drawNumber(minNPNa, 180,56,4); 
    else if (CurDUTclass == tcMOSFET)   drawDec1(minMOSa,       180,56,4, ILI9341_CYAN); 
    else if (CurDUTclass == tcJFET)     drawDec1(minFETa,       180,56,4, ILI9341_CYAN);
    else if (CurDUTclass == tcDIODE)    tft.drawNumber(maxDIOa, 180,56,4);
       
     
  
    tft.fillRect(179,83,44,17, ILI9341_BLACK);        // ### ##################################
    tft.setTextColor(ILI9341_CYAN);
    if      (CurDUTclass == tcBIPOL)    tft.drawNumber(incNPNa, 180, 81, 4);   // tft.drawNumber(*amin, 180,56,4);  // ### e 24.10.18 //
    else if (CurDUTclass == tcMOSFET)   drawDec1(incMOSa,       180, 81, 4, ILI9341_CYAN);
    else if (CurDUTclass == tcJFET)     drawDec1(incFETa,       180, 81, 4, ILI9341_CYAN);
    else if (CurDUTclass == tcDIODE)    tft.drawNumber(incDIOa, 180, 81, 4);
    
    while (true) {    // 2
   /* if (count2 < 0)      {              // ### encoder PIN2,PIN4 for min-data
        *amin  = 0;                       // ### e
        count2 = 0;
      }  */
    
      if (count2 != count2_a)  {  // 3
    //  if (count2 < 0) count2 = 0; ##################################
 //       tft.fillRect(0,0,20,20,ILI9341_BLACK);
 //       tft.drawNumber(count2,0,1,2);
 //       tft.drawNumber(count2_a,50,1,2);
        tft.fillRect(179,58,44,17, ILI9341_BLACK);   // ###
        if (CurDUTclass == tcBIPOL) {         // npn / pnp  4 ##################################
    //    && (count2 != c2_BIP))    {       // 
    // count2 = c2_BIP + count2;          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
          if      (count2 <= 0)  *amin =   0;
          else if (count2 == 1)  *amin =   1; 
          else if (count2 == 2)  *amin =   2; 
          else if (count2 == 3)  *amin =   5;  
          else if (count2 == 4)  *amin =  10; 
          else if (count2 == 5)  *amin =  20; 
          else if (count2 == 6)  *amin =  30; 
          else if (count2 == 7)  *amin =  40; 
          else if (count2 == 8)  *amin =  50; 
          else if (count2 == 9)  *amin = 100; 
          else if (count2 == 10) *amin = 150;
          else if (count2 == 11) *amin = 200;
          else if (count2 == 12) *amin = 250;  
          else if (count2 >= 13) *amin = 300;  
                
          encoderpos[0] = count2;
          c2_BIP = count2;
          tft.setTextColor(ILI9341_CYAN);  tft.drawNumber(*amin, 180,56,4);  // ### e 24.10.18 70->56 ############################
          minNPNa = *amin;
    //    tft.setCursor(180,70); DrawInt(*amin, 4, ILI9341_CYAN);  // ### e 24.10.18
        }          // BIPOL 4
        else if (CurDUTclass == tcDIODE)  { // n /p-Diode or Shottky or LED or Zener < 12V  // 4
     //   && (count2 != c2_DIO))       {     
       // count2 = c2_DIO + count2;          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
          if    (count2 <= 0) *amin =  1; 
          else  {
            if (count2 <= 50) *amin = count2;
            else {
              *amin = 50;
              count2 = 50; 
            }  
          }
          encoderpos[0] = count2;
          c2_DIO = count2;
          tft.setTextColor(ILI9341_CYAN);  tft.drawNumber(*amin, 180,56,4);  // ### e 24.10.18 // 70 -> 56 ####################
          maxDIOa = *amin;
     //  tft.setCursor(180,70); DrawInt(*amin, 4, ILI9341_CYAN);  // ### e 24.10.18
        }        // Diode  4
        else   {                // j-Fet/Mosfet       4
          if   (CurDUTclass == tcJFET) {   // j-JFET  5
       //   && (count2 != c2_FET))       {  
        //  count2 = c2_FET + count2;          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
            if      (count2 > 50)  count2 = 50;
            else if (count2 <= 0)  count2 = 0;
            drawDec1(count2,180,56,4,ILI9341_CYAN);  //drawDec_1(count2,50,56,ILI9341_CYAN);  // max 5.0V
           
            *amin = count2;
            encoderpos[0] = count2;
            c2_FET = count2;
            minFETa = *amin;
          }  
          else if (CurDUTclass == tcMOSFET)  {  // MOS-FET 5
         //    && (count2 != c2_MOS))       {  
       //  count2 = c2_MOS + count2;          //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
            if      (count2 > 50) count2 = 50;
            else if (count2 <= 0) count2 = 0;
            drawDec1(count2,180,56,4,ILI9341_CYAN);  //drawDec_1(count2,50,56,ILI9341_CYAN);  // max 5.0V
           
            *amin = count2;
            encoderpos[0] = count2;
            c2_MOS = count2;
            minMOSa = *amin;
          
          } // MOS-FET
        }  // j-Fet/MOS-Fet       4
        count2_a = count2;
      }  // if (count2 = ...      3
   //################################ count3 #################################################################################    
  //   if (count3 < 0)        {              // ### encoder PIN3,PIN5 for inc-data
  //     *ainc  = 0;                         // ### e
  //     count3 = 0;   
  //   }
     if (count3 != count3_a)    {          // count3 changed
 //       if (count3 < 0) count3 = 0;
  //     tft.fillRect(24,0,20,20,ILI9341_BLACK);
  //     tft.drawNumber(count3,25,1,2);
       tft.fillRect(179,83,44,17, ILI9341_BLACK);                             // ### 
       if (CurDUTclass == tcBIPOL) {      // bipolar
                
         if      (count3 == 0) *ainc =   1; 
         else if (count3 == 1) *ainc =   2; 
         else if (count3 == 2) *ainc =   5; 
         else if (count3 == 3) *ainc =  10; 
         else if (count3 == 4) *ainc =  20;
         else if (count3 == 5) *ainc =  30;
         else if (count3 == 6) *ainc =  40;
         else if (count3 == 7) *ainc =  50;   
         else     {
           count3 = 8;
           *ainc = 100;
         }
         encoderpos[1] = count3;
         c3_BIP = count3;
         tft.setTextColor(ILI9341_CYAN);  tft.drawNumber(*ainc, 180,81,4);  // ### e 24.10.18 120->106
         incNPNa = *ainc;
     //    tft.setCursor(180,120);  DrawInt(*ainc, 4, ILI9341_CYAN);  // ### e  24.10.18
       }                         // end BIPOL
       else if (CurDUTclass == tcDIODE)   {  // Diode
    
         s_secx  = 0;                       // ?????????????????????????
         if      (count3 == 0) *ainc =  0;  // automatic: 1V,2V,3V,4V,6V,12V,24   
         else if (count3 == 1) *ainc =  1;  // 1V 
         else if (count3 == 2) *ainc =  2;  // 2V 
         else if (count3 == 3) *ainc =  3;  // 3V
         else if (count3 == 4) *ainc =  4;  // 4V
         else if (count3 == 5) *ainc =  6;  // 6V
         else if (count3 == 6) *ainc = 12;  // 12V
         else {
           count3 = 7;                      // Z??
           *ainc  = 24;                     // Z??
         }
         zen_v = *ainc;
         encoderpos[1] = count3;
         c3_DIO = count3;
         s_xfactor = 1;
         zen_v = *ainc;
         if      (*ainc ==  1) s_xfactor = 12;
         else if (*ainc ==  2) s_xfactor =  6;
         else if (*ainc ==  3) s_xfactor =  4;
         else if (*ainc ==  4) s_xfactor =  3;
         else if (*ainc ==  6) s_xfactor =  2; 
         else if (*ainc == 12) s_xfactor =  1; 
    //   else if (*ainc == 24) s_xfactor =  1; 
         else if (*ainc ==  0) s_secx = 1;   // atomatisch: 1.Lauf scant ud/uz, 2. Lauf mit angepasstem s_xfactor
   
         if (s_secx == 1)  {
           zen_v  = 24;        // start with 24V  
           *ainc  = 0;
         }       
         tft.setTextColor(ILI9341_CYAN);  tft.drawNumber(*ainc, 180,81,4);  // ### e 24.10.18 // ##################### 120->106
         incDIOa = *ainc;
         if (incDIOa == 0) s_auto = 1;
         else              s_auto = 0;  
       }  
       else {                             // MOSFET / jFET
         if      (CurDUTclass == tcMOSFET)   {
           if (count3 > 24) count3 = 24;   // maximal 2.5 V
           *ainc = count3 + 1;
           drawDec1(*ainc,180,81,4,ILI9341_CYAN);  // drawDec_1(count3,25,81,ILI9341_CYAN);  // max 2.5V
           encoderpos[1] = count3; 
           c3_MOS = count3;
         }
         else if (CurDUTclass == tcJFET)    {   
           if (count3 > 24) count3 = 24;   // maximal 2.5 V
           *ainc = count3 + 1; 
           drawDec1(*ainc,180,81,4,ILI9341_CYAN);  // drawDec_1(count3,25,81,ILI9341_CYAN);  // max 2.5V
           encoderpos[1] = count3; 
           c3_FET = count3;
         }
       }   
       count3_a = count3;
      
     }   // if (count3


       
     if (GetTouch())      {                 // touched ?
       x = xt;
       y = yt;
    //   tft.fillRect(0,0,70,20,ILI9341_BLACK);   tft.drawNumber(xt,0,1,2);     tft.drawNumber(yt,40,1,2);
       if (y < 20 + TFT_HEIGHT / 2)   {        //  oben rechts
         if (y < 80)  {   // oben  
           s_exec_pnp = 0;
           s_exec_npn = 0; 
           
           if ((x > 80) && (x < 160))  {      // > 250 "START" p  top on the left corner  // #?  
              s_exec_pnp = 1;    // rel##
              tft.setTextColor(ILI9341_RED);
              digitalWrite(BTN20, LOW);   // rel  // stop  Scan_n_Dev  
              delay(20);
              digitalWrite(BTN19, HIGH);  // rel  // start Scan_p_Dev 
              tft.drawString("OK",85,10,4);   // "SAVE"
           }
           else if (x > 240) {
              s_exec_npn = 1;    // right side: npn rel##
              
              tft.setTextColor(ILI9341_BLUE);
              digitalWrite(BTN19, LOW);  // rel##   // stop  Scan_p_Dev: pnp...  nJFet
              delay(20);
              digitalWrite(BTN20, HIGH);  // rel##  // start Scan_n_Dev: npn...  pJFet
              tft.drawString("OK",255,10,4);   // "SAVE" 
           }
           else  if (( x > 10) && ( x <= 80))   {  // links   top left for EEPROM??
             displayEEPROM(CurDUTclass);   // test_ee
             goto start1;
           }
           else  if (( x >= 160) && ( x <= 240))   {  // links   top left for EEPROM??
             updateEEPROM(CurDUTclass);   // test_ee
             goto start1;
           }
           if      (CurDUTclass == tcBIPOL)  class_old = 1;
           else if (CurDUTclass == tcMOSFET) class_old = 2;
           else if (CurDUTclass == tcJFET)   class_old = 3;
           else if (CurDUTclass == tcDIODE)  class_old = 4; 
           saveCurClass(class_old);
           saveParam(*amin,*ainc);   //#################################################################################
                        
          // tft.setTextColor(ILI9341_WHITE); tft.drawString("DUT-Class: ",20,70,4); tft.drawNumber(CurDUTclass,160,70,4); delay(500);  // test
                          
           return true;                // start new trace ??
         } 
       
         goto start1;   //return false; // ?? ######################??
       }   // y < TFT_HEIGHT / 2
       if (y > TFT_HEIGHT / 2)      {         // bottom: Diode, BIPOL , Mos, jFet
          if      (x < TFT_WIDTH / 4) {        // Diode/Zener
        
   //        if (class_old == 4) goto start1;
          // mi = count3; mc = count2; 
           class_new = 4;   //
           saveParam(*amin,*ainc);   // 2. time: save parameter old curDUTclass
           drawCheckCross(CurDUTclass,0);   // erase cross for old_class
          
     //      count3_a = 0;  //??????????????????????????????????????????????????????????????
         
           tft.fillRect(95,61,225,74,ILI9341_BLACK); //38,,97 erase old data
                      
           saveCurClass(4);     // Diode
           *amin = maxDIOa;
           *ainc = incDIOa;
           if (incDIOa == 0) {
             zen_v = 0;  // Z?? ######################################################################## !!!!##
             s_secx = 1;
           } 
           else s_secx = 0;
           encoderpos[0] = c2_DIO;
           encoderpos[1] = c3_DIO;
           count2 = c2_DIO;
           count3 = c3_DIO;
           
   //        tft.fillRect(45,0,40,20,ILI9341_BLACK);
   //        tft.drawNumber(count2,46,1,2);
   //        tft.drawNumber(count3,67,1,2);
           drawCheckCross(CurDUTclass,1);   // paint cross for cur_class  
           drawSocket();
           goto start1;     
         }
         if      (x < TFT_WIDTH / 2) {        // PNP NPN : BIPOL
        //   if (class_old == 1) goto start1;
           class_new = 1;
           saveParam(*amin,*ainc);   //
        //   mi = count3;  mc = count2;  saveParam(mi,mc);          // save parameter curDUTclass
      
           drawCheckCross(CurDUTclass,0);   // erase cross for old_class
           
           tft.fillRect(95,61,225,74,ILI9341_BLACK); // 38 ,, 97 erase old data  ##???pnp
            
           saveCurClass(1);     // NPN/PNP  
           *amin = minNPNa;
           *ainc = incNPNa;
           encoderpos[0] = c2_BIP;
           encoderpos[1] = c3_BIP;
           count2 = c2_BIP;
           count3 = c3_BIP;
           
           drawCheckCross(CurDUTclass,1);     // paint cross for cur_class  
           drawSocket();
           goto start1;     
         }
         else if (x < TFT_WIDTH * 3 / 4)  {      // MOSFET
        //   if (class_old == 2) goto start1;
           class_new = 2;
           saveParam(*amin,*ainc);          // save parameter curDUTclass
        //   mi = count2; mc = count3;  saveParam(mi,mc);          // save parameter curDUTclass
        //   saveParam(*amin,*ainc);   // 2. time: save parameter old curDUTclass
           drawCheckCross(CurDUTclass,0);   // erase cross for old_class
           
           tft.fillRect(95,61,225,74,ILI9341_BLACK); // 38,,97 erase old data
                      
           saveCurClass(2);  // MOS-FET
           *amin = minMOSa;
           *ainc = incMOSa;
           encoderpos[0] = c2_MOS;
           encoderpos[1] = c3_MOS;
           count2 = c2_MOS;
           count3 = c3_MOS;
           
           drawCheckCross(CurDUTclass,1);   // paint cross for cur_class  
           drawSocket();  
           goto start1;     
         }
         else  if (x > 10 + TFT_WIDTH * 3 / 4)  {  // JFET 
       //    if (class_old == 3) goto start1;
       //    mi = *amin;  mc = *ainc;   saveParam(*amin,*ainc);          // save parameter curDUTclass
           class_new = 3;
           saveParam(*amin,*ainc);   // 2. time: save parameter old curDUTclass
           drawCheckCross(CurDUTclass,0);   // erase cross for old_class
                  
           tft.fillRect(95,61,225,74,ILI9341_BLACK); // 38,,97 erase old data
                    
           saveCurClass(3);   // J-FET
           *amin = minFETa;
           *ainc = incFETa;
           encoderpos[0] = c2_FET;
           encoderpos[1] = c3_FET;
           count2 = c2_FET;
           count3 = c3_FET;
          
           drawCheckCross(CurDUTclass,1);   // paint cross for cur_class
           drawSocket();     
           goto start1;     
         } 
       }  // y > High/2
       
     }  // if (GetTouch
    
       
     }  // while (true
    
   }  // while true
}




//-------------------------------------------------------------------------
// MainMenuTouch
//   execute a touch command of main menu
//-------------------------------------------------------------------------

void MainMenuTouch(void) {
 
  int16_t x, y;

  if (s_func == 1) drawFunc(1,1,90,"M_M_Touch ",ILI9341_WHITE);  //####
 
  if ((!GetTouch()) && (s_start == 0))  return; // goto nxt_touch;
 // s_start = 0;
  //s_secx = 0;  //#######################################################################################
  DrawMenuScreen();
 
  x = xt; //##############################################
  y = yt; //#############################################  
 
   // ####################################################
  if (class_old == 1) goto cl_bip;   
  if (class_old == 2) goto cl_mos; 
  if (class_old == 3) goto cl_fet; 
  if (class_old == 4) goto cl_dio; 
  
   // ####################################################
  if (y > 40 + TFT_HEIGHT / 2) {  // > 160
 
    if (x < TFT_WIDTH / 4) {       // Diode/Zener
  //    tft.drawNumber(4,120,60,2);    // 4 ################
      if (CurDUTclass != tcDIODE) {  // ### di
        drawCheckCross(class_old,0); // erase old cross 
      }  
  cl_dio:    
      if (s_start == 1) {  //###<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
         count2_a = c2_DIO;
         count3_a = c3_DIO;
         count2 = c2_DIO;
         count3 = c3_DIO;
         if (count3 == 0) {
           zen_v = 24;    //###########################################################  Z??
           s_secx = 1;
         }  
         else s_secx = 0;
         encoderpos[0] = c2_DIO;
         encoderpos[1] = c3_DIO;  
         s_start = 0; 
      }
      class_cur = 4;               // 4 = Diode
      drawCheckCross(4,1);         // paint new cross
      class_old = class_cur; 
      CurDUTclass = tcDIODE;       // ### di
      curkind     = tkNothing; //###############################################??????????????????????????????????????????????????????
      DrawMenuScreen();            // ### di
      delay(100);
      if (s_start == 1) return;        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      if (ExecSetupMenuDiode())   return;  // ### di // #?
      return;
    }
  
   
    if (x < TFT_WIDTH / 2) {           // BIPOL npn/pnp
     // tft.drawNumber(1,120,60,2);     // 1 ##############
      if (CurDUTclass != tcBIPOL) { // ungleich 1 = BIPOL
        drawCheckCross(class_old,0);  // erase old cross 
      }
  cl_bip:  
      if (s_start == 1) {
         count2_a = c2_BIP;
         count3_a = c3_BIP;
         count2 = c2_BIP;
         count3 = c3_BIP;
         encoderpos[0] = c2_BIP;
         encoderpos[1] = c3_BIP;
         s_start = 0; 
      }
      class_cur = 1;                // 1 = BIPOL
 //     count2 = 0;
 //     count3 = 0;
      drawCheckCross(1,1);          // paint new cross
      class_old = class_cur; 
      CurDUTclass = tcBIPOL;
      curkind     = tkNothing; 
      DrawMenuScreen();
      delay(100);
      if (s_start == 1) return; 
      if (ExecSetupMenuBIPOL()) return;
      return;  // ??
    
    }
    else if (x < TFT_WIDTH * 3 / 4) {   // MOSFET/JFET
   //   tft.drawNumber(2,120,60,2);  // 2 ################
      if (CurDUTclass != tcMOSFET) {
        drawCheckCross(class_old,0); // erase old cross 
      }
    cl_mos:    
      if (s_start == 1) {
         count2_a = c2_MOS;
         count3_a = c3_MOS;
         count2 = c2_MOS;
         count3 = c3_MOS;
         encoderpos[0] = c2_MOS;
         encoderpos[1] = c3_MOS;
         s_start = 0; 
      }  
      class_cur = 2;               // 2 = MOSFET
      drawCheckCross(2,1);         // paint new cross
      class_old   = class_cur; 
      CurDUTclass = tcMOSFET;
      curkind     = tkNothing;
      DrawMenuScreen();
      delay(100);
      if (s_start == 1) return; 
      if (ExecSetupMenuFET()) return;  // #?
      return;  //*
    }
    else {
 //     tft.drawNumber(3,120,60,2);  // 3 ################
      if (CurDUTclass != tcJFET) {
        drawCheckCross(class_old,0); // erase old cross 
      }  
    cl_fet:  
      if (s_start == 1) {
         count2_a = c2_FET;
         count3_a = c3_FET;
         count2 = c2_FET;
         count3 = c3_FET;
         encoderpos[0] = c2_FET;
         encoderpos[1] = c3_FET;
         s_start = 0; 
      }   
      class_cur = 3;               // 3 = j-FET
      drawCheckCross(3,1);         // paint new cross
      class_old = class_cur; 
      CurDUTclass = tcJFET;
      curkind     = tkNothing;
      DrawMenuScreen();
      delay(100);
      if (s_start == 1) return; 
      if (ExecSetupMenuFET()) return;  // #?
      return;
    }
  }
/*
  if (y < 20 + TFT_HEIGHT / 2) {   // < 140 ### di
    if (x < TFT_WIDTH / 3) {       // Diode/Zener
  //    tft.drawNumber(4,120,60,2);    // 4 ################
      if (CurDUTclass != tcDIODE) {  // ### di
        drawCheckCross(class_old,0); // erase old cross 
      }  
  cl_dio:    
      if (s_start == 1) {  //###<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
         count2_a = c2_DIO;
         count3_a = c3_DIO;
         count2 = c2_DIO;
         count3 = c3_DIO;
         if (count3 == 0) {
           zen_v = 24;    //###########################################################  Z??
           s_secx = 1;
         }  
         else s_secx = 0;
         encoderpos[0] = c2_DIO;
         encoderpos[1] = c3_DIO;  
         s_start = 0; 
      }
      class_cur = 4;               // 4 = Diode
      drawCheckCross(4,1);         // paint new cross
      class_old = class_cur; 
      CurDUTclass = tcDIODE;       // ### di
      curkind     = tkNothing; //###############################################??????????????????????????????????????????????????????
      DrawMenuScreen();            // ### di
      delay(100);
      if (s_start == 1) return;        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      if (ExecSetupMenuDiode())   return;  // ### di // #?
      return;
    }
  }  
  */
}  

// ------------------------
void testPins()  {     // 
// ------------------------

     // set pin I/O direction
  pinMode(PA11,OUTPUT);        // intern USB_DM 
  digitalWrite(PA11,HIGH);
  pinMode(PA12,OUTPUT);        // intern USB_DP 
  digitalWrite(PA12,HIGH);

}

void drawDiode(uint8_t typ,uint16_t xp, uint16_t yp, int color) {
/*
  if (color != ILI9341_BLACK) {
    tft.drawRect(xp,yp,40,40,color);
    tft.fillRect(xp+1,yp+1,38,38,ILI9341_WHITE);
  }  
  */
  if (typ == 'p') {
    tft.fillTriangle(xp+10,yp+10,xp+26,yp+10,xp+18,yp+20,color);
  //  tft.drawTriangle(xp+11,yp+11,xp+25,yp+11,xp+18,yp+19,color);
    tft.fillTriangle(xp+13,yp+12,xp+23,yp+12,xp+18,yp+17,ILI9341_WHITE); //ILI9341_YELLOW);
    tft.drawFastHLine(xp+11,yp+21,16,color);
    tft.drawFastHLine(xp+11,yp+22,16,color);
  }  
  else  { 
    tft.drawFastHLine(xp+11,yp+9,16,color);
    tft.drawFastHLine(xp+11,yp+10,16,color);
    tft.fillTriangle(xp+18,yp+10,xp+10,yp+21,xp+26,yp+21,color);
    tft.fillTriangle(xp+18,yp+13,xp+13,yp+19,xp+23,yp+19,ILI9341_WHITE); //ILI9341_YELLOW);
  }  
  tft.drawFastVLine(xp+18,yp+6,5,color);
  tft.drawFastVLine(xp+19,yp+6,5,color);
  tft.drawFastVLine(xp+18,yp+22,5,color);
  tft.drawFastVLine(xp+19,yp+22,5,color);
 // delay(4000);
}

void drawArrow(uint16_t xp,uint16_t yp ,uint16_t angle, int color) {
  if (angle == 45) {  // for pnp
     xp = xp+20;
     yp= yp+20;                          
     tft.fillTriangle(xp,yp,xp-4,yp-4,xp+4,yp-8,color);
  }
   if (angle == 315) {  // for npn
     xp = xp+20;
     yp= yp+20;                          
     tft.fillTriangle(xp,yp,xp+4,yp-4,xp+8,yp+3,color);
  }
}


void drawTransistor(uint8_t typ,uint16_t xp, uint16_t yp, int color) {
  
  if (color == ILI9341_BLACK) {
    tft.drawRect(xp,yp,40,40,color);
    tft.fillRect(xp+1,yp+1,38,38,ILI9341_WHITE);
  }  
 
  if (typ  == 'p') { 
   
    tft.drawFastVLine(xp+8,yp+5,4,color);     tft.drawFastVLine(xp+9,yp+5,4,color);
    tft.drawFastVLine(xp+18,yp+9,20,color);   tft.drawFastVLine(xp+19,yp+9,20,color);
    tft.drawFastVLine(xp+8,yp+30,4,color);    tft.drawFastVLine(xp+9,yp+30,4,color);

    tft.drawFastHLine(xp+18,yp+18,11,color);  tft.drawFastHLine(xp+18,yp+19,11,color);
  
    tft.drawLine(xp+9,yp+7,xp+17,yp+15,color);   tft.drawLine(xp+9,yp+8,xp+17,yp+16,color);  tft.drawLine(xp+9,yp+9,xp+17,yp+17,color);

    drawArrow( xp-7, yp+10, 45, color);
      
    tft.drawLine(xp+9,yp+29,xp+17,yp+21,color);  tft.drawLine(xp+9,yp+30,xp+17,yp+22,color);  tft.drawLine(xp+9,yp+31,xp+17,yp+23,color);
   
    
  }
  else {
     
    tft.drawFastVLine(xp+24,yp+4,4,color);       tft.drawFastVLine(xp+25,yp+4,4,color);
    tft.drawFastVLine(xp+14,yp+9,20,color);      tft.drawFastVLine(xp+15,yp+9,20,color);
    tft.drawFastVLine(xp+24,yp+31,4,color);      tft.drawFastVLine(xp+25,yp+31,4,color);

    tft.drawFastHLine(xp+3,yp+18,11,color);      tft.drawFastHLine(xp+3,yp+19,11,color);

    tft.drawLine(xp+15,yp+15,xp+25,yp+5,color);  tft.drawLine(xp+15,yp+16,xp+25,yp+6,color);   tft.drawLine(xp+15,yp+17,xp+25,yp+7,color);
   
    tft.drawLine(xp+15,yp+20,xp+25,yp+30,color); tft.drawLine(xp+15,yp+21,xp+25,yp+31,color);  tft.drawLine(xp+15,yp+22,xp+25,yp+32,color);
    
    drawArrow( xp-3, yp+7, 315, color);  
  }  
 
}


void drawMosFet(uint8_t typ,uint16_t xp, uint16_t yp, int color) {
  
  if (color == ILI9341_BLACK) {
    tft.drawRect(xp,yp,40,40,color);
    tft.fillRect(xp+1,yp+1,38,38,ILI9341_WHITE);
  }  
 
  if (typ  == 'p') { 

    tft.drawFastVLine(xp+3,yp+5,5,color);     tft.drawFastVLine(xp+4,yp+5,5,color);
    tft.drawFastVLine(xp+3,yp+17,13,color);   tft.drawFastVLine(xp+4,yp+17,13,color);
    tft.drawFastVLine(xp+3,yp+28,4,color);    tft.drawFastVLine(xp+4,yp+28,4,color);
   
    tft.drawFastVLine(xp+13,yp+6,5,color);    tft.drawFastVLine(xp+14,yp+6,5,color);
    tft.drawFastVLine(xp+13,yp+15,6,color);   tft.drawFastVLine(xp+14,yp+15,6,color);
    tft.drawFastVLine(xp+13,yp+24,5,color);   tft.drawFastVLine(xp+14,yp+24,5,color);

    tft.drawFastVLine(xp+18,yp+7,21,color);   tft.drawFastVLine(xp+19,yp+7,21,color);

    tft.drawFastHLine(xp+4,yp+8,11,color);    tft.drawFastHLine(xp+4,yp+9,11,color);
    tft.drawFastHLine(xp+4,yp+17,11,color);   tft.drawFastHLine(xp+4,yp+18,11,color);
    tft.drawFastHLine(xp+4,yp+26,11,color);   tft.drawFastHLine(xp+4,yp+27,11,color);
    
    tft.drawFastHLine(xp+18,yp+26,11,color);  tft.drawFastHLine(xp+18,yp+27,11,color);
    
    tft.fillTriangle(xp+4,yp+17,xp+10,yp+14,xp+10,yp+20,color);     

  
    
  }
  else {

    tft.drawFastVLine(xp+30,yp+5,5,color);     tft.drawFastVLine(xp+31,yp+5,5,color);
    tft.drawFastVLine(xp+30,yp+17,14,color);    tft.drawFastVLine(xp+31,yp+17,14,color);
    tft.drawFastVLine(xp+30,yp+28,5,color);     tft.drawFastVLine(xp+31,yp+28,5,color);
     
    tft.drawFastVLine(xp+21,yp+6,5,color);     tft.drawFastVLine(xp+20,yp+6,5,color);
    tft.drawFastVLine(xp+21,yp+15,6,color);    tft.drawFastVLine(xp+20,yp+15,6,color);
    tft.drawFastVLine(xp+21,yp+24,5,color);    tft.drawFastVLine(xp+20,yp+24,5,color);

    tft.drawFastVLine(xp+16,yp+7,21,color);   tft.drawFastVLine(xp+15,yp+7,21,color);

    tft.drawFastHLine(xp+20,yp+8,11,color);   tft.drawFastHLine(xp+20,yp+9,11,color);
    tft.drawFastHLine(xp+20,yp+17,11,color);  tft.drawFastHLine(xp+20,yp+18,11,color);
    tft.drawFastHLine(xp+20,yp+26,11,color);  tft.drawFastHLine(xp+20,yp+27,11,color);

    tft.drawFastHLine(xp+5,yp+26,11,color);  tft.drawFastHLine(xp+5,yp+27,11,color);

   
    tft.fillTriangle(xp+21,yp+17,xp+27,yp+14,xp+27,yp+20,color);  
     
  }  
 
}

void drawJFet(uint8_t typ,uint16_t xp, uint16_t yp, int color) {
  
  if (color == ILI9341_BLACK) {
    tft.drawRect(xp,yp,40,40,color);
    tft.fillRect(xp+1,yp+1,38,38,ILI9341_WHITE);
  }  
 
  if (typ  == 'p') { 

    tft.drawFastVLine(xp+3,yp,9,color);       tft.drawFastVLine(xp+4,yp,9,color);
    tft.drawFastVLine(xp+3,yp+19,8,color);    tft.drawFastVLine(xp+4,yp+19,8,color);

    tft.drawFastVLine(xp+9,yp+5,16,color);    tft.drawFastVLine(xp+10,yp+5,16,color);

    tft.drawFastHLine(xp+3,yp+7,7,color);     tft.drawFastHLine(xp+3,yp+8,7,color);
    tft.drawFastHLine(xp+3,yp+17,7,color);    tft.drawFastHLine(xp+3,yp+18,7,color);
   
    tft.drawFastHLine(xp+16,yp+15,8,color);   tft.drawFastHLine(xp+16,yp+16,8,color);
   
    
    
    tft.fillTriangle(xp+11,yp+15,xp+17,yp+12,xp+17,yp+18,color);     

  
    
  }
  else {    // if (typ  == 'n') { 

    tft.drawFastVLine(xp+23,yp,9,color);       tft.drawFastVLine(xp+24,yp,9,color);
    tft.drawFastVLine(xp+23,yp+19,8,color);    tft.drawFastVLine(xp+24,yp+19,8,color);

    tft.drawFastVLine(xp+17,yp+5,16,color);    tft.drawFastVLine(xp+18,yp+5,16,color);

    tft.drawFastHLine(xp+18,yp+7,7,color);     tft.drawFastHLine(xp+18,yp+8,7,color);
    tft.drawFastHLine(xp+18,yp+17,7,color);    tft.drawFastHLine(xp+18,yp+18,7,color);
   
    tft.drawFastHLine(xp+8,yp+14,9,color);     tft.drawFastHLine(xp+8,yp+15,9,color);
   
    
    
    tft.fillTriangle(xp+8,yp+15,xp+14,yp+12,xp+14,yp+18,color);     

  
    
  }

} 

// ------------------------
void setup()  {     // 
// ------------------------

  pinMode(BOARD_LED, OUTPUT);
  // Initialize virtual COM over USB on Maple Mini
   Serial.begin(9600);    // BAUD has no effect on USB serial: placeholder for physical UART
  // wait for serial monitor to be connected.
  /*
  while (!Serial)
  {
    digitalWrite(BOARD_LED,!digitalRead(BOAR_LED)); // Turn the LED from off to on, or on to off
    delay(100);         // fast blink
  }
  Serial.println("Blink LED & count Demo");
 */
     // set pin I/O direction
  pinMode(PB4,INPUT_PULLUP);        // ## enc 1
  pinMode(PB5,INPUT_PULLUP);        // ## enc 1
//  pinMode(PB6,INPUT_PULLUP);        // ## PB3    enc 2
//  pinMode(PB7,INPUT_PULLUP);        // ## PA15 ?? enc 2
  pinMode(13,INPUT_PULLUP);        // ## PB3    enc 2
  pinMode(14,INPUT_PULLUP);        // ## PA15 ?? enc 2


  pinMode(pin_ADC0_Vo_OPV, INPUT_ANALOG);  // A0 Pin D11
  pinMode(pin_ADC1_Vc_NPN, INPUT_ANALOG);  // A1 Pin D10
  pinMode(pin_ADC2_Vc_PNP, INPUT_ANALOG);  // A2 Pin D9
  pinMode(pin_ADC3_Bat_12V,INPUT_ANALOG);  // A3 Pin D8
  pinMode(pin_ADC4, INPUT_ANALOG);         // A4 PB0 D7
  pinMode(pin_ADC5_Bat_3V, INPUT_ANALOG);  // A5 PB0 D3

  pinMode(SPI1_NSS_PIN, OUTPUT);     //pinMode(TFT_CS, OUTPUT);
  digitalWrite(SPI1_NSS_PIN, HIGH); 
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_DC,HIGH);

  pinMode(SPI2_NSS_PIN, OUTPUT); 
  digitalWrite(SPI2_NSS_PIN, HIGH); 
  
  pinMode(BOARD_LED, OUTPUT);
  //pinMode(BTN0,INPUT_PULLUP);  //#######################
  pinMode(BTN0, OUTPUT);
  digitalWrite(BTN0, LOW); 
  pinMode(BTN12, OUTPUT);
  digitalWrite(BTN12, LOW); 
  pinMode(BTN19, OUTPUT);
  digitalWrite(BTN19, LOW); 
  pinMode(BTN20, OUTPUT);
  digitalWrite(BTN20, LOW); 
  pinMode(BTN32, OUTPUT);     // test 
  digitalWrite(BTN32, HIGH);   // test
  
  //attachInterrupt(BTN0, btn0ISR, FALLING); //RISING); //CHANGE); //#######################

  amax = ADC_MAX - 1;  // max index adc
  dmax = DAC_MAX - 1;  // max index dac
  dhalf = 1 + dmax/2;

  initEncoders();                  // madias : mault_encoder_acc #############################
      
  SPI.begin();                     //Initialize the SPI_1 port.

  SPI.setClockDivider(SPI_CLOCK_DIV2);   //Set the SPI speed tft: < 20MHz
 
  tft.begin();                     //  Init Display
  tft.setRotation(3);              // LANDSCAPE);

  
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  delay(200);
// drawMosFet('p',10,10,ILI9341_AQUA);  // test
//  drawjFet('n',10,60,ILI9341_PINK);  // test
// drawMosFet('n',10,10,ILI9341_PINK);  // test
//   drawMosFet('p',70,70,ILI9341_AQUA);  // test
//  drawTransistor('p',10,10,ILI9341_RED);  // test
//  drawDiode('n',60,40,ILI9341_BLUE); // test


//delay(6000);
//delay(6000);
//delay(6000);

 /* tft.drawRect(10,10,40,40,ILI9341_RED);
  tft.fillRect(11,11,38,38,ILI9341_WHITE);
  tft.drawTriangle(20,20,36,20,28,30,ILI9341_RED);
  tft.fillTriangle(23,22,34,22,28,28,ILI9341_YELLOW);
  tft.drawFastHLine(21,31,16,ILI9341_RED);
  tft.drawFastHLine(21,32,16,ILI9341_RED);
  tft.drawFastVLine(28,16,4,ILI9341_RED);
  tft.drawFastVLine(29,16,4,ILI9341_RED);
  tft.drawFastVLine(28,32,4,ILI9341_RED);
  tft.drawFastVLine(29,32,4,ILI9341_RED);
  delay(4000);
  */
  uint16_t i5=0;
  float i5f=0.0;
  i5 = Aver4_ADC(pin_ADC5_Bat_3V); // analogRead(pin_ADC5_Bat_3V);    // pin_ADC5_Bat_3V
  delay(100);
  i5f = (float)i5;
  i5f = i5f * 0.000806884;                 // i5f * 3.305 / 4096.0;   // vref/4096
  i5f = i5f * (100.00 + 33.00)/33.00;      // 4.03
  tft.drawString("BatVolt: ",1,1,2);       // "Bat.Volt"
  tft.drawFloat(i5f,2,60,1,2);
  delay(100);
  
  SPI_2.begin();                            //Initiallize the SPI 2 port.
  SPI_2.setBitOrder(MSBFIRST);              // Set the SPI-2 bit order (*) 
  SPI_2.setDataMode(SPI_MODE0);             //Set the  SPI-2 data mode (**) 
  SPI_2.setClockDivider(SPI_CLOCK_DIV4);    //Set the SPI speed DAC: < 20MHz  : 4
  pinMode(SPI2_NSS_PIN, OUTPUT);

  delay(200);

  myTouch.InitTouch();  // 3
  myTouch.setPrecision(PREC_LOW); // PREC_MEDIUM
   
  TurnOffLoad(tkNothing);  //#######################################################
  
  delay(200);
/*  in loadDefaults() / loadConfEEPROM ??
  maxDIOa = MaxIanode; c2_DIO = 5;     // 5-ma
  incDIOa = IncIanode; c3_DIO = 5;     // 12V (x-scale: 1,2,3,4,6,12V)
  
  minNPNa = MinIbase;  c2_BIP = 3;    // 50;
  incNPNa = IncIbase;  c3_BIP = 4;    // 20;
  
  minFETa = MinV_FET;  c2_FET = 30;   // 3.0 V
  incFETa = IncV_FET;  c3_FET = 2;    // 0.2 V 

  incMOSa = IncV_MOS;  c2_MOS = 10;   // 1. komma 
  minMOSa = MinV_MOS;  c3_MOS = 2;    // 1. komma

*/
// 

 
/* 
// ################## EEPROM auslesen
  bool eraseEEPR = false;
  bool displEEPR = true;
  uint8_t ilx; 
  long count3a   = count3;
  for (ilx = 0; ilx < 100; ilx++) {
    delay(100);
    if (count3 != count3a) {
      // show EEPROM Data
      if (count3 > count3a) {  // CW
       // loadConfEEPROM(eraseEEPR,displEEPR);
        displayDataEEPROM(ILI9341_LIME);
        break;
      }
    }
  }
  if (displEEPR == true) {
    delay(7000);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
  }

 */
 if(EEPROM.init() != EEPROM_OK)  {
   loadDefaults();   // set c2_BIP,C3_BIPO etc with start-values f.i. 5
   tft.fillRect(0,1,70,12,ILI9341_BLACK);
   tft.setTextColor(ILI9341_ORANGE);
   tft.drawString("default ",2,1,2);
   delay(500);
 }
 else { 
   setParamEEPROM(); 
 
   tft.fillRect(0,1,70,12,ILI9341_BLACK);
   tft.setTextColor(ILI9341_GREEN);
   tft.drawString("EEPROM",2,1,2);  
   delay(500);
 } 
//  bool  BTNX = LOW; // ???
//  loadConfig(digitalRead(BTN0) == LOW);    // load config or factory reset to defaults
//  loadConfEEPROM((BTNX == LOW),true);  // no reset, read data from EEPROM write to C2_,c3_
 // loadConfig(BTNX == LOW);    // read EEPROM for start-values
//  loadConfig(BTNX == HIGH);    // rerest EEPROM and load start-values 

  s_first = 1;
  s_start = 1;
  s_trace = 0;
  SetDacBase(0, 20);  // base-voltage to 0V
 /* test I2C for MCP230... expander
  Wire.begin();                // wake up I2C bus
  Wire.beginTransmission(0x20);// Connect to chip  MCP23017
  Wire.write((byte)0x01);      // Select Bank B
  Wire.write((byte)0x00);      // Set all of bank B to outputs
  Wire.endTransmission();      // Close connection
  */
  digitalWrite(BTN32, HIGH);   // standard: 12V U-batt, 12 V display gain = 3
  digitalWrite(BTN0,  LOW);    // 
 
}




// ------------------------
void loop() {    // Arduino loop function
// ------------------------
static unsigned long time = 0;

 
  if (s_func == 1) drawFunc(1,1,60,"saved ",ILI9341_WHITE);  //####
  // test serial
  /*
  digitalWrite(BTN32, HIGH);   // test
  digitalWrite(BOARD_LED, HIGH);   // set the LED on
  delay(2500);              // wait for a second
  digitalWrite(BTN32, LOW);   // test
  digitalWrite(BOARD_LED, LOW);    // set the LED off
  Serial.print("Loop #: ");
  n++;
  Serial.println(n);
 //
 */
  anf_loop:
//  tft.fillRect(300,80,20,20,ILI9341_BLACK); tft.setTextColor(ILI9341_MAROON);     tft.drawNumber(s_comp,300,80,4);

  if (s_save == 1)  {  // 1 
     
    ILI9341_KindCol = ILI9341_AQUA;
    TkindDUT kind;
    kind = tkNothing;
  
    if (s_exec_pnp == 1) kind = TestDevNeg(CurDUTclass);  
    if (s_exec_npn == 1) kind = TestDevPos(CurDUTclass);  

 
    delay(500);
    if (kind == tkNothing)  {  // I hope you see this very seldom !! ##################################
      tft.fillRect(0,0,90,20,ILI9341_BLACK);
      tft.setTextColor(ILI9341_RED);tft.drawNumber(CurDUTclass,5,1,2);
      tft.setTextColor(ILI9341_ErrorCol);
      tft.drawString("Change Pins",20,1,2);
      delay(500);
      if (ILI9341_ErrorCol == ILI9341_WHITE)  ILI9341_ErrorCol = ILI9341_ORANGE;
      else                                    ILI9341_ErrorCol = ILI9341_WHITE;
      if (HaveTouch()) s_save = 0;
      goto anf_loop;
    }   
    else  ScanKind(kind); 
    
    if (HaveTouch() )  {
      if (s_trace == 1)  s_first = 1;  
      s_save  = 0;
      s_trace = 0;
    }
   
  }   // if save ...
 
  if (s_trace == 0) {
    digitalWrite(BTN0, LOW); 
    digitalWrite(BTN32, HIGH);       // 12V standard
    s_start = 1;                     //+++++++++++++++++++++++++++++++<<<<<<<<<<<<<<<<<<<<<<<<<<<<<######
    SetDacBase(0, 10);               // base-voltage to 0V
    MainMenuTouch();
    TurnOffLoad(tkNothing);
   
  }
}



