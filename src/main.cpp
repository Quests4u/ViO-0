/**
* Copyrights by @Edtronix-LABs 1992-2021
* v1.7x
*/

// Arduino: base
#include <Arduino.h>

// AVR: base
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <math.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <avr/eeprom.h>




/* INC. *************************************/

// EEPROM
#define EP_AV 1
#ifdef EP_AV
  #include <EEPROM.h>
#endif



// Posch:
#define JWA_280_AV 1
#ifdef JWA_280_AV  // Jan Wasserbauer
  #include <Bme280BoschWrapper.h>
#else
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif


// Lcd
#define LCD_AV  1
#include <LiquidCrystal.h>



/* DEF ****** available driver config ***************/


// Ed~IO
#define  LOGO                "S.N.A.P."
#define  V_BASE              "1.74"

  #ifdef JWA_280_AV
    #define V_JWA     "jw"
  #else
    #define V_JWA     "af"
  #endif
  #ifdef LCD_AV
    #define  V_LCD     "l"
  #else
    #define  V_LCD     ""
  #endif

#define  VERS                V_BASE V_JWA V_LCD

#define  DEBUG_AV            1

#define  H_                  0x1    // HIGH
#define  L_                  0x0    // LOW  -- WOT: BUG to bind #define HIGH/LOW

#define  DELAY_INIT          2000   // WOT: if LCD on dont use less than 1500ms
//#define  DELAY_MX            355    


// EEprom
#ifdef EP_AV
#define  _EP_TEST_AV  1

#define  EP_ADR_START        0x04

#define  EP_DOUBLE_DIV       10.0

#define  EP_PREM             "EdAvr"
  #define  EP_PREM_N         "EP:"
  #define  EP_PREM_L         sizeof(EP_PREM)
#define  EP_SUM              "Snap+08c15"
  #define  EP_SUM_N           "-/-"  
  #define  EP_SUM_L          sizeof(EP_SUM)

#define  EP_GET_ATSTART      1
  #define EP_GET_INIT        "EP: Init V!"
#define  EP_NONE_PUTSTD      false            // if none stored use std
 

#endif

// mode: select
#define  PIN_MS_LCD_8X2_EMU   7               // D7(i)    = PC7

//#define VA_LCD_USE_WR 1
//
#ifndef VA_LCD_USE_WR 
  #define  PIN_MS_DA_NUL        12            // D12(i) 
                                              // ^-- shared (incase) with PIN_LCD_WR (NU at mom.)
  #define  PIN_MS_DA_00         A0            // A0(i)    = PC0
#else                                              
  #define  PIN_MS_DA_NUL        A0            // A0(i)    = PC0
#endif

#define  PIN_MS_DA_99           A1            // A1(i)    = PC2

#define  PIN_MS_SENSOR          A3            // A2(i)    = PC2

#define  PIN_MS_DEBUG           A2            // A3(i)    = PC3



// Posch BME280
#ifdef  JWA_280_AV
  #define  BME_T0_DIV            100.0
  #define  BME_P0_DIV            100.0
  #define  BME_F0_DIV            1024.0
#else
  #define  BME_T0_DIV            1.0
  #define  BME_P0_DIV            100.0
  #define  BME_F0_DIV            1.0
#endif

#define    BME_MX_ON              "BME: online!"
#define    BME_MX_OFF             "BME: offline!"



// PWM: [DAC]
#define PWM_AV 1
#ifdef PWM_AV

#define  PWM_DAC_HIGH             100
#define  PWM_DAC_LOW              0
#define  PWM_DAC_NONENULL         20
//
#define  PWM_PIN_DAC_T            10             // D10(o)   = PB2/PCM(T)
#define  PWM_PIN_DAC_P            11             // D11(o)   = PB3/PCM(P)   
#define  PWM_PIN_DAC_F            9              // D9(o)    = PB1/PCM(F)


#define  PWM_T0_0             -20.0     //  -> -20C   = 0%
#define  PWM_T0_M             +80.0     //  -> +80C   = 100%

#define  PWM_P0_0             +950.0    //  -> 950    = 0%
#define  PWM_P0_M            +1030.0    //  -> 1030   = 100%

#define  PWM_F0_0             0.0       //  -> 0%     = 0%
#define  PWM_F0_M             100.0     //  -> 100%   = 100%

#endif

// Analysis
//!! N
#define _VAL_ALYS  1
#ifdef VAL_ALYS

#define  VAL_COUNT              30.0     //  -> count of integrater values

#endif



// LED:
#define  LED_TIME_SLOW        533//533
#define  LED_TIME_FAST        266//266

#define   LED_DELAY_WD_SLOW   0x02    
#define   LED_DELAY_WB_FAST   0x04
#define     LED_DELAY_WB_LS     0x01



// [NC yet] an0 -> timing isr 
#define  PIN_AN0_VCOM         6              // D6(o)    = PD6/AN0 (ACOMP-)
#define  PIN_AN0_REF          7              // D7(i)    = PD7/ANI (ACOMP+)
                                             // ^-- !! shared with MS_LCD_8X2_EMU



// Serial
#define  SIO_Baud              9600



// Command
#define COM_NONE              "[??]"
  #define  COMI_NONE             -1
#define COM_DONE              "(!)"

#define  COM_DBG_DELAY         866

#define  COM_HELP              "help"
  #define  COMI_HELP             0
  #define  COMH_HELP             "[list]"
#ifdef DEBUG_AV
  #ifdef COM_TEST_AV                      
    #define  COM_TEST            "CMtest"
      #define  COMI_TEST           1
      #define  COMH_TEST            "@"
  #endif
  #ifdef EP_TEST_AV
    #define  COM_EPTEST           "EPtest"
      #define  COMI_EPTEST         2
      #define  COMH_EPTEST          "@"
  #endif
#endif
#define  COM_VERS              "vers"
  #define  COMI_VERS             0x08
  #define  COMH_VERS             "[logo+vers]"
#define  COM_DEBUG             "debug"
  #define  COMI_DEBUG            0x09
  #define  COMH_DEBUG            "0[1]"
//
#ifdef LCD_AV
#define  COM_A_LCD             "Alcd"
  #define  COMI_A_LCD            0x10
  #define  COMH_A_LCD            ">rotate"
#define  COM_T_LCD             "Tlcd"
  #define  COMI_T_LCD            0x11
  #define  COMH_T_LCD            ">show T"
#define  COM_P_LCD             "Plcd"
  #define  COMI_P_LCD            0x12
  #define  COMH_P_LCD            ">show P"
#define  COM_F_LCD             "Flcd"
  #define  COMI_F_LCD            0x13
  #define  COMH_F_LCD            ">show F"
#endif
//
#ifdef EP_AV
#define  COM_EP_SAVE           "EPsave"
  #define  COMI_EP_SAVE          0x20
  #define  COMH_EP_SAVE            ">store now"
#define  COM_EP_LOAD           "EPload"
  #define  COMI_EP_LOAD          0x21
  #define  COMH_EP_LOAD            ">set stored"
#define  COM_EP_STD            "EPstd"
  #define  COMI_EP_STD            0x22
  #define  COMH_EP_STD            ">reset STD."
#endif
//
#ifdef PWM_AV
#define  COM_EP_T_PWM          "Tpwm"
  #define  COMI_EP_T_PWM          0x30
  #define  COMH_EP_T_PWM            "[min,max]"
#define  COM_EP_P_PWM           "Ppwm"
  #define  COMI_EP_P_PWM           0x31
  #define  COMH_EP_P_PWM            "[min,max]"
#define  COM_EP_F_PWM           "Fpwm"
  #define  COMI_EP_F_PWM           0x32
  #define  COMH_EP_F_PWM            "[min,max]"
#endif

#define  COM_S_PARA             "SPara"
  #define  COMI_S_PARA           0x40
  #define  COMH_S_PARA            "1[0]"



// CParser
#define   CP_BIN                2
#define   CP_DEC                10
#define   CP_HEX                16

#define    CP_TRUE              "true"
#define    CP_FALSE             "false"

#define    CP_ON                "on"
#define    CP_OFF               "off" 


#define  CP_COMI0   1
#ifdef   CP_COMI0
  #define  CP_COMI1   1       // -> only max 2 param
  #ifdef   CP_COMI1
    #define   _CP_COM2   1
    #ifdef    CP_COM2
      #define   CP_COM3   1
    #endif
  #endif
#endif





// LCD: [HD44780 comp] -> MS_LCD_8X2 emu
#ifdef  LCD_AV

#define  PIN_LCD_RS           13             // D13(o)   = LCD set (1)
#ifdef VA_LCD_USE_WR
  #define  PIN_LCD_WR           12             // D12(o)   = LCD 0_write/1_read
  //^-- const low for write (used otherwise)
#endif
#define  PIN_LCD_E            8              // D8(o)    = LCD enable
//
#define  PIN_LCD_D4           5              // D5(o)    = LCD (hData) D4-7 
#define  PIN_LCD_D5           4              // D4(o)
#define  PIN_LCD_D6           3              // D3(o)
#define  PIN_LCD_D7           2              // D2(o)

#define  LCD_CDADR_LINE1       LCD_SETDDRAMADDR|0x40       // SETDDRAM => 0x80
//#define  LCD_CDADR_LINE1       (0x80|0x40)       // SETDDRAM => 0x80




#define  LCD_MS_AUTO_SI       0
#define  LCD_MS_T             1
#define  LCD_MS_P             2
#define  LCD_MS_F             3

#define  LCD_CLS_DELAY        250//ms

#endif


/*** STATIC / DYNAMIC  Data Collection - Transfer **********************/



// ED~IO
const String       _logo                  = "S.N.A.P.";
String       _ver                   = VERS;      
int                 _delayInit      = DELAY_INIT;

const String              _tx_S0_noTPF    = "-1.0,-1.0,-1.0;";
String              _tx_BME_noTPF   = BME_MX_OFF;
String              _tx_BME_hasTPF  = BME_MX_ON;

const String              _tx_noVal       = "no values!";

bool              dbgMain           = false;
bool              dbgCom            = false;// -> just on _dbg(false)



// EEProm:
#ifdef EP_AV
bool              EP_none_putstd    = EP_NONE_PUTSTD;

#endif


// Serial:
String            SO_line   = String(   "012345678901234567890123456789");//30
//String            SI_line   = String(   "0123456789012345678901234567");//28

const String            SO_HEAD        = "T;P;F@";


bool              SO_para     = true; // -> put parameter regular on SO_line

// Command:
String            comP        = String( "01234567");    // command
int               comP_i      = 0;
int               com_m       = 8;

//
bool              comPD       = true;                   // proc/data
bool              comPD_en    = true;                   // -> (Data) use char 
                                                       // incase ignore until next synch (,:,@)

String            comD        = String( "01234567");  // data
int               comD_i      = 0;
int               comD_m      = 10;


int               comDcou     = 0;
// %0..%2[3]
String            com_d0       = String("01234567");
int               com_i0       = -1;
  bool              com_i0Has  = false;
bool              com_f0       = false;
  bool              com_f0Has  = false;

#ifdef  CP_COMI1
String            com_d1       = String("01234567");
int               com_i1       = -1;
  bool              com_i1Has  = false;
bool              com_f1       = false;
  bool              com_f1Has  = false;
#endif

#ifdef CP_COMI2
String            com_d2       = String("01234567");
int               com_i2       = -1;
  bool              com_i2Has  = false;
bool              com_f2       = false;
  bool              com_f2Has  = false;
#endif


#ifdef CP_COMI3
String            com_d3       = String("01234567");
int               com_i3       = -1;
  bool              com_i3Has  = false;
bool              com_f3       = false;
  bool              com_f3Has  = false;
#endif

// com-string/ix/info
bool              comHas      = false;
int               comIX       = -1;
bool              comDelayed  = false;
//
const String            com_A[]   ={ 
                      COM_HELP,
#ifdef DEBUG_AV  
  #ifdef COM_TEST_AV                                          
                        COM_TEST, 
  #endif
  #ifdef EP_TEST_AV                     
                        COM_EPTEST,                      
  #endif
#endif
                      //
                      COM_VERS,
                      COM_DEBUG,
                      //
#ifdef LCD_AV
                      COM_A_LCD,
                      COM_T_LCD,
                      COM_P_LCD,
                      COM_F_LCD,
#endif
                      //
#ifdef EP_AV
                      COM_EP_SAVE,
                      COM_EP_LOAD,
                      COM_EP_STD,
#endif
                      //
#ifdef PWM_AV
                      COM_EP_T_PWM,
                      COM_EP_P_PWM,
                      COM_EP_F_PWM,
#endif

                      COM_S_PARA
                      //
                      };
const String          com_H[]   ={
                      COMH_HELP,
#ifdef DEBUG_AV
  #ifdef COM_TEST_AV                      
                        COMH_TEST,
  #endif
  #ifdef EP_TEST_AV
                        COMH_EPTEST,
  #endif
#endif
                      //
                      COMH_VERS,
                      COMH_DEBUG,
                      //
#ifdef LCD_AV
                      COMH_A_LCD,
                      COMH_T_LCD,
                      COMH_P_LCD,
                      COMH_F_LCD,
#endif
                      //
#ifdef EP_AV
                      COMH_EP_SAVE,
                      COMH_EP_LOAD,
                      COMH_EP_STD,
#endif
                      //
#ifdef PWM_AV
                      COMH_EP_T_PWM,
                      COMH_EP_P_PWM,
                      COMH_EP_F_PWM,
#endif

                      COMH_S_PARA
                      //
                      };
int               com_I[]  ={
                      COMI_HELP,
#ifdef DEBUG_AV
  #ifdef COM_TEST_AV                      
                        COMI_TEST,
  #endif
  #ifdef EP_TEST_AV
                        COMI_EPTEST,                      
  #endif
#endif
                      //
                      COMI_VERS,
                      COMI_DEBUG,
                      //
#ifdef LCD_AV
                      COMI_A_LCD,
                      COMI_T_LCD,
                      COMI_P_LCD,
                      COMI_F_LCD,
#endif
                      //
#ifdef EP_AV
                      COMI_EP_SAVE,
                      COMI_EP_LOAD,
                      COMI_EP_STD,
#endif
                      //
#ifdef PWM_AV
                      COMI_EP_T_PWM,
                      COMI_EP_P_PWM,
                      COMI_EP_F_PWM,
#endif
      
                      COMI_S_PARA
      
                      //
                  };
int               com_am  = -1;//<-- init with setup  
int               com_ai  = -1;

String            com_line        = String("012345678901234567890123");    // output collector
int               com_lineDelay   = COM_DBG_DELAY;



// CParser:
bool              cpHas      = false;
int               cp_0base  = 10;
int               cp_0       = 0;
bool              cp_0sgn    = true;//pos[neg]



// PWM: [DAC]
int                 _PWM_DAC_null     = PWM_DAC_LOW;         

int               pwm_t_value     = -1;
int               pwm_p_value     = -1;
int               pwm_f_value     = -1;

double            pwm_t_0         = PWM_T0_0;
double            pwm_t_M         = PWM_T0_M;
//
double            pwm_p_0         = PWM_P0_0;
double            pwm_p_M         = PWM_P0_M;
//
double            pwm_f_0         = PWM_F0_0;
double            pwm_f_M         = PWM_F0_M;


// Analyses:: Parameter
#ifdef VAL_ALYS

int               xCount          =0,               
                  MCount          =30;


float             Tx              =0.0,
                  dTx           =0.0,
                  dTdif         =0.0,
                  Px              =0.0,
                  dPx           =0.0,
                  dPdif         =0.0,
                  Fx              =0.0,
                  dFx           =0.0,
                  dFdif         =0.0,
                  xLimit        =0.0;

#endif

// LED: -> minimum Out

bool                _LedHas       = false;
bool                _LedState     = false;
int                 _LedValue     = LOW;
//
int                 _LedTime      = 0;

bool                _LedAuto_SW   = true;



// AN[0]   
int                 _AN0_count    = 0;
int                 _AN0_time     = 1000;



// Sensor: posch BME280 <-- sadly just one Senor supported!
// ^-- WOT:: else some kind of overflow on String -> LCD OPS!!!

#ifdef JWA_280_AV
Bme280BoschWrapper  BME0_280        (true);
//--> Dont try!! -> Bme280BoschWrapper  BME280_1        (true);
#else
Adafruit_BME280     BME0_280;
//--> Dont try!! -> Adafruit_BME280     BME280_1;
#endif
bool                _BME0_Has      = false;
bool                _BME0_ValuesE  = false;
bool                _BME0_Values   = false;


// -- in general
int                 _BME_I2C[4]   = {   0x66,0x67,
                                        0x76,0x77
                                    };
int                 _BME_I2C_b    = 0;                          
int                 _BME_I2C_e    = 4;                          



//  LCD (HD 44780 comp.)
#ifdef LCD_AV

LiquidCrystal  LCD( 
      PIN_LCD_RS,
      PIN_LCD_E,
      PIN_LCD_D4,
      PIN_LCD_D5,
      PIN_LCD_D6,
      PIN_LCD_D7
);

String        lcd_line        = String("012345678901234567890123");


struct  dim_t{
  int  w;
  int  h;
};
bool                _lcdHas       = false;

dim_t               _lcdSize      = {  8, 2 };     // Real Config
bool                _lcdEMU       = true;          // emulation 8x2 -> 16x1

dim_t               _lcdBox       = { 16, 1 };     // vir. dimension
bool                _lcdCenter    = true;          // center view


bool                _lcdRefresh   = false;// WOT:  refresh busted LCD each 3rd delay loop(5)
  int                 _lcd_wot_i    = 0;
  int                 _lcd_wot_m    = 3;


int                 _lcd_ms_auto    = 0;//<-rotate  LCD_MS_T;// --> rotate T->P->F
                                             //            ^------
int                 _lcd_MS_rotate  = 1;                 

#endif


#ifdef EP_AV
/* EEProm: *********************************/

#define EP_ADR( _v)   (EP_ADR_START+_v)

int EP_prem_adr  = EP_ADR( 0);
struct  EP_prem_t{
  //
  bool   Valid;
  //
  char   prem[6];
  char   sum[16];
  //                        
};
//
union EP_prem_u{
  EP_prem_t  t;
  uint8_t     b[sizeof(EP_prem_t)];
};
//
EP_prem_u  EP_prem;


int EP_bulk_adr     = EP_ADR( sizeof( EP_prem_t));
struct EP_bulk_t{
  //
  bool  Valid; 
  //
  /**   define  storeable values (raw!!) ! ********/
  /*            (fill below)                      */
  //
  bool  dbgCom;
  //
  // lcd
  int   lcd_ms_auto;
  //
  // PWM
  int   pwm_t_0;
  int   pwm_t_M;
  int   pwm_p_0;
  int   pwm_p_M;
  int   pwm_f_0;
  int   pwm_f_M;
  //
  // Serial
  // 
  bool  SO_para;
  //
  /**    ^-- define above               *****/
  char  prem[6];
  //
};
//
union EP_bulk_u{
  EP_bulk_t   t;
  uint8_t     b[sizeof(EP_bulk_t)];
};
EP_bulk_u  EP_bulk;

  /** insert fillxxx  meth. with std/load/save values! ***/

          void _EP_fillStdValues(){ 
            //
            dbgCom                = false;
            //
            _lcd_ms_auto          = LCD_MS_T;
            //
            pwm_t_0               = PWM_T0_0;
            pwm_t_M               = PWM_T0_M;
            pwm_p_0               = PWM_P0_0;
            pwm_p_M               = PWM_P0_M;
            pwm_f_0               = PWM_F0_0;
            pwm_f_M               = PWM_F0_M;
            //
            SO_para               = true;
          }

          void _EP_fillValues2Bulk(){
            //
            EP_bulk.t.dbgCom      = dbgCom;
            //
            EP_bulk.t.lcd_ms_auto = _lcd_ms_auto;
            //
            EP_bulk.t.pwm_t_0     = pwm_t_0;//*EP_DOUBLE_DIV;
            EP_bulk.t.pwm_t_M     = pwm_t_M;//*EP_DOUBLE_DIV;
            EP_bulk.t.pwm_p_0     = pwm_p_0;//*EP_DOUBLE_DIV;
            EP_bulk.t.pwm_p_M     = pwm_p_M;//*EP_DOUBLE_DIV;
            EP_bulk.t.pwm_f_0     = pwm_f_0;//*EP_DOUBLE_DIV;
            EP_bulk.t.pwm_f_M     = pwm_f_M;//*EP_DOUBLE_DIV;
            //
            EP_bulk.t.SO_para     = SO_para;
            //
          }

          void _EP_fillBulk2Values(){
            //
            dbgCom              = EP_bulk.t.dbgCom;
            //
            _lcd_ms_auto        = EP_bulk.t.lcd_ms_auto;
            //
            pwm_t_0             = EP_bulk.t.pwm_t_0;///EP_DOUBLE_DIV;
            pwm_t_M             = EP_bulk.t.pwm_t_M;///EP_DOUBLE_DIV;
            pwm_p_0             = EP_bulk.t.pwm_p_0;///EP_DOUBLE_DIV;
            pwm_p_M             = EP_bulk.t.pwm_p_M;///EP_DOUBLE_DIV;
            pwm_f_0             = EP_bulk.t.pwm_f_0;///EP_DOUBLE_DIV;
            pwm_f_M             = EP_bulk.t.pwm_f_M;///EP_DOUBLE_DIV;
            //
            SO_para             = EP_bulk.t.SO_para;//
          }



#endif



/* SUB *************************************/



#ifdef EP_AV
// EEprom: ***********

      void _EP_resetPrem(){
        //
        EP_prem.t.Valid  = false;
        memcpy( EP_prem.t.prem, EP_PREM_N, EP_PREM_L);
        memcpy( EP_prem.t.sum, EP_SUM_N, EP_SUM_L);
        //
      }
      void _EP_getPrem(){
        //
          uint8_t   _b  = 0;
          int  _m  = sizeof(EP_prem_t);
          int  _o  = -1;
          for( int _i=0; _i<_m; _i++){
            //
            _o=_i+EP_prem_adr;
            _b=EEPROM.read( _o);
            EP_prem.b[_i]=_b;
          } 
      }
      void _EP_setPrem(){
        //
        EP_prem.t.Valid  = true;
        memcpy( EP_prem.t.prem, EP_PREM, EP_PREM_L);
        memcpy( EP_prem.t.sum, EP_SUM, EP_SUM_L);
        //
          uint8_t  _b   = 0;
          int _m   = sizeof(EP_prem_t);
          int _o   = -1;
          for( int _i=0; _i<_m; _i++){
            //
            _o=_i+EP_prem_adr;
            _b=EEPROM.read( _o);
            if (EP_prem.b[_i]!=_b){
              //
              _b=EP_prem.b[_i];
              EEPROM.write( _o, _b);
            }
          } 
      }
      bool _EP_checkPrem(){
        //
        if (EP_prem.t.Valid){
          //
          int  _m=EP_SUM_L;
          int _i=memcmp( EP_prem.t.sum, EP_SUM, _m);
          //
          //WOT:! 
          //_i=1;
          return (_i==0);
        }
        return false;
      }
    
      bool _EP_checkBulk(){
        //
        if (EP_bulk.t.Valid){
          //
          int  _m=EP_PREM_L;
          int _i=memcmp( EP_bulk.t.prem, EP_PREM, _m);
          //
          return (_i==0);
        }
        return false;
      }



/**
 *  @param _read .. true/false [read from EEP. / use std.]
 */
bool EP_loadValues( bool _read){
  //
  bool _f     =false;
  bool _fData =false;
  // select
  if (_read){
    //
    _EP_getPrem();
    //
    if (_EP_checkPrem()){
      //
      uint8_t _b    = 0;
        int  _m = sizeof(EP_bulk_t);
        int  _o = -1;
        for( int _i=0; _i<_m; _i++){
          //
          _o=_i+EP_bulk_adr;
          _b=EEPROM.read( _o);
          EP_bulk.b[_i]=_b;
        }       
      _fData=_EP_checkBulk();
    }
    else{
      //
      _EP_resetPrem();
    }
  }
  else{
    //
    _EP_fillStdValues();
    _f=true;
  }
  // 
  // strategie
  // fill in bulk -> var
  if (_fData){
    //
    _EP_fillBulk2Values();
    _f=true;
  }
  else{
    if (EP_none_putstd){
      //
      _EP_fillStdValues();
      //_f=true??? maybe
    }
  }
  //
  return _f;
}


bool EP_saveValues(){
  //
  bool _f=false;
  //
  // prem: refreah
  _EP_setPrem();
  //
    // fillin var -> store
    _EP_fillValues2Bulk();
    // put closing prem;
    memcpy( EP_bulk.t.prem, EP_PREM, EP_PREM_L);
    // set Valid
    EP_bulk.t.Valid=true;
      //
      uint8_t _b    = 0;
        int  _m = sizeof(EP_bulk_t);
        int  _o = -1;
        for( int _i=0; _i<_m; _i++){
          //
          _o=_i+EP_bulk_adr;
          _b=EEPROM.read( _o);
          if (EP_bulk.b[_i]!=_b){
            //
            _b=EP_bulk.b[_i];
            EEPROM.write( _o, _b);
          }
        }
      //
      _f=true;
  return _f;
}


#endif


// ISR:   ************

  /** WatchDog
   */
  ISR( WDT_vect){
    //
    // system should restart.
    // loop should repeat.
    //
  }


  /** AComp0
   */
  ISR( ANALOG_COMP_vect  ){
    //
    // system should restart.
    // loop should repeat.
    //
    if (_AN0_count<_AN0_time){
      //
      _AN0_count=0;
    }
    else{
      //
      _AN0_count++;
    }
  }



// WDT:  *****************

/**
 *  WDT:
 */
void _delaySetWD(
            int _pwm,
            int _time,
            bool _state
          ){
    noInterrupts();
      //
      _LedAuto_SW=true;
      int _WD_time=LED_DELAY_WD_SLOW+LED_DELAY_WB_LS;  // <-- 0.5s
      //
      WDTCSR =   _BV(WDCE)      // do changes to WDTCSR
                |_BV(WDIF)      // clear WDIF sigflag
                |_BV(WDE)       // after IF set Reset (should never happen)
                |_BV( WDIE)     // set IF (WD_vect)
                //
                |_WD_time
      ;
      //
    interrupts();
}


//  ANx:  ************

/**
 *  AN0:
 */
void  _setAN(){
    //
    int _x=0;
    noInterrupts();
      // wrt
      _x = // _BV( ACD)
           //|_BV( ACBG)
           //|_BV( ACIC)
           //|_BV( ACIS0)
           _BV( ACIE)
          |_BV( ACIS1)
      ;
      ACSR = _x;      
      // 
      //
    interrupts();
}


// PWR/SLEEP:  **********


/**
 *  POWER/SLEEP: 
 */
void  _sleepMode(){
  //
  noInterrupts();//cli
  //
    set_sleep_mode( SLEEP_MODE_ADC);
    //
    sleep_enable();
    sleep_bod_disable();
      //
      interrupts();//sti ->  WDT_vect
      //
      sleep_cpu();
      sleep_disable();
  //
}


// PWM:   ***********
#ifdef PWM_AV

/**
 * 
 *  @return  delay(0-time)
 */
int  _PWM_dependentDelay( 
          int _pwm, 
          int _time,
          bool _state
          ){
    //    
    double _x=_pwm;
    int _s1=round((_x/100)*_time);
    if (_state){
        //
        return _s1;
    } 
    else{
        //
        return _time+1-_s1;
    }       
}



/**
 * 
 * @param  _pwm         .. 0.00-100.00%
 * @param  _none_null   .. nonull offset
 * @return level(0-256)
 */
int _PWM_getLevel( 
            double _pwm,
            int _none_null
          ){
    //
    double _v=0.0;
    if (_pwm>0){
      _v=100.0;
      if (_pwm<_v){
        _v=_pwm;
      } 
    }
    int _=round( (_v*(255.0-_none_null))/100.0)+_none_null;
    return _;
}
  
/**
 * set test value to all valid DACs
 * @return _pwm
 */
int _PWM_setTestDAC( 
            double _v
          ){
    //
    int _=_PWM_getLevel( _v, _PWM_DAC_null);
    
    //
    analogWrite( PWM_PIN_DAC_T, _);
    analogWrite( PWM_PIN_DAC_P, _);
    analogWrite( PWM_PIN_DAC_F, _);
    //
    return _;
}


/**
 * set DA[C] within Min-Max BoxParameters
 * @see pcm_X_0/d
 */
bool _PWM_setDAC( 
            double _T,
            double _P,
            double _F
          ){
    //
    bool   _f=true;
    int     _=-1;
    double  _v=0,
            _pwm_t_d=pwm_t_M-pwm_t_0,
            _pwm_p_d=pwm_p_M-pwm_p_0,
            _pwm_f_d=pwm_f_M-pwm_f_0
            ;


    // T: 
      _v=(_T-pwm_t_0)*100.0/_pwm_t_d;
        _=_PWM_getLevel( _v, _PWM_DAC_null);
    pwm_t_value=_;
      analogWrite( PWM_PIN_DAC_T, _);
    // P:
    _v=(_P-pwm_p_0)*100.0/_pwm_p_d;
      _=_PWM_getLevel( _v, _PWM_DAC_null);
    pwm_p_value=_;
      analogWrite( PWM_PIN_DAC_P, _);

    // F: 
    _v=(_F-pwm_f_0)*100.0/_pwm_f_d;
      _=_PWM_getLevel( _v, _PWM_DAC_null);
    pwm_f_value=_;
      analogWrite( PWM_PIN_DAC_F, _);

    //
    return _f;
}

#endif


// BME280: ***********

/**
 *  find I2C of BME280
 */
bool _BME_find280(){
  //
  bool _f=false;
  int _i2c, 
      _i=_BME_I2C_e;
      //
      while( _i>_BME_I2C_b){
        _i--;
        //
        _i2c=_BME_I2C[_i];
#ifdef JWA_280_AV
        _f=BME0_280.beginI2C( _i2c);
#else
        _f=BME0_280.begin( _i2c);
#endif
        if (_f){
          //
          _i=_BME_I2C_b;
        }
      }
  return _f;
}




// LCD: ***********


#ifdef LCD_AV



/**
 *  modus of the lcd
 *  (Needs to be fitted spezial to each LCD!)
 */
bool  _LCD_setup( 
            bool _emu
                ){
    // speczal fit
    if (_emu){
      _lcdSize.w=8;
      _lcdSize.h=2;
    }
    else{
      _lcdSize.w=16;
      _lcdSize.h=1;
    }

    // modus                  
    LCD.begin( 
          _lcdSize.w, 
          _lcdSize.h,
          LCD_5x8DOTS
          );
    //
    LCD.noBlink();
    //
    LCD.noCursor();
    //
  return true;
}


/**
 * std/emu print of home single line.
 * WOT!!  no auto mode just emu/ or not
 * each LCD needs different ajustments to add.
 * WOT!! 2020-07-20_01! (no chance to fix troubles with, String+LCD!)
 */
bool  _LCD_0print2( 
                int    _delay,
                bool   _clr
                ){
  //
 bool _f=false;
  // 
  if (!_lcdHas){ return _f;}// exception!
  //
  if (_clr){
    //
    LCD.clear();
  }

    //
    char    _c=' ';
    uint8_t _b=0;
    int   _i=0,
          _h=0,
          //
          _Lbox=_lcdBox.w,
          _L=lcd_line.length(),
          _l=0, 
          _=0;
    //
    if      (_L<0){
      _L=0;
    }
    else if (_L>=_Lbox){
      _L=_Lbox;
    }
    //
    if (_lcdCenter){
      _=(_lcdBox.w-_L)>>1;// center gap
    }
    //
    while( (!_f)&
           (_h<_lcdSize.h)
         ){
      //
      _i=0;
      while( (!_f)& 
             (_i<_lcdSize.w)
            ){
        //  
        _b=lcd_line.charAt( _l);
        _c=(char)_b;
        //
        if ((_c>0)&
            (_l<_L)){
          if (_>0){
            //
            LCD.print( ' ');
            _--;
          }
          else{
            LCD.print( _c);
            _l++;
          }
          _i++;
          //delay( 1);
        }
        else{
          _f=true;
        }
      }
      // NextLine
      _h++;
      LCD.send( LCD_CDADR_LINE1, LOW);
      //delay( 1);
      //
    }
    //
    if (_delay>0){
      //
      delay( _delay);
    }
  return _f;
}
#endif



// cp: parser *******


bool cpReset( bool _sgn){
    //
    cpHas     =false;
    cp_0      =0;
    cp_0sgn   =_sgn;// pos/neg
    cp_0base  =10;
  //
  return true;
}


      int _cpAdd( 
            char _c,
            int  _ofs
            ){
        int _i=0;
          //
          if (cpHas){
            //
            cp_0=cp_0*cp_0base;
          }
          else{
            //
            cpReset( cp_0sgn);
            cpHas=true;
          }
          _i=_c-_ofs;
          cp_0+=_i;
          //
        return _i; 
      }



bool cpCParse( 
            char _c
            ){
    //
    bool _f=false;
    //
    // Gk: -> just k
    if ((_c>=65)&&//'A'
        (_c<=91)){//'Z'
      _c+=0x20;// 'a'
    }
    // select Num
    switch( _c){
      //
    case '+':
            //cp_0sgn=true;
          break;
    case '!':
    case '-':
            cp_0sgn=!cp_0sgn;
            //cp_0sgn=false;
          break;          

    case '0':
    case '1':
          if (cp_0base==CP_BIN){//2
            //
            _cpAdd( _c, 48);// '0'
            break;
          }
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
          _cpAdd( _c, 48);// '0'
        break;
    case 'b':
          if ((cpHas)&& 
              (cp_0==0)){ // "0b"
            //
            cp_0base=CP_BIN;//2
            break;
          }
    case 'a':
    case 'c':
    case 'e':
    case 'f':
          if (cp_0base==CP_HEX){//16
            //
            _cpAdd( _c, 97);// 'a'
          }
          else{
            _f=cpHas;
          }
        break;
        //
    case 'x':
          if ((cpHas)&& 
              (cp_0==0)){ // "0x"
            //
            cp_0base=CP_HEX;
          }
          else{
            //
            _f=cpHas;
          }
        break;
    default:
          _f=cpHas;
        break;
    }
    //
    return _f;
}


// COM:  ***********




bool comReset(){
    //
    comP    = "";
    comP_i   = 0;
    comD    = "";
    comD_i  = 0;
    //
    comPD     = true;
    comPD_en  = true;
    comDcou   = 0;
    //
    com_d0   = "";
      com_i0   = -1;
        com_i0Has = false;
      com_f0   = false;
        com_f0Has = false;
#ifdef CP_COMI1
    com_d1   = "";
      com_i1   = -1;
        com_i1Has = false;
      com_f1   = false;
        com_f1Has = false;
#endif
#ifdef CP_COMI2
    com_d2   = "";
      com_i2   = -1;
        com_i2Has = false;
      com_f2   = false;
        com_f2Has = false;
#endif
#ifdef CP_COMI3
    com_d3   = "";
      com_i3   = -1;
        com_i3Has = false;
      com_f3   = false;
        com_f3Has = false;
#endif
    //
    cpReset( true);
    //
    comHas  = false;
    comIX   = COMI_NONE;
    com_ai  = -1;
    //
    return true;
}


      bool _comCParseI( 
                    char _c
                    ){ 
        //
        bool _f=cpCParse( _c);
          //
            if (_f){
              //
              // bool
              bool _fB=(cp_0==0)||(cp_0==1);
              // pos[neg]
              if (!cp_0sgn){
                //
                cp_0=-cp_0;
              }
              //
              switch( comDcou){
            case 0: com_i0=cp_0;//_cp0;
                    com_i0Has=true;
                    if (_fB){
                      com_f0=cp_0==1;
                      com_f0Has=true;
                    }
                break;
  #ifdef CP_COMI1
            case 1: com_i1=cp_0;
                    com_i1Has=true;
                    if (_fB){
                      com_f1=cp_0==1;
                      com_f1Has=true;
                    }
                break;
  #endif
  #ifdef CP_COMI2
            case 2: com_i2=cp_0;
                    com_i2Has=true;
                    if (_fB){
                      com_f2=cp_0==1;
                      com_f2Has=true;
                    }
                break;
  #endif
  #ifdef CP_COMI3
            case 3: com_i3=cp_0;
                    com_i3Has=true;
                    if (_fB){
                      com_f3=cp_0==1;
                      com_f3Has=true;
                    }
                break;
  #endif
              }//switch
              //
              cpReset( true);
            }
        return _f;
      } 


      bool _comBV( bool _isBool){
              // true:true
              if (comD==CP_TRUE){
                  return true;
              }
              else if (comD==CP_ON){
                  return true;
              }
              // true:false
              if (comD==CP_FALSE){
                  return _isBool;
              }
              if (comD==CP_OFF){
                  return _isBool;
              }
              //
              return false;  
      }

      /*
      bool _comBV( bool _isBool){
              if (_v){
                if (comD==CP_TRUE){
                  return true;
                }
              }
              else{
                if (comD==CP_FALSE){
                  return true;
                }
              }
              return false;  
      }
      */

      /*
      bool _comHasB(){
              //
              if ((_comBV( true))|
                  (_comBV( false))){
                return true;
              }
              else{
                return false;
              }
      }
      */

      bool _comDAdd(){ 
              //
              //bool _fB=_comHasB();
              bool _fB=_comBV( true);
              //
              switch( comDcou){
            case 0: com_d0=comD;
                    if (_fB){
                      com_f0    =_comBV( false);
                      com_f0Has =true;
                    }
                break;
    #ifdef CP_COMI1
            case 1: com_d1=comD;
                    if (_fB){
                      com_f1    =_comBV( false);
                      com_f1Has =true;
                    }
                break;
    #endif
    #ifdef CP_COMI2
            case 2: com_d2=comD;
                    if (_fB){
                      com_f2    =_comBV( false);
                      com_f2Has =true;
                    }
                break;
    #endif
    #ifdef CP_COMI3
            case 3: com_d3=comD;
                    if (_fB){
                      com_f3    =_comBV( false);
                      com_f3Has =true;
                    }
                break;
    #endif
              default:
                comDcou--;
              }
              comDcou++;
              //
              comD="";
              comD_i=0;
        return true;  
      } 




/**
 *  parse _c and concats com proc/date [true/false] 
 *  @return true  .. a full com is complete to execute
 */
bool comParseC( 
            char _c
            ){
      //
      bool _f=false,
           _fC=false,
           _fSep=false;

      // select:
      switch( _c){
        // .. eoC
    case '\n':
    case  '@':
    case  ';':
          for (int _i=0;_i<com_am; _i++){
            //
            if (com_A[_i]==comP){
              //
              com_ai=_i;
              comIX=com_I[_i];
              comHas=true;
              //
              if (_comCParseI( '@')){
                //
              }
              _comDAdd();
            }
          }
          //
          _f=comHas;
        break;
        // .. param
    case '=':
    case ':':
          if (comPD){
            //
            comPD=false;
            comPD_en=true; 
          }
        break;
    case ',':
    case '|':
          if (!comPD){
            //
            _fSep=true;
            comPD_en=true; 
          }
          else{
            //
            comPD_en=false; 
          }
     default:
          _fC=true;
        break;
    }

    // add:
    if (comPD){
      //
      if (comP_i<com_m){
        //
        if (comPD_en){
          //
          if (isWhitespace(_c)){
            //
            // -> exclude pre^WS on comP
          }
          else{
            //
            comP+=_c;
            comP_i++;
          }
        }
      }
    }
    else{
      //
      if ((comPD_en)&&
          (comD_i<comD_m)){  
        //
        if (_comCParseI( _c)){
          //
          if (!_fSep){
            //
            comPD_en=false; //--> no parse until sep
            _fC=false;
          }
        }
        if (_fSep){
          //
          _comDAdd();
        }
        else{
          //
          if (_fC){
            //
            comD+=_c;
            comD_i++;
          }
        } 
        //
      }
    }
    //
  return _f;
}



      bool _com_checkMinLTMax( int _d){
        //
        bool _f=false;
          if ((com_i0Has)&
              (com_i1Has)){
            //
            if (com_i1>(com_i0+_d)){
              _f=true;
            }
          }
        return _f;
      }


      /**
       * find com_ai(index) for com_I[] value
       *@param  _comI   ..  value of com_I[] 
       */
      int _com_ai4com_I( int _comI){
          //
          int _ai=0;
          //
          for (int _i=0;_i<com_am; _i++){
              //
              if (com_I[_i]==_comI){
                //
                _ai=_i;
                break;
              }
          }
          return _ai;
      }




      /**
       *  com_line: fill with com_* parameter [min,max]
       *@param  _ai   ..  com_ai <- index <- com_A[] = com_I[]
       */
      void _com_MinMax2Line( int _ai){
        //
          com_line.concat( com_A[_ai]);
          com_line.concat( ':');
              switch( com_I[_ai]){
          case COMI_EP_T_PWM:
                com_line.concat( (int)pwm_t_0);
                com_line.concat( ',');
                com_line.concat( (int)pwm_t_M);
              break;
          case COMI_EP_P_PWM:
                com_line.concat( (int)pwm_p_0);
                com_line.concat( ',');
                com_line.concat( (int)pwm_p_M);
              break;
          case COMI_EP_F_PWM:
                com_line.concat( (int)pwm_f_0);
                com_line.concat( ',');
                com_line.concat( (int)pwm_f_M);
              break;
            }//switch
      }//Get 




      


/**
 * execute element selected from comIX.
 * enabled by comHas
 * @return true .. %done
 */
bool comExec(){
    //
    bool  _f=false,
          _fAddCL=false,
          _fLCD=dbgMain,
          _fSOut=false;
    int   _i=0;
    //
    com_line="";
    //
    if (comHas){
      //
      switch( comIX){
        // debug just onable
  case COMI_DEBUG:
          if (com_f0Has){
            //
            dbgCom=com_f0;
            //
            _fAddCL=true;
            _f=true;
          }
          break;
#ifdef DEBUG_AV
          // test
#ifdef COM_TEST_AV                      
        
  case COMI_TEST:
          com_line=">";
          com_line.concat( com_i0);
          com_line.concat( '|');
          com_line.concat( com_d0);
          com_line.concat( ';');
          com_line.concat( com_i1);
          com_line.concat( '|');
          com_line.concat( com_d1);
          com_line.concat( ';');
          com_line.concat( com_i2);
          com_line.concat( '|');
          com_line.concat( com_d2);
          com_line.concat( ';');
          com_line.concat( com_am);
          //
          _f=true;
        break;
#endif          

#ifdef EP_TEST_AV
  case COMI_EPTEST:
          //
          if (com_f0Has){
            //
            if (com_f0){
              //
              // put check
              //_EP_setPrem();
                //
              _fAddCL=true;
              _f=true;
              //
            }
            else{
              //
              // read 
              _EP_getPrem();
              //
              int __f=-1;
              if (_EP_checkPrem()){
                //
                __f=2;
              }
              if (_EP_checkBulk()){
                //
                __f=3;
              }
              //
              int _m=sizeof(EP_prem_t);
              com_line.concat( _m);
                com_line.concat( ';');
              com_line.concat( EP_prem.t.prem);
                com_line.concat( ';');
              com_line.concat( __f);
                com_line.concat( ';');
              //
              _f=true;
            }
            //
          }
          //
          break;
#endif
        //
#endif
        // show logo 
  case COMI_VERS:
          //
          com_line.concat( LOGO);
          com_line.concat( VERS);
          //
          _fLCD=true;
          _f=true;
        break;
        // show help info
  case COMI_HELP:
          _i=0;// <-- index in
          //
          if (_i==0){
            //
            Serial.println( COMH_HELP);
            for (int _i=1; _i<com_am; _i++){
              //
              Serial.print( com_A[_i]);
              Serial.print( '=');
              Serial.println( com_H[_i]);
            }
          }
          //
          com_line="";
          com_line.concat( com_A[_i]);
          com_line.concat( '=');
          com_line.concat( com_H[_i]);
          //
          _f=true;
        break;
#ifdef EP_AV
        // EEProm: put(save)
  case COMI_EP_SAVE:
          //
          if (EP_saveValues()){
            //
            _f=true;
            _fAddCL=true;
            _fLCD=true;
          }
          else{
            com_line.concat( _tx_noVal);
          }
        break;
        // EProm: get(set to loaded)
  case COMI_EP_LOAD:
          //
          if (EP_loadValues( true)){
            //
            _f=true;
            _fAddCL=true;
            _fLCD=true;
          }
          else{
            //
            com_line.concat( _tx_noVal);
          }
        break;
        // EEprom: get(reset to std)
  case COMI_EP_STD:
          //
          _f=EP_loadValues( false);
          _fAddCL=true;
          _fLCD=true;
          break;
#endif
        // on auto: show->T 
#ifdef LCD_AV
  case COMI_A_LCD:
          _lcd_ms_auto=0;
          //
          _fAddCL=true;
          _fLCD=true;
          _f=true;
        break;
  case COMI_T_LCD:
          _lcd_ms_auto=LCD_MS_T;
          //
          _fAddCL=true;
          _f=true;
        break;
        // on auto: show->P 
  case COMI_P_LCD:
          _lcd_ms_auto=LCD_MS_P;
          //
          _fAddCL=true;
          _f=true;
        break;
        // on auto: show->F 
  case COMI_F_LCD:
          _lcd_ms_auto=LCD_MS_F;
          //
          _fAddCL=true;
          _f=true;
        break;
#endif


  case COMI_EP_T_PWM:
  case COMI_EP_P_PWM:
  case COMI_EP_F_PWM:
          if (_com_checkMinLTMax(10)){
              //
              switch( comIX){
          case COMI_EP_T_PWM:
                pwm_t_0=com_i0;
                pwm_t_M=com_i1;
              break;
          case COMI_EP_P_PWM:
                pwm_p_0=com_i0;
                pwm_p_M=com_i1;
              break;
          case COMI_EP_F_PWM:
                pwm_f_0=com_i0;
                pwm_f_M=com_i1;
              break;
              }
          }
          //
          _com_MinMax2Line( com_ai);
          _fLCD=true;
          _fSOut=true;
          _f=true;
        break;

  case COMI_S_PARA:
          if (com_f0Has){
            //
            SO_para=com_f0;
            //
            _fAddCL=true;
            _fLCD=true;
            _f=true;
          }
        break;


      }//comIX


      // -> put parsed result -> [pr,ix]
        if (_fAddCL){
          //
          if ((com_ai>-1)&&
              (com_ai<com_am)){
            //
            com_line.concat( com_A[com_ai]);
            if (com_d0.length()>0){
              //
              com_line.concat( '<');
              com_line.concat( com_d0);
            }
    #ifdef CP_COMI1
            if (com_d1.length()>0){
              //
              com_line.concat( ',');
              com_line.concat( com_d1);
            }
    #endif
    #ifdef CP_COMI2
            if (com_d2.length()>0){
              //
              com_line.concat( ",..");
            }
    #endif
            com_line.concat( COM_DONE);
          }
          else{
            //
            com_line.concat( COM_NONE);
          }
          //
        }

        //
#ifdef LCD_AV
      //
      lcd_line="";
      if (_fLCD){
        //
        lcd_line.concat( com_line);
        _LCD_0print2( com_lineDelay, true);
        lcd_line="";
      }
#endif
      // Out: Serial[CL]
        if (_fSOut){
          //
          Serial.println( com_line);
        }
      //
    }
    // HadCom
    return _f;
}






/* SERIAL:  *************/


/**
 * Serial: data in Evt
 */
void serialEvent(){
  //
  int   _m=Serial.available();
  char  _c=0;
  int   _iC=0;
    //
    //SI_line="";
    comReset();
    //
    for ( ;_m>0; _m--){
      //
      _c=(char)Serial.read();
      if (comParseC( _c)){
        //
        if (comExec()){
          //
          _iC++;
          // <-- done/check/ oelse!
        }
        //
        comReset();
      }
    }
  //

}



/* MAIN: ***************/



/**
 *  INITIALISATION
 */
//
void setup() {
  //
  // ed~IO
  //
  // MS: modeselect
#ifdef LCD_AV
  pinMode( PIN_MS_LCD_8X2_EMU, INPUT_PULLUP);
#endif

#ifdef PWM_AV
  // PWM: DAC->STD/+NULL
  pinMode( PIN_MS_DA_NUL, INPUT_PULLUP);
#endif

#ifndef VA_LCD_USE_WR
    pinMode( PIN_MS_DA_00, INPUT_PULLUP);
#endif

  pinMode( PIN_MS_DA_99, INPUT_PULLUP);
  // Sensor
  pinMode( PIN_MS_SENSOR, INPUT_PULLUP);
  //
  pinMode( PIN_MS_DEBUG, INPUT_PULLUP);


  // PCM: [DAC]
#ifdef PWM_AV
  pinMode( PWM_PIN_DAC_T, OUTPUT);
  pinMode( PWM_PIN_DAC_P, OUTPUT);
  pinMode( PWM_PIN_DAC_F, OUTPUT);
    //
    _PWM_DAC_null=0;
    _PWM_setTestDAC( _PWM_DAC_null);
#endif


  // -- VCOM
  pinMode( PIN_AN0_VCOM, OUTPUT);     // --> AIN1 = (PWM out [~1kHz])
  analogWrite( PIN_AN0_VCOM, 100);

  // PWM
  _setAN();
  
  // Serial
  Serial.begin( SIO_Baud);
    SO_line="";
    SO_line.concat( _logo);
    SO_line.concat( _ver);
      Serial.println( SO_line);
    //SI_line="";
    

  // -- LCD
#ifdef LCD_AV
  #ifdef VA_LCD_USE_WR
    pinMode( PIN_LCD_WR, OUTPUT);
    digitalWrite( PIN_LCD_WR, LOW);
  #endif
  _lcdEMU=(digitalRead( PIN_MS_LCD_8X2_EMU)==LOW);
      //
      //_lcdHas=
      //_lcdEMU=true;
      _lcdHas=_LCD_setup( _lcdEMU);
      //
          lcd_line="";
          lcd_line.concat( _logo);
          lcd_line.concat( _ver);
          _LCD_0print2( _delayInit, true);
          //
#endif

  // LED: test
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite( LED_BUILTIN, LOW);
  //
  //
  //

  // com[mand]:
  com_am=sizeof(com_I)/sizeof(COMI_HELP);
  com_ai=0;
  //

  // EEprom
#ifdef EP_AV

  EP_bulk.t.Valid=true;// bulk -> valid(load/save)
  //
  if (EP_GET_ATSTART){
    //
    if (EP_loadValues( true)){
      //
      SO_line="";
      SO_line.concat( EP_GET_INIT);
    }
    else{
      //
      SO_line="";
      SO_line.concat( EP_PREM_N);
      SO_line.concat( _tx_noVal);
    }
    //
    Serial.println( SO_line);

#ifdef LCD_AV
    lcd_line="";
    lcd_line.concat( SO_line);
    _LCD_0print2( com_lineDelay, true);
#endif
    //
      
  }
#endif
}




/**
 * Thread  Loop
 */
int  _iLoop = 0;
int  _mLoop = 10;
//
void loop() {

int _Pwm  =50;//%
    //
int   _time =LED_TIME_SLOW,
      _PwmB=_Pwm,
      _PwmValue=-1;

  // sensor values
#ifdef JWA_280_AV
  int32_t _T=0,
          _F=0,
          _P=0;
#else
  float   _T=0.0,
          _F=0.0,
          _P=0.0;
#endif
  float   _T0,
          _F0,
          _P0
          //
  ;
  // visual
  bool    _vis=false;
  bool    _dbg=false;
  bool    _lcdRefreshed=false;

  // mod-select
#ifdef LCD_AV
  bool  _LCD_emu  =(digitalRead( PIN_MS_LCD_8X2_EMU)==LOW);
#endif
      //
  int   _DA_99    =digitalRead( PIN_MS_DA_99),
        _DA_NUL   =digitalRead( PIN_MS_DA_NUL),
#ifndef VA_LCD_USE_WR
        _DA_00    =digitalRead( PIN_MS_DA_00),
          _ms= ~(_DA_00 | _DA_99<<1) & 0x03,
#else
          _ms= ~(_DA_NUL | _DA_99<<1) & 0x03,
#endif
      //
      _SN       =digitalRead( PIN_MS_SENSOR),
      //
      _DBG      =digitalRead( PIN_MS_DEBUG)
    ;
  //
  // global



  // internal: loop counter
  if ((_iLoop++)>=_mLoop){
    //
    _iLoop=0;
#ifdef LCD_AV
    if ((_lcd_wot_i++)>_lcd_wot_m){
      //
      _lcd_wot_i=0;
      _lcdRefresh=!_lcdRefreshed;
    }
#endif

  }

  // led:   
  _LedHas=true;



  // debug: select
  if (_DBG==LOW){
    //
    _dbg=true;
  }
  else{
    //
    _dbg=dbgCom;// <-- from comLine (just on no dbgJumper)
  }
  //
  dbgMain=_dbg;


  // PWM: DAC(SPS) null level(0V) / plus level(20% = 2V / 4mA)
#ifdef PWM_AV
  if (_DA_NUL==LOW){
    //
    _PWM_DAC_null=_PWM_getLevel( PWM_DAC_NONENULL, 0);
    _time=LED_TIME_FAST;
  }
  else{
    //
    _PWM_DAC_null=0;
    _time=LED_TIME_SLOW;
  }
#endif


#ifdef LCD_AV
  // LCD:  STD/EMU   display setup
  if ((_lcdRefresh)
     |(_LCD_emu!=_lcdEMU)
     ){
      //
      //
      _lcdEMU=_LCD_emu;
      _lcdHas=_LCD_setup( _lcdEMU);
      _lcdRefresh=false;
      _lcdRefreshed=true;
      //
      //LCD.clear();
      //delay( LCD_CLS_DELAY);
  }
#endif


  //  M[ODE]S[ET]: PINS / Posch BMExx / Out LED
  //
  if ( _SN==LOW){// ~suspend
    //
    // check BME
    if (_BME0_Has){
      //
      _LedHas=false;
      _PwmB=0;//clear=0%
    }
    else{
      //
      _BME0_Has=_BME_find280();
      if (!_BME0_Has){
        //
        _LedHas=true;
        _time =LED_TIME_FAST,
        _PwmB=10;//%
      }
      else{
        //
        _LedHas=false;
        _iLoop=_mLoop-1;
      }
    }
    // BME: measure
    if (_BME0_Has){
      //
#ifdef  JWA_280_AV      
      _BME0_Values=BME0_280.measure();
#else
      _BME_Values=BME0_280.init();
#endif
      if (!_BME0_Values){
          //
          _LedHas=true;
          _time =LED_TIME_FAST,
          _PwmB=5;//%
      }
      //
    }
    else{
      //
      //_BME_Values=false;--> ll be cleared later

    }
    //
  }
  else{
    //
    // Test Modus
    _BME0_Has=false;

    // PWM: test select 
    //          _  .. 99%/0% or NUL ->(0%/20%)
    //
#ifdef PWM_AV
    if      (_DA_99==LOW){ 
      //
      _Pwm=PWM_DAC_HIGH;
      _PwmB=75;//%
      _time=LED_TIME_FAST;
    }
#ifndef VA_LCD_USE_WR
    else if (_DA_00==LOW){
#else  
    else{
#endif
      //
      _Pwm=PWM_DAC_LOW;
      _PwmB=25;//%
      _time=LED_TIME_FAST;
    }
#endif
  }   


  //  DAC: (BME ON && VALUES)  PWM 0-2: Values -> T0,P0,F0
  if ((_BME0_Has)&&
      (_BME0_Values)){
    //
#ifdef JWA_280_AV
          _T =BME0_280.getTemperature();
          _P =BME0_280.getPressure();
          _F =BME0_280.getHumidity();
    _T0= (_T*1.0)/BME_T0_DIV;   // 100.0  -> C
    _P0= (_P*1.0)/BME_P0_DIV;   // 100.0  -> mB
    _F0= (_F*1.0)/BME_F0_DIV;   // 1024.0 -> %
#else
          _T =BME0_280.readTemperature();
          _P =BME0_280.readPressure();
          _F =BME0_280.readHumidity();
    _T0= (_T*1.0)/BME_T0_DIV;   // 1.0  -> C
    _P0= (_P*1.0)/BME_P0_DIV;   // 100.0  -> mB
    _F0= (_F*1.0)/BME_F0_DIV;   // 1.0 -> %
#endif
    _PWM_setDAC( _T0, _P0, _F0);
  }
  else{
    //
    //  PWM:  -> jumpe setup to PWM0-2
    _PwmValue=_PWM_setTestDAC( _Pwm);
    //
    _T0 = _Pwm/100.0*(pwm_t_M-pwm_t_0)+pwm_t_0;
    _P0 = _Pwm/100.0*(pwm_p_M-pwm_p_0)+pwm_p_0;
    _F0 = _Pwm/100.0*(pwm_f_M-pwm_f_0)+pwm_f_0;
    //
    pwm_t_value=_PwmValue;
    pwm_p_value=_PwmValue;
    pwm_f_value=_PwmValue;
    //
    //
  }
  //
#ifdef VAL_ALYS
  // relativ werte abnahme
  dTx+=Tx-_T0;
  dPx+=Px-_P0;
  dFx+=Fx-_F0;
#endif  




  // 1.) delayed visual with Loop==0(0-6 oe.)
  //
  if (_iLoop==0){
    //
    // analysis parameter
#ifdef VAL_ALYS
    xCount++;

    if (xCount==MCount){
        //
        xCount=0;
        //
        dTdif=dTx/xLimit;
        dPdif=dPx/xLimit;
        dFdif=dFx/xLimit;
        //
        dTx=0;
        dPx=0;
        dFx=0;
    }
#endif

    // rotate
    _lcd_MS_rotate=_lcd_MS_rotate+1;
    if (_lcd_MS_rotate>3){
      _lcd_MS_rotate=1;
    }
    //

    _vis=true;
    // BME Check
    if (_BME0_Values!=_BME0_ValuesE){ 
        // measure broke:
        // S0:
        if (_dbg){
          if (_BME0_Values){   
            Serial.println( _tx_BME_hasTPF);
          }
          else{               
            Serial.println( _tx_BME_noTPF);
          }
        }
        else{ 
            SO_line="";    
            SO_line.concat( SO_HEAD);
            SO_line.concat(_tx_S0_noTPF);        
            Serial.println( SO_line);
        }
        //
        _BME0_ValuesE=_BME0_Values; //-> clear
        //delay( DELAY_MX);
        //
        _BME0_Has=false;           //-> rescan sensor!
        _vis=false;
    } 

    // standard parameter serial out
    if ((_vis)
       &(SO_para)
       ){
        // head:
        SO_line="";
        SO_line.concat( SO_HEAD);

          // T
          if (_dbg)   {   
            SO_line.concat( pwm_t_value);
          }
          else{
            SO_line.concat( _T0);
          }
          SO_line.concat( ';');
          // P
          if (_dbg){      
              SO_line.concat( pwm_p_value);
          }
          else{
            SO_line.concat( _P0);
          }
          SO_line.concat( ';');
          // F
          if (_dbg){       
            SO_line.concat( pwm_f_value);
          }
          else{
            SO_line.concat( _F0);
          }
        //
        Serial.println( SO_line);
      //
    }// std. visual
    //
  }// iLoop


  // 2.) prompt visual each loop
  // 
  // check 
  _vis=true;
  if (_BME0_Values!=_BME0_ValuesE){
    //
      lcd_line="";
      if (_BME0_Values){
#ifdef LCD_AV
        lcd_line.concat(_tx_BME_hasTPF);
#endif
      }
      else{
#ifdef LCD_AV
        lcd_line.concat(_tx_BME_noTPF);
#endif
      }
        // lcd:
#ifdef LCD_AV
      _LCD_0print2( -1, true);
#endif
      //_BME_Has=false;  -> setback by iLoop check --^
      _lcdRefresh=!_lcdRefreshed;
      _vis=false;
    //
  }//V-VE




  // internal visual out
  // rotation
      if      (_ms==0){
        // auto
        _ms=_lcd_ms_auto;
        // incase -> rotate value put
        if (_ms==0){
          //
          _ms=_lcd_MS_rotate;
          //
          if ((_iLoop==0)
             ){
              //
              _lcdRefresh=!_lcdRefreshed;
              _vis=_lcdRefreshed;
          }
        }
      }


  if (_vis){
    // LCD
#ifdef LCD_AV
    lcd_line="";
    // prepare

    // fetch lcd_line
    if(_SN==LOW){

      // select value to put
      if (_ms==1){
        //
        lcd_line.concat( "T "); 
        lcd_line.concat( _T0);
        if (_dbg){
          lcd_line.concat( '/');
          lcd_line.concat( pwm_t_value);
        }
        else{
          lcd_line.concat( 'C');
        }
      }
      else if (_ms==2){
        //
        lcd_line.concat( "P "); 
        lcd_line.concat( _P0);
        if (_dbg){
          lcd_line.concat( '/');
          lcd_line.concat( pwm_p_value);
        }
        else{
          lcd_line.concat( "mB");
        }
      }
      else if (_ms==3){
        //
        lcd_line.concat( "F "); 
        lcd_line.concat( _F0);
        if (_dbg){
          lcd_line.concat( '/');
          lcd_line.concat( pwm_f_value);
        }
        else{
          lcd_line.concat( '%');
        }
          //
      }
      else{
        //
        lcd_line.concat( _tx_noVal);
      }
    }
    else{
      //
      if (_dbg){
        //
        // select put
        int _comI=COMI_HELP;
        if (_ms==1){
          _comI=COMI_EP_T_PWM;
        }
        else if (_ms==2){
          _comI=COMI_EP_P_PWM;
        }
        else if (_ms==3){
          _comI=COMI_EP_F_PWM;
        }
        //
        com_ai=_com_ai4com_I( _comI);
        com_line="";
        _com_MinMax2Line( com_ai);
        lcd_line.concat( com_line);
      }  
      else{
        // Test PWM out
        lcd_line.concat( "PWM: ");
        lcd_line.concat( _Pwm);
        lcd_line.concat( "% [");
        lcd_line.concat( _PwmValue);
        lcd_line.concat( ']');
      }
      //
    }// collect lcd_line;
    //
    _LCD_0print2( -1, true);
    //
#endif
  }


  //  LED:  just to control
  if (_LedHas){
    // led:on
    if (_LedState){
      //
      if (!_LedAuto_SW){
        //
        _LedValue=HIGH;
      }
      else{
        // led: chg -> off
        _LedValue=LOW;
        _LedState=false;
      }
    }// led:off
    else{
      //
      if (!_LedAuto_SW){
        //
        _LedValue=LOW;
      }
      else{
        // led: chg -> on
        _LedValue=HIGH;
        _LedState=true;
      }
    }   
  }
  else{
      //
      _LedValue=LOW;
      _LedState=false;
  }

  // led: -> set pin
  digitalWrite( LED_BUILTIN, _LedValue);
  //

  //  Delay:   
  _LedTime=_PWM_dependentDelay( _PwmB, _time, _LedState);
  delay( _LedTime);

  //  ISR..   
    //
    //_delaySetWD( _PwmB, _time, _LedState);
    //_LedAT_SW=true;
    //wdt_reset();
    //wdt_enable( WDTO_500MS);
    
    //  _delay_us( 250);
    //  _sleepMode();
    
}
