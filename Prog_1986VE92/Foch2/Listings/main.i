#line 1 "main.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"





















 

 







 
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"

















 





















 

 



#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_lib.h"





















 

 







#line 59 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_lib.h"



 










 
#line 50 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 51 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"

#line 58 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"





 
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"





















 

 





 



 



 






  #pragma anon_unions


 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn     = -14,   
  HardFault_IRQn          = -13,   
  MemoryManagement_IRQn   = -12,   
  BusFault_IRQn           = -11,   
  UsageFault_IRQn         = -10,   
  SVCall_IRQn             = -5,    
  PendSV_IRQn             = -2,    
  SysTick_IRQn            = -1,    

 
  CAN1_IRQn               =  0,    
  CAN2_IRQn               =  1,    
  USB_IRQn                =  2,    
  DMA_IRQn                =  5,    
  UART1_IRQn              =  6,    
  UART2_IRQn              =  7,    
  SSP1_IRQn               =  8,    
  I2C_IRQn                =  10,   
  POWER_IRQn              =  11,   
  WWDG_IRQn               =  12,   
  Timer1_IRQn             =  14,   
  Timer2_IRQn             =  15,   
  Timer3_IRQn             =  16,   
  ADC_IRQn                =  17,   
  COMPARATOR_IRQn         =  19,   
  SSP2_IRQn               =  20,   
  BACKUP_IRQn             =  27,   
  EXT_INT1_IRQn           =  28,   
  EXT_INT2_IRQn           =  29,   
  EXT_INT3_IRQn           =  30,   
  EXT_INT4_IRQn           =  31    
}IRQn_Type;



 

 





   

 
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
 







 

























 
























 




 


 

 













#line 108 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 







#line 138 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

#line 140 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 











 









 









 









 











 











 











 







 










 










 









 






#line 685 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cmInstr.h"

   

#line 141 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 307 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cmFunc.h"


#line 632 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cmFunc.h"

 


#line 142 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"








 
#line 172 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

 






 
#line 188 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

 












 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 




#line 415 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1[1];

} SCnSCB_Type;

 



 










 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 






























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1246 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

#line 1255 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"






 










 

 



 




 










 

static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 3)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 3)) & 0xff);    }         
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 3)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 3)));  }  
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<3) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}

 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}



 









#line 99 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"
#line 1 "C:\\SOFT_project\\Mk\\Prog_1986VE92\\Foch2\\RTE\\Device\\MDR1986BE92\\system_MDR32F9Qx.h"




















 



 



 



 

 





 

extern uint32_t SystemCoreClock;          
 

   



 

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

   



   

   

   



 
#line 100 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"



 

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus;



typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

   



 



 



 

 
typedef struct
{
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
}MDR_CAN_BUF_TypeDef;

 
typedef struct
{
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
}MDR_CAN_BUF_FILTER_TypeDef;

 
typedef struct
{
  volatile uint32_t CONTROL;
  volatile uint32_t STATUS;
  volatile uint32_t BITTMNG;
       uint32_t RESERVED0;
  volatile uint32_t INT_EN;
       uint32_t RESERVED1[2];
  volatile uint32_t OVER;
  volatile uint32_t RXID;
  volatile uint32_t RXDLC;
  volatile uint32_t RXDATAL;
  volatile uint32_t RXDATAH;
  volatile uint32_t TXID;
  volatile uint32_t TXDLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
  volatile uint32_t BUF_CON[32];
  volatile uint32_t INT_RX;
  volatile uint32_t RX;
  volatile uint32_t INT_TX;
  volatile uint32_t TX;
       uint32_t RESERVED2[76];
    MDR_CAN_BUF_TypeDef CAN_BUF[32];
       uint32_t RESERVED3[64];
    MDR_CAN_BUF_FILTER_TypeDef CAN_BUF_FILTER[32];
}MDR_CAN_TypeDef;

   



 



  

 
 






 






   



  

 
 
#line 220 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 236 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 251 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 259 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 
 
 



 



   



  

 
 
 
 
#line 314 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 322 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
 
 





 





   



  

 
 
 
 





 





   



  

 
 
#line 381 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 391 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
}MDR_USB_SEP_TypeDef;

 
typedef struct
{
  volatile uint32_t RXFD;
       uint32_t RESERVED0;
  volatile uint32_t RXFDC_L;
  volatile uint32_t RXFDC_H;
  volatile uint32_t RXFC;
       uint32_t RESERVED1[11];
  volatile uint32_t TXFD;
       uint32_t RESERVED2[3];
  volatile uint32_t TXFDC;
       uint32_t RESERVED3[11];
}MDR_USB_SEP_FIFO_TypeDef;

 
typedef struct
{
  volatile uint32_t HTXC;
  volatile uint32_t HTXT;
  volatile uint32_t HTXLC;
  volatile uint32_t HTXSE;
  volatile uint32_t HTXA;
  volatile uint32_t HTXE;
  volatile uint32_t HFN_L;
  volatile uint32_t HFN_H;
  volatile uint32_t HIS;
  volatile uint32_t HIM;
  volatile uint32_t HRXS;
  volatile uint32_t HRXP;
  volatile uint32_t HRXA;
  volatile uint32_t HRXE;
  volatile uint32_t HRXCS;
  volatile uint32_t HSTM;
       uint32_t RESERVED0[16];
  volatile uint32_t HRXFD;
       uint32_t RESERVED1;
  volatile uint32_t HRXFDC_L;
  volatile uint32_t HRXFDC_H;
  volatile uint32_t HRXFC;
       uint32_t RESERVED2[11];
  volatile uint32_t HTXFD;
       uint32_t RESERVED3[3];
  volatile uint32_t HTXFC;
       uint32_t RESERVED4[11];
    MDR_USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
    MDR_USB_SEP_FIFO_TypeDef USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
}MDR_USB_TypeDef;

   



 



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 
#line 570 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 580 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 
#line 618 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 628 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 






 






   



  

 
 






 






   



  

 
 
#line 708 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 718 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}MDR_EEPROM_TypeDef;

   



 



  

 
 
#line 782 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 796 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CLOCK_STATUS;
  volatile uint32_t PLL_CONTROL;
  volatile uint32_t HS_CONTROL;
  volatile uint32_t CPU_CLOCK;
  volatile uint32_t USB_CLOCK;
  volatile uint32_t ADC_MCO_CLOCK;
  volatile uint32_t RTC_CLOCK;
  volatile uint32_t PER_CLOCK;
  volatile uint32_t CAN_CLOCK;
  volatile uint32_t TIM_CLOCK;
  volatile uint32_t UART_CLOCK;
  volatile uint32_t SSP_CLOCK;
}MDR_RST_CLK_TypeDef;

   



 



  

 
 




 




   



  

 
 
#line 863 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 871 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 
#line 996 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1004 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 





 





   



  

 
 





 





   

   

   



 



 

 
typedef struct
{
  volatile uint32_t STATUS;
  volatile uint32_t CFG;
  volatile uint32_t CTRL_BASE_PTR;
  volatile uint32_t ALT_CTRL_BASE_PTR;
  volatile uint32_t WAITONREQ_STATUS;
  volatile uint32_t CHNL_SW_REQUEST;
  volatile uint32_t CHNL_USEBURST_SET;
  volatile uint32_t CHNL_USEBURST_CLR;
  volatile uint32_t CHNL_REQ_MASK_SET;
  volatile uint32_t CHNL_REQ_MASK_CLR;
  volatile uint32_t CHNL_ENABLE_SET;
  volatile uint32_t CHNL_ENABLE_CLR;
  volatile uint32_t CHNL_PRI_ALT_SET;
  volatile uint32_t CHNL_PRI_ALT_CLR;
  volatile uint32_t CHNL_PRIORITY_SET;
  volatile uint32_t CHNL_PRIORITY_CLR;
       uint32_t RESERVED0[3];
  volatile uint32_t ERR_CLR;
}MDR_DMA_TypeDef;

   



 



  

 
 





 





   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t DR;
  volatile uint32_t RSR_ECR;
       uint32_t RESERVED0[4];
  volatile uint32_t FR;
       uint32_t RESERVED1;
  volatile uint32_t ILPR;
  volatile uint32_t IBRD;
  volatile uint32_t FBRD;
  volatile uint32_t LCR_H;
  volatile uint32_t CR;
  volatile uint32_t IFLS;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_UART_TypeDef;

   



 



  

 
 






 






   



  

 
 





 





   



  

 
 
#line 1214 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1225 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1241 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1250 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1271 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1285 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 
#line 1320 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1333 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1353 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1366 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1386 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1399 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1419 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1432 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_SSP_TypeDef;

   



 



  

 
 






 






   



  

 
 





 





   



  

 
 






 






   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 



 



   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t PRL;
  volatile uint32_t PRH;
  volatile uint32_t CTR;
  volatile uint32_t RXD;
  volatile uint32_t STA;
  volatile uint32_t TXD;
  volatile uint32_t CMD;
}MDR_I2C_TypeDef;

   



 



  

 
 




 




   



  

 
 






 






   



  

 
 
#line 1713 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1721 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t PVDCS;
}MDR_POWER_TypeDef;

   



 



  

 
 
#line 1763 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1774 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}MDR_WWDG_TypeDef;

   



 



  

 
 



 



   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}MDR_IWDG_TypeDef;

   



 



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CNT;
  volatile uint32_t PSG;
  volatile uint32_t ARR;
  volatile uint32_t CNTRL;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t CH1_CNTRL;
  volatile uint32_t CH2_CNTRL;
  volatile uint32_t CH3_CNTRL;
  volatile uint32_t CH4_CNTRL;
  volatile uint32_t CH1_CNTRL1;
  volatile uint32_t CH2_CNTRL1;
  volatile uint32_t CH3_CNTRL1;
  volatile uint32_t CH4_CNTRL1;
  volatile uint32_t CH1_DTG;
  volatile uint32_t CH2_DTG;
  volatile uint32_t CH3_DTG;
  volatile uint32_t CH4_DTG;
  volatile uint32_t BRKETR_CNTRL;
  volatile uint32_t STATUS;
  volatile uint32_t IE;
  volatile uint32_t DMA_RE;
  volatile uint32_t CH1_CNTRL2;
  volatile uint32_t CH2_CNTRL2;
  volatile uint32_t CH3_CNTRL2;
  volatile uint32_t CH4_CNTRL2;
  volatile uint32_t CCR11;
  volatile uint32_t CCR21;
  volatile uint32_t CCR31;
  volatile uint32_t CCR41;
}MDR_TIMER_TypeDef;

   



 



  

 
 
#line 1945 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1954 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1972 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1983 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1998 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2006 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   



  

 
 





 





   



  

 
 
#line 2059 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2069 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2086 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2096 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2113 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2123 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t ADC1_CFG;
  volatile uint32_t ADC2_CFG;
  volatile uint32_t ADC1_H_LEVEL;
  volatile uint32_t ADC2_H_LEVEL;
  volatile uint32_t ADC1_L_LEVEL;
  volatile uint32_t ADC2_L_LEVEL;
  volatile uint32_t ADC1_RESULT;
  volatile uint32_t ADC2_RESULT;
  volatile uint32_t ADC1_STATUS;
  volatile uint32_t ADC2_STATUS;
  volatile uint32_t ADC1_CHSEL;
  volatile uint32_t ADC2_CHSEL;
}MDR_ADC_TypeDef;

   



 



  

 
 
#line 2201 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2220 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2241 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2255 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
 



 



   



  

 
 
 






 






   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
}MDR_DAC_TypeDef;

   



 



  

 
 






 






   



  

 
 



 



   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t RESULT;
  volatile uint32_t RESULT_LATCH;
}MDR_COMP_TypeDef;

   



 



  

 
 
#line 2415 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2427 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t RXTX;
  volatile uint32_t OE;
  volatile uint32_t FUNC;
  volatile uint32_t ANALOG;
  volatile uint32_t PULL;
  volatile uint32_t PD;
  volatile uint32_t PWR;
  volatile uint32_t GFEN;
}MDR_PORT_TypeDef;

   



 



  

 
 
#line 2500 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2518 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 



 



   



  

 
 
#line 2573 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2591 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t REG_00;
  volatile uint32_t REG_01;
  volatile uint32_t REG_02;
  volatile uint32_t REG_03;
  volatile uint32_t REG_04;
  volatile uint32_t REG_05;
  volatile uint32_t REG_06;
  volatile uint32_t REG_07;
  volatile uint32_t REG_08;
  volatile uint32_t REG_09;
  volatile uint32_t REG_0A;
  volatile uint32_t REG_0B;
  volatile uint32_t REG_0C;
  volatile uint32_t REG_0D;
  volatile uint32_t REG_0E;
  volatile uint32_t REG_0F;
  volatile uint32_t RTC_CNT;
  volatile uint32_t RTC_DIV;
  volatile uint32_t RTC_PRL;
  volatile uint32_t RTC_ALRM;
  volatile uint32_t RTC_CS;
}MDR_BKP_TypeDef;

   



 



  

 
 
#line 2651 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2660 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2684 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2701 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2717 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2726 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;



}MDR_EBC_TypeDef;

   



 



  

 
 
#line 2771 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2780 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2799 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2811 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"


   

#line 2838 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   




 

#line 2878 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



 

#line 2913 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   





   

   

   





 
#line 66 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"
#line 73 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"


 

 



 
 






 
#line 97 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"


 


#line 128 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"





 



 
 


 


 
            


 

 


 
 
 
 


 


 
 
 
 
 

 
 

 



 



 


 
 


 







 

 
 








 
#line 218 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"

#line 232 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Config\\MDR32F9Qx_config.h"









 
#line 34 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"
#line 35 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"




 



 



 

#line 68 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"



 

typedef enum
{
  PORT_OE_IN            = 0x0,
  PORT_OE_OUT           = 0x1
}PORT_OE_TypeDef;






 

typedef enum
{
  PORT_MODE_ANALOG      = 0x0,
  PORT_MODE_DIGITAL     = 0x1
}PORT_MODE_TypeDef;






 

typedef enum
{
  PORT_PULL_UP_OFF      = 0x0,
  PORT_PULL_UP_ON       = 0x1
}PORT_PULL_UP_TypeDef;






 

typedef enum
{
  PORT_PULL_DOWN_OFF    = 0x0,
  PORT_PULL_DOWN_ON     = 0x1
}PORT_PULL_DOWN_TypeDef;






 

typedef enum
{
  PORT_PD_SHM_OFF       = 0x0,
  PORT_PD_SHM_ON        = 0x1
}PORT_PD_SHM_TypeDef;







 

typedef enum
{
  PORT_PD_DRIVER        = 0x0,
  PORT_PD_OPEN          = 0x1
}PORT_PD_TypeDef;





 

typedef enum
{
  PORT_GFEN_OFF         = 0x0,
  PORT_GFEN_ON          = 0x1
}PORT_GFEN_TypeDef;






 

typedef enum
{
  PORT_FUNC_PORT        = 0x0,
  PORT_FUNC_MAIN        = 0x1,
  PORT_FUNC_ALTER       = 0x2,
  PORT_FUNC_OVERRID     = 0x3
}PORT_FUNC_TypeDef;






 

typedef enum
{
  PORT_OUTPUT_OFF       = 0x0,
  PORT_SPEED_SLOW       = 0x1,
  PORT_SPEED_FAST       = 0x2,
  PORT_SPEED_MAXFAST    = 0x3
}PORT_SPEED_TypeDef;







 

typedef struct
{
  uint16_t PORT_Pin;                     
 
  PORT_OE_TypeDef PORT_OE;               
 
  PORT_PULL_UP_TypeDef PORT_PULL_UP;     
 
  PORT_PULL_DOWN_TypeDef PORT_PULL_DOWN; 
 
  PORT_PD_SHM_TypeDef PORT_PD_SHM;       
 
  PORT_PD_TypeDef PORT_PD;               
 
  PORT_GFEN_TypeDef PORT_GFEN;           
 
  PORT_FUNC_TypeDef PORT_FUNC;           
 
  PORT_SPEED_TypeDef PORT_SPEED;         
 
  PORT_MODE_TypeDef PORT_MODE;           
 
}PORT_InitTypeDef;




 

typedef enum
{
  Bit_RESET = 0,
  Bit_SET
}BitAction;



   



 



 

#line 249 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"














   



 

#line 286 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"




#line 306 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_port.h"

   

   



 

   



 

void PORT_DeInit(MDR_PORT_TypeDef* PORTx);
void PORT_Init(MDR_PORT_TypeDef* PORTx, const PORT_InitTypeDef* PORT_InitStruct);
void PORT_StructInit(PORT_InitTypeDef* PORT_InitStruct);

uint8_t PORT_ReadInputDataBit(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin);
uint32_t PORT_ReadInputData(MDR_PORT_TypeDef* PORTx);

void PORT_SetBits(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin);
void PORT_ResetBits(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin);
void PORT_WriteBit(MDR_PORT_TypeDef* PORTx, uint32_t PORT_Pin, BitAction BitVal);
void PORT_Write(MDR_PORT_TypeDef* PORTx, uint32_t PortVal);

   

   

   









 
#line 2 "main.c"
#line 3 "main.c"
#line 4 "main.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"





















 

 







 
#line 34 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"
#line 35 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"



 



 



 

typedef enum {BaudRateInvalid = 0, BaudRateValid = !BaudRateInvalid} BaudRateStatus;



 

typedef struct
{
  uint32_t UART_BaudRate;            


 
  uint16_t UART_WordLength;          
 
  uint16_t UART_StopBits;            
 
  uint16_t UART_Parity;              
 
  uint16_t UART_FIFOMode;            
 
  uint16_t UART_HardwareFlowControl; 
 
}UART_InitTypeDef;

   




 



#line 90 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"



 











   



 







   



 













   



 







   



 
#line 162 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"





   



 

#line 184 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"

#line 191 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"

   



 













   



 









   



 







   



 

#line 251 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"









   



 

















   



 

#line 296 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_uart.h"



   

   



 

   



 

void UART_DeInit(MDR_UART_TypeDef* UARTx);
BaudRateStatus UART_Init(MDR_UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct);
void UART_StructInit(UART_InitTypeDef* UART_InitStruct);

void UART_Cmd(MDR_UART_TypeDef* UARTx, FunctionalState NewState);

void UART_ITConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState);
ITStatus UART_GetITStatus(MDR_UART_TypeDef* UARTx, uint32_t UART_IT);
ITStatus UART_GetITStatusMasked(MDR_UART_TypeDef* UARTx, uint32_t UART_IT);
void UART_ClearITPendingBit(MDR_UART_TypeDef* UARTx, uint32_t UART_IT);

void UART_DMAConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IT_RB_LVL, uint32_t UART_IT_TB_LVL);
void UART_DMACmd(MDR_UART_TypeDef* UARTx, uint32_t UART_DMAReq, FunctionalState NewState);

void UART_SendData(MDR_UART_TypeDef* UARTx, uint16_t Data);
uint16_t UART_ReceiveData(MDR_UART_TypeDef* UARTx);
void UART_BreakLine(MDR_UART_TypeDef* UARTx, FunctionalState NewState);

void UART_IrDAConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IrDAMode);
void UART_IrDACmd(MDR_UART_TypeDef* UARTx, FunctionalState NewState);

FlagStatus UART_GetFlagStatus(MDR_UART_TypeDef* UARTx, uint32_t UART_FLAG);
void UART_BRGInit(MDR_UART_TypeDef* UARTx, uint32_t UART_BRG);

   

   

   









 
#line 5 "main.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"





















 

 







 
#line 34 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



 



 



 



 

typedef struct
{
  uint32_t CPU_CLK_Frequency;
  uint32_t USB_CLK_Frequency;
  uint32_t ADC_CLK_Frequency;
  uint32_t RTCHSI_Frequency;
  uint32_t RTCHSE_Frequency;
}RST_CLK_FreqTypeDef;



 

typedef struct {
     uint32_t REG_0F;
} Init_NonVolatile_RST_CLK_TypeDef;

   



 



 



 









   


#line 113 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



 



 









   



 



 











   



 



 

#line 176 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



   




 



 











   



 



 

#line 226 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



   




 



 

#line 249 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 259 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 











   



 



 
#line 296 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 304 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   



 



 

#line 323 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 333 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 
#line 349 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 356 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   



 

 

 
#line 374 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 384 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   




 



 








#line 434 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"







#line 477 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 532 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 

#line 550 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 558 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   






 



 

#line 581 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 591 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 
#line 610 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 620 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   

   



 

   



 

void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);


void RST_CLK_HSEconfig(uint32_t RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);

void RST_CLK_LSEconfig(uint32_t RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint32_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
void RST_CLK_LSIadjust(uint32_t LSItrimValue);
ErrorStatus RST_CLK_LSIstatus(void);

void RST_CLK_CPU_PLLconfig(uint32_t RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul);
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL);
void RST_CLK_CPU_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_CPU_PLLstatus(void);

void RST_CLK_CPUclkPrescaler(uint32_t CPUclkDivValue);
void RST_CLK_CPUclkSelection(uint32_t CPU_CLK);

void RST_CLK_USB_PLLconfig(uint32_t RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul);
void RST_CLK_USB_PLLuse(FunctionalState UsePLL);
void RST_CLK_USB_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_USB_PLLstatus(void);

void RST_CLK_USBclkPrescaler(FunctionalState NewState);
void RST_CLK_USBclkEnable(FunctionalState NewState);

void RST_CLK_ADCclkSelection(uint32_t ADC_CLK);
void RST_CLK_ADCclkPrescaler(uint32_t ADCclkDivValue);
void RST_CLK_ADCclkEnable(FunctionalState NewState);

void RST_CLK_HSIclkPrescaler(uint32_t HSIclkDivValue);
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState);

void RST_CLK_HSEclkPrescaler(uint32_t HSEclkDivValue);
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState);

void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState);
#line 687 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG);






   

   

   









 
#line 6 "main.c"
#line 1 "MDR32F9Qx_it.h"



















 

 



#line 27 "MDR32F9Qx_it.h"
#line 28 "MDR32F9Qx_it.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void CAN1_IRQHandler(void);
void CAN2_IRQHandler(void);
void USB_IRQHandler(void);
void DMA_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void SSP1_IRQHandler(void);
void I2C_IRQHandler(void);
void POWER_IRQHandler(void);
void WWDG_IRQHandler(void);
void Timer1_IRQHandler(void);
void Timer2_IRQHandler(void);
void Timer3_IRQHandler(void);
void ADC_IRQHandler(void);
void COMPARATOR_IRQHandler(void);
void SSP2_IRQHandler(void);
void BACKUP_IRQHandler(void);
void EXT_INT1_IRQHandler(void);
void EXT_INT2_IRQHandler(void);
void EXT_INT3_IRQHandler(void);
void EXT_INT4_IRQHandler(void);



 

 

#line 7 "main.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"





















 

 







 
#line 34 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"



 



 



 



 

typedef struct
{
  uint16_t SSP_SCR;                        


 
  uint16_t SSP_CPSDVSR;                    
 
  uint16_t SSP_Mode;                       
 
  uint16_t SSP_WordLength;                 
 
  uint16_t SSP_SPH;                        
 
  uint16_t SSP_SPO;                        
 
  uint16_t SSP_FRF;                        
 
  uint16_t SSP_HardwareFlowControl;        
 
}SSP_InitTypeDef;

   




 







#line 92 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"











 







   



 

#line 130 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"

#line 144 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"

   



 







   



 







   



 









   



 











   



 













   



 
















 




   



 







   



 

#line 269 "C:\\Keil_v5\\ARM\\PACK\\Keil\\MDR1986BExx\\1.4\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"



   

   



 

   



 

void SSP_DeInit(MDR_SSP_TypeDef* SSPx);
void SSP_Init(MDR_SSP_TypeDef* SSPx, const SSP_InitTypeDef* SSP_InitStruct);
void SSP_StructInit(SSP_InitTypeDef* SSP_InitStruct);
void SSP_Cmd(MDR_SSP_TypeDef* SSPx, FunctionalState NewState);

void SSP_ITConfig(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT, FunctionalState NewState);
ITStatus SSP_GetITStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);
ITStatus SSP_GetITStatusMasked(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);
void SSP_ClearITPendingBit(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);

void SSP_DMACmd(MDR_SSP_TypeDef* SSPx, uint32_t SSP_DMAReq, FunctionalState NewState);

void SSP_SendData(MDR_SSP_TypeDef* SSPx, uint16_t Data);
uint16_t SSP_ReceiveData(MDR_SSP_TypeDef* SSPx);

FlagStatus SSP_GetFlagStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_FLAG);
void SSP_BRGInit(MDR_SSP_TypeDef* SSPx, uint32_t SSP_BRG);

   

   

   









 
#line 8 "main.c"


#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }




    inline float _sqrtf(float __x) { return (float)sqrt(__x); }

    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 803 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );

inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __declspec(__nothrow) __attribute__((const)) double fmax(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __attribute__((const)) double fmin(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );

inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }


extern __declspec(__nothrow) long long llrint(double  );
extern __declspec(__nothrow) long long llrintf(float  );

inline __declspec(__nothrow) long long llrintl(long double __x)     { return llrint((double)__x); }


extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );

inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }


extern __declspec(__nothrow) long long llround(double  );
extern __declspec(__nothrow) long long llroundf(float  );

inline __declspec(__nothrow) long long llroundl(long double __x)     { return llround((double)__x); }


extern __declspec(__nothrow) __attribute__((const)) double nan(const char * );
extern __declspec(__nothrow) __attribute__((const)) float nanf(const char * );

inline __declspec(__nothrow) __attribute__((const)) long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 856 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) __attribute__((const)) double nearbyint(double  );
extern __declspec(__nothrow) __attribute__((const)) float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );

inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }


extern __declspec(__nothrow) __attribute__((const)) double round(double  );
extern __declspec(__nothrow) __attribute__((const)) float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __attribute__((const)) double trunc(double  );
extern __declspec(__nothrow) __attribute__((const)) float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 1034 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1250 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 11 "main.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 985 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 12 "main.c"

#line 1 "spec_1508pl10.h"

typedef struct reg_1508pl10   
{
           char          Name[5];  
  unsigned char       T_Amp_pres:1; 
  unsigned char          T_Fop_o:1; 

  unsigned char             T_vr:1;   
  unsigned char             T_Pd:1;   
  unsigned char          T_Del_m:1;   

  unsigned char               Kz:1; 
  unsigned char               FD:2; 
 
  unsigned char               Kb:1; 
  unsigned char               Kp:1; 

  unsigned char             Kok:1; 
  unsigned char             Kpo:1; 
  unsigned char             Klt:1; 
                                   

  unsigned char            KREF4:1; 
  unsigned char            KREF3:1; 

  unsigned char            KREF2:1; 
  unsigned char            KREF1:1; 
  unsigned char            KREF0:1; 

  unsigned int             Kosn16_19:4; 
  unsigned int             Kosn00_15:16; 


} reg_1508pl10;
#line 14 "main.c"



static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;
static SSP_InitTypeDef sSSP;




uint8_t TxIdx = 0, RxIdx = 0;



 unsigned char DataTr_flag=0;
 unsigned char Data_in_port=0;
 unsigned char DontConnect_flag=0;      
 unsigned      CntDown = 0;             
 unsigned char CyclEnd_flag=0;          
                                        
 unsigned char RcvBufOverflow_flag=0;    
                                        
 unsigned char BufIsRead_flag=0;       
 unsigned char Data4TrAbsent_flag=1;    

 unsigned char TrBuf[256];
 unsigned char RcvBuf[256];
 unsigned char sch_buf=0;

                                         
                                       
 unsigned char *pTr_Buf;                
 unsigned char *pcur_Tr;                
 unsigned char *pRcv_Buf;           
 unsigned char *pcur_Rcv;

 void fillBuf  (         char* ptrBuf, unsigned short lengthBuf, unsigned char symb );
 void clearBuf (         char* ptrBuf, unsigned short lengthBuf );
 void zputs    (              char* s, unsigned char s_size );
  int getStr   (              char* s, unsigned char* s_size );

  

  uint32_t uart1_IT_TX_flag=RESET;
  uint32_t uart1_IT_RX_flag=RESET;

  uint32_t uart2_IT_TX_flag=RESET;
  uint32_t uart2_IT_RX_flag=RESET;




volatile  unsigned int  text_lengh;


unsigned int j_pack=0;
unsigned char packet[4][32];
unsigned char flag_clear=0;
 char ok[5];
unsigned int led_tick;
int *pointer1;
unsigned adspChipVersion;
unsigned nn=0;
unsigned tt;
unsigned tested;
short sch_uart;
unsigned char  sch_plis_ppi=0;
unsigned char  flag_test_sync=0;
unsigned char  sch_packet_UDP_reciv=0;

 char s1[64];
unsigned l1= 5;
 char s2[64];
unsigned l2=6;
 char s3[64];
unsigned l3=7;
 char sr[64];
static  char lsr=0;
unsigned char lk;
unsigned int dFo1;


uint32_t Temp, CurrentLed = 0;
uint32_t i=0;

unsigned short flag_rsv=0;


#line 109 "main.c"



       unsigned char  k;
        char  strng[64];

 volatile unsigned    char lsym;
 volatile unsigned    char  sym;
 volatile unsigned    char flag;
 volatile unsigned    char packet_flag;
 volatile unsigned      char  NB;

  unsigned    char Adress=0x31;       
  unsigned    char Master_flag=0x0;   

  static volatile   unsigned    char packet_sum;
  static volatile   unsigned    char crc,comanda=1;
  static volatile   unsigned    char     InOut[64];
 
  static volatile    char      Word [64];     
  static volatile    char DATA_Word [64];    
  static volatile    char DATA_Word2[64];     
  static volatile unsigned    char crc_ok;
  static volatile unsigned    char packet_ok;
  unsigned    char ink1;  
  unsigned    char data_in;
  static volatile unsigned    char index_word=0;
  static volatile unsigned    char index_data_word =0;
  static volatile unsigned    char index_data_word2=0;
  static volatile unsigned    char data_flag=0;
  static volatile unsigned    char data_flag2=0;
  unsigned    char crc_input; 
  unsigned    char crc_comp;  
  unsigned    char sch_obmen=0;
  unsigned    char   Process_code=0;
  unsigned    char   Test_f_mono=0;

volatile  unsigned int   sch_avariya=0;
  
               
 unsigned volatile int  time_uart;  
 unsigned volatile int  tick_wait;
      
    unsigned  char UDP_TCP_flag=0;
    
                     unsigned char CRC_m[3][6];  
                     unsigned char K615_indik=0;  
              volatile       unsigned char K615_crc_sch=0;   
              volatile       unsigned char K615_crc_sch2=0;  
      
          unsigned  char Error_ethernet_obmen=0;  
 volatile unsigned  char flag_pachka_sinhron;  

 volatile  unsigned  char flag_pachka_TXT; 
 volatile  unsigned  char flag_pachka_TXT2; 

 volatile  unsigned  int sch_pachek_test=0;


  unsigned volatile char          flag_K615_event; 

 volatile   unsigned char flag_cikl_ON=0;
 volatile   unsigned char flag_Ethernet_packet_rcv=0;
 volatile            int sch_UDP_pakets=0;

        unsigned char flag_process=1;
        unsigned char flag_event_K615_run=0;
        unsigned int  delay_process=400;

        unsigned char flag_uart_trcv=0;

        unsigned char Pachka_START=0;
        unsigned char   sync_flag=0;

      volatile  unsigned char   sync_sch_flag=3;
      unsigned volatile char tick_us;

    unsigned char sinc_type=0;

   volatile unsigned char  flag_MASTER_pachka=0;
   volatile unsigned int   TNC_number_run=0;
            unsigned char  flag_form_packet_SDRAM=0;


 
 
 unsigned volatile  char index1  =0;
 unsigned volatile char lsym1    =0;
 unsigned volatile char pack_ok1 =0;
 unsigned volatile char pack_sum1=0;   
 unsigned volatile char sym1     =0;  
  
 volatile unsigned char               Flag_K611=0;
 volatile unsigned char          Flag_init_K611=0;
 volatile unsigned char       Flag_control_K611=0;
 volatile unsigned char   Flag_control_sig_K611=0;
 volatile unsigned char   Flag_control_end_K611=0;
 volatile unsigned char    Flag_zahvat_sig_K611=0;
 volatile  unsigned char   Flag_zahvat_end_K611=0;
 volatile unsigned char              Qwant_K611=0;
 volatile unsigned char     Flag_zahvat_OK_K611=0;
 volatile unsigned char     Flag_signal_OK_K611=0;


 volatile unsigned char                  PROCESS=0;

  unsigned char   test1=0;
  unsigned char   test2=0;

 volatile unsigned char sys_life_k612=0;
 volatile unsigned char sys_life_k613=0;

volatile  unsigned char AVARIYA_flag=0;

 unsigned volatile int  tick_process;
 unsigned volatile int  tick_process_K611;
 unsigned volatile int  tick_process_K615;
 unsigned volatile int  tick_process_K612;
 unsigned volatile int  tick_process_K613;
 unsigned volatile int  tick_process_OK;
 unsigned volatile int  tick_TCP;
 unsigned volatile int  tick_UDP;

  
 unsigned volatile char flag_Ethernet;  
       unsigned volatile char flag_Ethernet_Terminal=0; 

     
   volatile    unsigned char flag_PPI_sz1 =0;
   volatile    unsigned char flag_PPI_sz2 =0;
   volatile    unsigned char flag_PPI_sinc=0;
   volatile    unsigned char flag_contr_TNC_TNO=0;
   volatile    unsigned char flag_Packet_form=0;

   unsigned char flag_PPI_START=0;


       unsigned char label_PPI=0;
       unsigned char Test_PPI_flag1=0;
       unsigned char Test_PPI_flag2=0;
       unsigned char Test_PPI_flag3=0;

       unsigned char RESET_SINTEZ_flag=0;

   volatile    unsigned int time_TNO=0;
   volatile    unsigned int time_TNC=0;

   volatile    unsigned int time_TNO_min=0;
   volatile    unsigned int time_TNC_min=0;

   volatile    unsigned int time_TNO_max=0;
   volatile    unsigned int time_TNC_max=0;

   volatile    unsigned char flag_1HZ_sync=0;
               unsigned char flag_START_packa_SINTEZ=0;
  

 



reg_1508pl10 reg_FAPCH1={ 
              {'Ф','А','П','Ч','1'},  
              0,
							0,
							0,
							0,
							0,
							0,
						  0x0,
						    0,
						    0,
						    0,
						    0,
						    1,
						    0,
						    0,
						    0,
						    0,
						    0,
						    0,
						  240 

                          }; 

 reg_1508pl10 reg_FAPCH2={ 
              {'Ф','А','П','Ч','2'},     
              0,
							0,
							0,
							0,
							0,
							0,
						  0x0,
						    0,
						    0,
						    0,
						    0,
						    1,
						    0,
						    0,
						    0,
						    0,
						    0,
						    0,
						  240 

                          };    


void Uart1PinCfg(void)
{
   
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
     
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = 0x0040U;
    PORT_Init(((MDR_PORT_TypeDef *) (0x400B0000)), &PortInit);
     
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = 0x0020U;
    PORT_Init(((MDR_PORT_TypeDef *) (0x400B0000)), &PortInit);  
}

void Uart2PinCfg(void)
{
	
	   
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
     
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = 0x0001U;
    PORT_Init(((MDR_PORT_TypeDef *) (0x400C0000)), &PortInit);
     
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = 0x0002U;
    PORT_Init(((MDR_PORT_TypeDef *) (0x400C0000)), &PortInit); 


}
void MltPinCfg (void)
{


   
    PortInit.PORT_PULL_UP   = PORT_PULL_UP_ON;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM    = PORT_PD_SHM_OFF;
    PortInit.PORT_PD        = PORT_PD_DRIVER;
    PortInit.PORT_GFEN      = PORT_GFEN_OFF;
   
  PortInit.PORT_Pin   = (0x0004U |  0x0020U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400A8000)), &PortInit);

  PortInit.PORT_Pin   = (0x0008U);
  PortInit.PORT_OE    = PORT_OE_IN;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400A8000)), &PortInit);

   
  PortInit.PORT_Pin   = (0x0200U | 0x0400U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400B0000)), &PortInit);

  PortInit.PORT_Pin   = (0x0080U );
  PortInit.PORT_OE    = PORT_OE_IN;
    PORT_Init(((MDR_PORT_TypeDef *) (0x400B0000)), &PortInit);

   
  PortInit.PORT_Pin   = (0x0001U | 0x0004U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400B8000)), &PortInit);

     
  PortInit.PORT_Pin   = (0x0040U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400C8000)), &PortInit);

  PortInit.PORT_Pin   = (0x0001U|0x0002U|0x0004U|0x0008U|0x0080U);
  PortInit.PORT_OE    = PORT_OE_IN;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400C8000)), &PortInit);

}

void LedPinGfg (void)
{
   
  PortInit.PORT_Pin   = (0x0040U | 0x0080U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(((MDR_PORT_TypeDef *) (0x400A8000)), &PortInit);

     
  PortInit.PORT_Pin   = (0x0100U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(((MDR_PORT_TypeDef *) (0x400B0000)), &PortInit);

       
  PortInit.PORT_Pin   = (0x0002U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(((MDR_PORT_TypeDef *) (0x400B8000)), &PortInit);
}


void Uart2Setup(void)
{
	unsigned int temp;
 
    UART_InitStructure.UART_BaudRate                = 115200;
    UART_InitStructure.UART_WordLength              = ((uint16_t)0x0060);
    UART_InitStructure.UART_StopBits                = ((uint16_t)0x0000);
    UART_InitStructure.UART_Parity                  = ((uint16_t)0x0000);
    UART_InitStructure.UART_FIFOMode                = ((uint16_t)0x0000);
    UART_InitStructure.UART_HardwareFlowControl     = ((uint16_t)0x0200) | ((uint16_t)0x0100);

     
    UART_Init (((MDR_UART_TypeDef *) (0x40038000)),&UART_InitStructure);
       
 
     
    UART_Cmd(((MDR_UART_TypeDef *) (0x40038000)),ENABLE); 

       
 

    
    UART_ITConfig (((MDR_UART_TypeDef *) (0x40038000)), ((uint32_t)0x00000010), ENABLE);

}

void Uart1Setup(void)
{
	unsigned int temp;
 
    UART_InitStructure.UART_BaudRate                = 115200;
    UART_InitStructure.UART_WordLength              = ((uint16_t)0x0060);
    UART_InitStructure.UART_StopBits                = ((uint16_t)0x0000);
    UART_InitStructure.UART_Parity                  = ((uint16_t)0x0000);
    UART_InitStructure.UART_FIFOMode                = ((uint16_t)0x0000);
    UART_InitStructure.UART_HardwareFlowControl     = ((uint16_t)0x0200) | ((uint16_t)0x0100);

     
    UART_Init (((MDR_UART_TypeDef *) (0x40030000)),&UART_InitStructure);
       
 
     
    UART_Cmd(((MDR_UART_TypeDef *) (0x40030000)),ENABLE); 

          
 

    
  UART_ITConfig (((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000010), ENABLE);

}

void SPI_init(void)

{

	 


    
  PortInit.PORT_Pin   = (0x0008U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400C0000)), &PortInit);	

   
  PortInit.PORT_Pin   = (0x0040U |  0x0020U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400C0000)), &PortInit);

   

   
  PortInit.PORT_Pin   = (0x0001U | 0x0002U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400E8000)), &PortInit);

     

  PortInit.PORT_Pin   = (0x0004U);
  PortInit.PORT_OE    = PORT_OE_OUT;
  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInit.PORT_SPEED = PORT_SPEED_FAST;
  PortInit.PORT_PD    = PORT_PD_DRIVER;
  PORT_Init(((MDR_PORT_TypeDef *) (0x400E8000)), &PortInit);	

  SSP_BRGInit(((MDR_SSP_TypeDef *) (0x40040000)),((uint32_t)0x00000004));
  SSP_BRGInit(((MDR_SSP_TypeDef *) (0x400A0000)),((uint32_t)0x00000004));

   
  SSP_StructInit (&sSSP);

  sSSP.SSP_SCR  = 0xff; 


 
  sSSP.SSP_CPSDVSR = 2;
 
  sSSP.SSP_Mode = ((uint32_t)0x0000);
  sSSP.SSP_WordLength = ((uint16_t)0x000F);
  sSSP.SSP_SPH = ((uint16_t)0x0080);
  sSSP.SSP_SPO = ((uint16_t)0x0040);
  sSSP.SSP_FRF = ((uint16_t)0x0000);
  sSSP.SSP_HardwareFlowControl = ((uint16_t)0x0000);
  SSP_Init (((MDR_SSP_TypeDef *) (0x40040000)),&sSSP);

   
  
  sSSP.SSP_SCR  = 0xff;
  sSSP.SSP_CPSDVSR = 2;
  sSSP.SSP_Mode = ((uint32_t)0x0000);
  sSSP.SSP_WordLength = ((uint16_t)0x000F);
  sSSP.SSP_SPH = ((uint16_t)0x0080);
  sSSP.SSP_SPO = ((uint16_t)0x0040);
  sSSP.SSP_FRF = ((uint16_t)0x0000);
  sSSP.SSP_HardwareFlowControl = ((uint16_t)0x0000);
  SSP_Init (((MDR_SSP_TypeDef *) (0x400A0000)),&sSSP);

   
  SSP_Cmd(((MDR_SSP_TypeDef *) (0x40040000)), ENABLE);
   
  SSP_Cmd(((MDR_SSP_TypeDef *) (0x400A0000)), ENABLE);

}



void delay(int inc)
{

   inc <<= 3;
   while(inc !=0) inc--;

}



void CS_SPI1 (char a)
{
   while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x40040000)), ((uint16_t)0x0010)) == SET) {};

  if (a==1)
  {
    delay(300);
    PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400E8000)), 0x0004U);
        delay(300);
  }

  if (a==1)
  {
    delay(300);
    PORT_ResetBits (((MDR_PORT_TypeDef *) (0x400E8000)), 0x0004U);
        delay(300);
  }

}     

void CS_SPI2 (char a)
{

  while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x400A0000)), ((uint16_t)0x0010)) == SET) {};

  if (a==1)
  {
    delay(300);
    PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400C0000)), 0x0008U);
        delay(300);
  }

  if (a==1)
  {
    delay(300);
    PORT_ResetBits (((MDR_PORT_TypeDef *) (0x400C0000)), 0x0008U);
        delay(300);
  }

}                 

void SPI1_send(unsigned short a)
{
	 
    while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x40040000)), ((uint16_t)0x0001)) == RESET)
    {
    }
     
    SSP_SendData(((MDR_SSP_TypeDef *) (0x40040000)),a);
}

void SPI2_send(unsigned short a)
{
	 
    while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x400A0000)), ((uint16_t)0x0001)) == RESET)
    {
    }
     
    SSP_SendData(((MDR_SSP_TypeDef *) (0x400A0000)),a);
}

unsigned short SPI1_read(void)
{
	unsigned short a;
	 
    while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x40040000)), ((uint16_t)0x0004)) == RESET)
    {
    }
     
    a = SSP_ReceiveData(((MDR_SSP_TypeDef *) (0x40040000)));
    return a;
}

unsigned short SPI2_read(void)
{
	unsigned short a;
	 
    while (SSP_GetFlagStatus(((MDR_SSP_TypeDef *) (0x400A0000)), ((uint16_t)0x0004)) == RESET)
    {
    }
     
    a = SSP_ReceiveData(((MDR_SSP_TypeDef *) (0x400A0000)));
    return a;
}




void RegisterInits () 
{ 



 

   
 
   
 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400D8000)) >> 15) & 0x1F))),ENABLE);


   
    
     RST_CLK_HSEconfig(((uint32_t)0x00000001));

     while (RST_CLK_HSEstatus() != SUCCESS)  {}; 

     
     
     RST_CLK_CPU_PLLconfig(((uint32_t)0x00000002), 5);

      
     RST_CLK_CPU_PLLcmd(ENABLE);


     while (RST_CLK_HSEstatus() != SUCCESS)    {}; 

       
        RST_CLK_CPUclkPrescaler(((uint32_t)0x00000008));
        
        RST_CLK_CPU_PLLuse(ENABLE);
        
        RST_CLK_CPUclkSelection(((uint32_t)0x00000100));

     RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x40020000)) >> 15) & 0x1F))),ENABLE);
	 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400A8000)) >> 15) & 0x1F))),ENABLE);
	 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400B0000)) >> 15) & 0x1F))),ENABLE);
	 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400B8000)) >> 15) & 0x1F))),ENABLE);
	 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400C0000)) >> 15) & 0x1F))),ENABLE); 
	 RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400C8000)) >> 15) & 0x1F))),ENABLE);
     RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400E8000)) >> 15) & 0x1F))),ENABLE);


	RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x40040000)) >> 15) & 0x1F))),ENABLE);
	RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x400A0000)) >> 15) & 0x1F))),ENABLE);

  
     RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x40030000)) >> 15) & 0x1F))), ENABLE);
     RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)(0x40038000)) >> 15) & 0x1F))), ENABLE);


    UART_BRGInit(((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000000));
    UART_BRGInit(((MDR_UART_TypeDef *) (0x40038000)), ((uint32_t)0x00000000));

  NVIC_EnableIRQ(UART1_IRQn);
  NVIC_EnableIRQ(UART2_IRQn);


}



void RX_485 (void)
{

  while (UART_GetFlagStatus (((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000008))== SET)   {};

    PORT_ResetBits  (((MDR_PORT_TypeDef *) (0x400A8000)), 0x0004U); 
     
    PORT_ResetBits  (((MDR_PORT_TypeDef *) (0x400B8000)), 0x0001U);  

}

void TX_485 (void)
{
	while (UART_GetFlagStatus (((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000008))== SET)   {};
	
    PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400A8000)), 0x0004U); 

    PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400B8000)), 0x0001U);  
	
}


void zputc ( uint8_t c) 
{
  uint8_t DataByte;

  DataByte=c;

     
    while (UART_GetFlagStatus (((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000080))!= SET)   {};
    
    UART_SendData (((MDR_UART_TypeDef *) (0x40030000)),DataByte);

}


void zputc2 ( uint8_t c) 
{
  uint8_t DataByte;

  DataByte=c;

     
    while (UART_GetFlagStatus (((MDR_UART_TypeDef *) (0x40038000)), ((uint32_t)0x00000080))!= SET)   {};
    
    UART_SendData (((MDR_UART_TypeDef *) (0x40038000)),DataByte);

}


 void zputs( char s[], unsigned char l)
{
      unsigned char i;
      for (i=0;i<l;i++) {zputc ((s[i]));s[i]=0x0;}
}

 void zputs2( char s[], unsigned char l)
{
      unsigned char i;
      for (i=0;i<l;i++) {zputc2 ((s[i]));s[i]=0x0;}
}

unsigned int leng ( char s[])

  {
    unsigned  char i=0;
     while ((s[i]!='\0')&&(i<120)) { i++;}
    return i;
  }

void sendT ( char  s[])

 { 
 	   TX_485();
 	   zputs(s,leng(s));
 	  
    

 }

void sendT2 ( char s[])

 {
       zputs2(s,leng(s));

 }




  
 void reverse( char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }


 void itoa(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  
         n = -n;         
     i = 0;
     do {       
         s[i++] = n % 10 + '0';   
     } while ((n /= 10) > 0);    
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }




 void Transf( char s[]) 
   {
    
   
        sendT(s); 
        sendT2(s);   
   
  }
   

 void ZTransf( char* s,unsigned char a) 
   {

      zputs (s,a);
      zputs2(s,a);
      
  }
      


void fillBuf ( char* ptrBuf, unsigned short lengthBuf, unsigned char symb )
   { unsigned short i;

     for ( i=0; i < lengthBuf; i++ )
      {
        *ptrBuf=symb; ptrBuf++;
      }
   }

  void clearBuf ( char* ptrBuf, unsigned short lengthBuf )
   {  fillBuf( ptrBuf, lengthBuf, ' ' );
   }
      

   int getStr (  char* s, unsigned char*ps_size )
    { unsigned char* temp;
      temp=pRcv_Buf;
      *ps_size=0;
      if ( pcur_Rcv <  temp ){
        
   
        
        *ps_size=1;
        while(pcur_Rcv <  temp)
          {*s=*pcur_Rcv; s++; pcur_Rcv++;}
         
        }
       else if  ( temp < pcur_Rcv )
              {  
              
                if  ( CyclEnd_flag )
                    { while ( pcur_Rcv != temp )
                        {*s = *pcur_Rcv; s++;
                         *ps_size++;
                         if ( pcur_Rcv == (RcvBuf+ 256-1) )
                           { pcur_Rcv= RcvBuf;
                             CyclEnd_flag=0;  
                           }
                          else
                            pcur_Rcv++;
                        }
                    }
                  else
                   {; 
                    return 1;
                   }
              }
  
       else  
                            return 1;       
   
      return 0;
    }






void Menu()

{

}


void Menu1()
 
 {


    int i;
	
 
    for (i=0; i< 3; i++) Transf("\r\n");    
	for (i=0; i<15; i++) Transf ("*");  
	
	Transf("\r\n");
	Transf("\r\n");
	Transf("\r\n");
	Transf("\r\n");
	Transf("......Terminal ФОЧ-1 Комплекция-12....\r\n");
	Transf("\r\n");
	Transf("\r\n");
	Transf("\r\n");
	Transf("MENU :\r\n");
	Transf("-------\r\n");
	Transf("~ - стартовый байт\r\n");
	Transf("1 - адрес абонента\r\n");
	Transf("start_TCP -  команда\r\n");
	Transf("=30000000 - данные команды\r\n");
	Transf(";- конец пачки \r\n");
	Transf(".............. \r\n");
	Transf("Адреса: \r\n");
	Transf("1 - 1У-К6xx   , синхронизатор мастер: \r\n");
	Transf("2 - 1У-К6xx-1 , синхронизатор резерв\r\n");
	Transf("3 - 1У-К6xx , синтезатор частоты излучения  \r\n");
	Transf("4 - 1У-К6xx , синтезатор частоты гетеродина  \r\n");
	Transf("6 - 1У-К6xx , ФОЧ  \r\n");
	Transf("+++++++++++++++++++\r\n");
	Transf("~1 watch_reg_ФАПЧ1;\r\n");
  Transf("~1 watch_reg_ФАПЧ2;\r\n");
	








 
  
  Transf("~1 ФАПЧ1_Kdiv1();\r\n");
  Transf("~1 ФАПЧ2_Kdiv1();\r\n");

  Transf("~1 ФАПЧ1_Kdiv2();\r\n");
  Transf("~1 ФАПЧ2_Kdiv2();\r\n");

	Transf("~1 help;\r\n");
	Transf("~1 SPI_send():0x55;\r\n");
	
	








 
	
	Transf("~1 ФАПЧ1_init;\r\n");
	Transf("~1 ФАПЧ2_init;\r\n");
	Transf("~1 Включить_синтезатор;\r\n");

	Transf("~1 Индикация_состояния_синтезатора;\r\n");
	


 
	Transf("~1 Установить_Амплитуду=0;\r\n");
	






 
	











 
 








 
  
    Transf("~1 ADC=1;  проверка ADC микроконтроллера\r\n");
  






 
	Transf("\r\n");
	Transf("\r\n");
	Transf("++++++++++++++++++++\r\n");
	Transf("\r\n");
	Transf("\r\n");
	
	
	sendT("\r\n");
	


	
	}

void reg_FAPCH_watch (reg_1508pl10 a)
{
  Transf ("Индикация структуры бит ФАПЧ:");

  if (a.Name[4]=='1') Transf ("ФАПЧ1\r\n");
  if (a.Name[4]=='2') Transf ("ФАПЧ2\r\n");
  Transf("\r\n");

}

void Kdiv1(unsigned int dFo1,reg_1508pl10 a)
{

   Transf("\r\n");
   Transf ("..установка кода Kdiv1:");
   sprintf(strng,"%d",dFo1);
   Transf(strng);
   Transf("\r\n");

   if (dFo1==999)  Transf("принято 999\r\n");

       if (dFo1==10)  {
                        a.KREF4=0;
                        a.KREF3=0;
                      } else
                                
       if (dFo1==80)  {
                        a.KREF4=0;
                        a.KREF3=1;
                      } else
       if (dFo1==100) {
                        a.KREF4=1;
                        a.KREF3=0;
                      } else
       if (dFo1==125) {
                        a.KREF4=1;
                        a.KREF3=1;
                      } else Transf ("..Kdiv1 - НЕТ ТАКОГО КОДА!!!...\r\n");
                                           
 Transf("\r\n");

}

void Kdiv2(unsigned int dFo1,reg_1508pl10 a)
{
   Transf ("..установка кода Kdiv2:");
   sprintf(strng,"%d",dFo1);
   Transf(strng);
   Transf("\r\n");


       if (dFo1==160)  {
                        a.KREF2=0;
                        a.KREF1=0;
                        a.KREF0=0;
                                           } else
                                
       if (dFo1== 80)  {
                        a.KREF2=0;
                        a.KREF1=0;
                        a.KREF0=1;
                                           } else
       if (dFo1== 40) {
                        a.KREF2=0;
                        a.KREF1=1;
                        a.KREF0=0;
                                           } else
       if (dFo1== 20) {
                        a.KREF2=0;
                        a.KREF1=1;
                        a.KREF0=1;
                                           } else
       if (dFo1==200) {
                        a.KREF2=1;
                        a.KREF1=0;
                        a.KREF0=0;
                                           } else

       if (dFo1==100) {
                        a.KREF2=1;
                        a.KREF1=0;
                        a.KREF0=1;
                                           } else
       if (dFo1==50) {
                        a.KREF2=1;
                        a.KREF1=1;
                        a.KREF0=0;
                                           } else
       if (dFo1==10) {
                        a.KREF2=1;
                        a.KREF1=1;
                        a.KREF0=1;
                                           } else Transf ("..Kdiv2 - НЕТ ТАКОГО КОДА!!!...\r\n");

         Kdiv1(10,a); 



}





void IO ( char* sr)      
 
 {
       
 unsigned char s[6];   
 unsigned char i=0;
 unsigned char index=0;
 unsigned char dlsr=0;
  char z;
  char p[1];
  unsigned char  z_k611=0;
  char sys_char=0;
  int l=0;

  unsigned short PF1_var=0;
  unsigned short PF2_var=0;
  unsigned short PF3_var=0;
  unsigned short PF4_var=0;
  unsigned int Z_sum=0;
  unsigned short temp_short=0;
  
  
   i=leng (sr);
     
   sym1=sr[0];
   
   p[0]=sym1;
      
if ((sym1==0x7e)||(time_uart>100)) 
	{
		time_uart=0;  
		packet_flag=1; 
		index1=0; 
		crc=0; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0; 
		index_data_word =1;
    index_data_word2=1;
		data_flag =0;
    data_flag2=0;
		DATA_Word [0]=' ';
    DATA_Word2[0]=' ';
		
	} 

if (packet_flag==1)

{ 

	while (i>0)   
	
	{


	   InOut[index1]=sr[index];
	   sr[index]=0x0;
	  
	  if  (InOut[index1]==' ')  ink1=ink1+1;
	  
	  if  (InOut[index1]==';')  packet_ok=1;
	  
	  if ((InOut[index1]=='=')||(InOut[index1]==':')) data_flag=1;

    if  (InOut[index1]=='.') data_flag2=1;


        	  if ((index1>2)&&(InOut[2]==' '))  
        		{
        		
                		         if  ((InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (data_flag!=1))	       Word[index_word]=InOut[index1]; 
                		
                   
                        	   if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==0))    	DATA_Word[index_data_word]=InOut[index1]; 
                      
                   
                             if  ((data_flag==1)&&
                                 (InOut[index1]!=' ')&&
                                 (InOut[index1]!=';')&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':')&&(data_flag2==1))     DATA_Word2[index_data_word2]=InOut[index1]; 
                		      
                	   
                             if ((data_flag!=1)&&
                                 (InOut[index1]!='=')&&
                                 (InOut[index1]!=':'))      index_word=index_word+1; 
                            		   else 
                                            {
                                             
                                             if ((data_flag==1)&&
                                                 (InOut[index1]!='=')&&
                                                 (InOut[index1]!=':')&&
                                                 (InOut[index1]!='.')&&
                                                 (data_flag2==0))      index_data_word=index_data_word+1;
                                            
                                             if ((data_flag==1)&&
                                                 (InOut[index1]!='=')&&
                                                 (InOut[index1]!=':')&&
                                                 (InOut[index1]!='.')&&
                                                 (data_flag2==1))      index_data_word2=index_data_word2+1;
                                            }
        	  	           
        		}
				
        		index1=index1+1;
        		index =index +1;
        		i=i-1;
	
	   }
	
	i=index1;

	
	
if (packet_ok==1) 
	{

	
	    if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   
	  	if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   
		
	if (crc_ok==0x3)  
    	{


                    					if (strcmp(Word,"help")==0)        				      

                    			          {
                    			             Transf ("принял help\r\n"    ); 
                    			              Menu1('t');
                    			             flag_uart_trcv=0;
                    			             sch_pachek_test=1;
                    			           }

   
                              if (strcmp(Word,"watch_reg_ФАПЧ1"  )==0)       { Transf ("принял watch_reg_ФАПЧ1\r\n"  ); reg_FAPCH_watch (reg_FAPCH1); }
					 					          if (strcmp(Word,"watch_reg_ФАПЧ2"  )==0)       { Transf ("принял watch_reg_ФАПЧ2\r\n"  ); reg_FAPCH_watch (reg_FAPCH2); }

                  					  if (strcmp(Word,"ADC"      )==0) 	    
                                    				{ 	dFo1=atoi(DATA_Word);
                  								             	Transf ("принял ADC:" );
                                    					  sprintf(strng,"%x",dFo1);
                                                
                                    				   	Transf(strng);
                                    					  Transf("\r\n");
                  									             
                  									
                                    					 }
	
                            if (strcmp(Word,"Установить_Амплитуду"      )==0) 	    
                        				{ 	dFo1=atoi(DATA_Word);
      									            Transf ("принял Установить_Амплитуду:" );

                        				    sprintf(strng,"%x",dFo1);
                                    
                                    Transf(strng);
                                    Transf("\r\n");
                                    sendT(strng);
                                    Transf("\r\n");
                                    sendT2(strng);
                        					  Transf("\r\n");
      									
                        					 }   

                        		if (strcmp(Word,"SPI_send()"      )==0) 	    
                        				{ 	dFo1=atoi(DATA_Word);
      								            	Transf ("принял SPI_send():" );
                        				    sprintf(strng,"%x",dFo1);
                        					  Transf(strng);
                        					  Transf("\r\n");

                        					SPI1_send(dFo1);
                        					SPI2_send(dFo1);
      									
                        					 }   

                             if (strcmp(Word,"ФАПЧ1_Kdiv1()"      )==0)       
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ1_Kdiv1():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r\n");

                                      Kdiv1(dFo1,reg_FAPCH1);
                                      Write_FOCH1 (reg_FAPCH1);
                          
                                     } 
                             if (strcmp(Word,"ФАПЧ2_Kdiv1()"      )==0)       
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ2_Kdiv1():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r\n");

                                      Kdiv1(dFo1,reg_FAPCH2);
                                      Write_FOCH2 (reg_FAPCH2);
                          
                                     } 
                          if (strcmp(Word,"ФАПЧ1_Kdiv2()"      )==0)       
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ1_Kdiv2():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r\n");

                                      Kdiv2(dFo1,reg_FAPCH1);
                                      Write_FOCH1 (reg_FAPCH1);
                          
                                     } 
                        if (strcmp(Word,"ФАПЧ2_Kdiv2()"      )==0)       
                                  {   dFo1=atoi(DATA_Word);
                                      Transf ("принял ФАПЧ2_Kdiv2():" );
                                      sprintf(strng,"%d",dFo1);
                                      Transf(strng);
                                      Transf("\r\n");

                                      Kdiv2(dFo1,reg_FAPCH2);
                                      Write_FOCH2 (reg_FAPCH2);
                          
                                     } 

                  	    if (strcmp(Word,"ФАПЧ1_init")==0)      { Transf ("принял ФАПЧ1_init\r\n"   ); Write_FOCH1 (reg_FAPCH1);    };	
                  	    if (strcmp(Word,"ФАПЧ2_init")==0)      { Transf ("принял ФАПЧ2_init\r\n"   ); Write_FOCH2 (reg_FAPCH2);    };	

                 	      if (strcmp(Word,"Индикация_состояния_синтезатора")==0)      { Transf ("принял Индикация_состояния_синтезатора\r\n"   );     }		   
                        if (strcmp(Word,"Включить_синтезатор"		     )==0)      { Transf ("принял Включить_синтезатор\r\n"        		 );     }
             
						
                        if (strcmp(Word,"сигналы_захвата_1У-К611" )==0) 	   
                  				{ 
                                  crc_comp =atoi(DATA_Word);
                                  crc_input=atoi(DATA_Word2);  

                             if (crc_comp==crc_input)
                                 {
                                        z_k611=atoi(DATA_Word);

                                   
                                  
                                  

                                        
                                         Flag_zahvat_end_K611=1;
                                         tick_process=0;
                                        
                                         sch_obmen=0;

                                 }

                          
                  					}	
                  					
                  		if (strcmp(Word,"значение_уровней_1У-К611" )==0) 	   
                  				{ 

                                  crc_comp =atoi(DATA_Word);
                                  crc_input=atoi(DATA_Word2); 

                             if (crc_comp==crc_input)
                                 {
                                       z_k611=atoi(DATA_Word);
                            			
                            			
                                 
                                      
                            				   Flag_control_end_K611=1;
                            				   tick_process=0;
                            				  
                            				   sch_obmen=0;
                                  }
                  					}

                            
                  	
                  		if (strcmp(Word,"init_K611")==0) 
                  		{ 
                  	
                  		Flag_init_K611=1;
                  		Flag_K611=0;
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} 
                  		
                  		if (strcmp(Word,"состояние_1У-К611")==0)
                  		{ 
                  
                  		Flag_control_K611=1;
                  		
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  				} 
                  		
                  		if (strcmp(Word,"сигналы_контроля_1У-К611")==0)
                  		{ 
                  	
                  		Flag_zahvat_sig_K611=1;
                  		
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} 
                  		
                  		if (strcmp(Word,"контроль_уровней_1У-К611")==0)
                  		{ 
                  	
                  		Flag_control_sig_K611=1;
                  		
                  		tick_process=0;
                  		sch_obmen=0;
                  	    tick_process_K611=0;
                  		} 
                  		
                  
              
                      
		
	    } 
	
	}
	
	if ((packet_ok==1)&&(crc_ok==0x1))     

	{
		
	  if (Master_flag==0)

      {
    
        	Transf("\r\n");
        	ZTransf (InOut,index1);
       
       
          
      }

		
	}
	
	
	if ( packet_ok==1) 
		
		{
		
		
		for (i=0;i<64;i++)        Word[i]     =0x0;
		for (i=0;i<64;i++)   DATA_Word[i]     =0x0;
    for (i=0;i<64;i++)  DATA_Word2[i]     =0x0;  
	  for (i=0;i<64;i++)       InOut[i]     =0x0;
		
		
	
	
	    time_uart=0;  
		packet_flag=0; 
		index1=0; 
		crc=0; 
		crc_ok=0; 
		i=0;
		packet_ok=0; 
		index_word=0; 
		index_data_word=0;
		data_flag=0;
        index_data_word2=0;
        data_flag2=0;
		
			
         
			

		};

	}
   
   
   
        
 } 



void UART1_IRQHandler(void)
{ 
  unsigned stat;
    unsigned t;
  uint32_t value;
  uint8_t ReciveByte;
  unsigned char letter[1];

if (UART_GetITStatusMasked(((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000010)) == SET)
  {
    UART_ClearITPendingBit(((MDR_UART_TypeDef *) (0x40030000)), ((uint32_t)0x00000010));
    uart1_IT_RX_flag = SET;
  }

Transf ("+");

  if (uart1_IT_RX_flag == SET)
  {

     
       uart1_IT_RX_flag = RESET;

      
    ReciveByte = UART_ReceiveData (((MDR_UART_TypeDef *) (0x40030000)));

  letter[0]=ReciveByte;
  
  BufIsRead_flag=1; 
  
                      
            { 
                                                      
             t= letter[0];
             if (  1 ) 
               {
                 *pRcv_Buf=t;  
                 if ( pRcv_Buf == pcur_Rcv )         
                   { if ( CyclEnd_flag )
                       { 
                        RcvBufOverflow_flag=1;
                        CyclEnd_flag = 0;
                       }
                      
                     BufIsRead_flag=1;      
                   }
                 if ( pRcv_Buf == (RcvBuf+256-1))
                   {
                    pRcv_Buf = RcvBuf;
                    CyclEnd_flag = 0x1;
                   }
                  else
                   { 
                    pRcv_Buf++;
                   }
                  
               }
            }
  
  }


}










 
void UART2_IRQHandler(void)
{
  unsigned stat;
    unsigned t;
  uint32_t value;
  uint8_t ReciveByte;
  unsigned char letter[1];

if (UART_GetITStatusMasked(((MDR_UART_TypeDef *) (0x40038000)), ((uint32_t)0x00000010)) == SET)
  {
    UART_ClearITPendingBit(((MDR_UART_TypeDef *) (0x40038000)), ((uint32_t)0x00000010));
    uart2_IT_RX_flag = SET;
  }

  if (uart2_IT_RX_flag == SET)
  {

     
       uart2_IT_RX_flag = RESET;

      
    ReciveByte = UART_ReceiveData (((MDR_UART_TypeDef *) (0x40038000)));
  

  
  letter[0]=ReciveByte;

  BufIsRead_flag=1;   
  
                      
            { 
                                                      
             t= letter[0];
             if (  1 ) 
               {
                 *pRcv_Buf=t;  
                 if ( pRcv_Buf == pcur_Rcv )         
                   { if ( CyclEnd_flag )
                       { 
                        RcvBufOverflow_flag=1;
                        CyclEnd_flag = 0;
                       }
                      
                     BufIsRead_flag=1;      
                   }
                 if ( pRcv_Buf == (RcvBuf+256-1))
                   {
                    pRcv_Buf = RcvBuf;
                    CyclEnd_flag = 0x1;
                   }
                  else
                   { 
                    pRcv_Buf++;
                   }
                  
               }
            }
       }  
}









void UART_control (void)

{
     if ((BufIsRead_flag==1)) 
           
            {            	
               getStr(sr,&lsr);
               BufIsRead_flag=0;
               IO(sr);
                    
            } 
}







 


int main()
{
	int i=0;

    RegisterInits();

    MltPinCfg ();
    LedPinGfg ();

    Uart1PinCfg();
    Uart1Setup();
    

    Uart2PinCfg();
    Uart2Setup();

    SPI_init();

    pcur_Tr= TrBuf;
    pTr_Buf= TrBuf;

    pcur_Rcv= RcvBuf;
    pRcv_Buf= RcvBuf;

   for (i=0;i<64;i++)          strng[i]     =0x0;
   for (i=0;i<64;i++)             sr[i]     =0x0;
   for (i=0;i<64   ;i++)        Word[i]     =0x0;
   for (i=0;i<64;   i++)   DATA_Word[i]     =0x0;
   for (i=0;i<64;  i++)   DATA_Word2[i]     =0x0;  
   for (i=0;i<64   ;i++)       InOut[i]     =0x0;

   	i=0;

  

	RX_485();



 while(1){

  if (i!=12) i=i+1; else {i=14;  Menu1();}
   
   UART_control ();
    
   PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400A8000)), 0x0040U);
   delay(100000);
   PORT_ResetBits(((MDR_PORT_TypeDef *) (0x400A8000)), 0x0040U);
   
   delay(100000);
   PORT_SetBits  (((MDR_PORT_TypeDef *) (0x400A8000)), 0x0080U);
 
   delay(100000);
   PORT_ResetBits(((MDR_PORT_TypeDef *) (0x400A8000)), 0x0080U);
  
   delay(100000);

   RX_485();
      
  	}
 
}


