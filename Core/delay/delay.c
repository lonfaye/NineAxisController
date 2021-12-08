#include "delay.h"
#include "stm32f1xx_hal.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//锟斤拷锟绞癸拷锟絬cos,锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷头锟侥硷拷锟斤拷锟斤拷.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使锟斤拷
#endif

////////////////////////////////////////////////////////////////////////////////// 	 

static u8  fac_us=0;//us锟斤拷时锟斤拷锟斤拷锟斤拷
static u16 fac_ms=0;//ms锟斤拷时锟斤拷锟斤拷锟斤拷,锟斤拷ucos锟斤拷,锟斤拷锟斤拷每锟斤拷锟斤拷锟侥碉拷ms锟斤拷

#ifdef OS_CRITICAL_METHOD 	//锟斤拷锟絆S_CRITICAL_METHOD锟斤拷锟斤拷锟斤拷,说锟斤拷使锟斤拷ucosII锟斤拷.
//systick锟叫断凤拷锟斤拷锟斤拷,使锟斤拷ucos时锟矫碉拷
void SysTick_Handler(void)
{				   
	OSIntEnter();		//锟斤拷锟斤拷锟叫讹拷
    OSTimeTick();       //锟斤拷锟斤拷ucos锟斤拷时锟接凤拷锟斤拷锟斤拷锟�
    OSIntExit();        //锟斤拷锟斤拷锟斤拷锟斤拷锟叫伙拷锟斤拷锟叫讹拷
}
#endif

void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));
  if (SysTick_CLKSource == SYSTICK_CLKSOURCE_HCLK)
  {
    SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
  }
  else
  {
    SysTick->CTRL &= SYSTICK_CLKSOURCE_HCLK_DIV8;
  }
}
			   
//锟斤拷始锟斤拷锟接迟猴拷锟斤拷
//锟斤拷使锟斤拷ucos锟斤拷时锟斤拷,锟剿猴拷锟斤拷锟斤拷锟绞硷拷锟絬cos锟斤拷时锟接斤拷锟斤拷
//SYSTICK锟斤拷时锟接固讹拷为HCLK时锟接碉拷1/8
//SYSCLK:系统时锟斤拷
void delay_init(u8 SYSCLK)
{
#ifdef OS_CRITICAL_METHOD 	//锟斤拷锟絆S_CRITICAL_METHOD锟斤拷锟斤拷锟斤拷,说锟斤拷使锟斤拷ucosII锟斤拷.
	u32 reload;
#endif
	SysTick_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=72000000/8000000;
// 	SysTick->CTRL&=~(1<<2);	//SYSTICK使锟斤拷锟解部时锟斤拷源
//	fac_us=SYSCLK;///8;		//锟斤拷锟斤拷锟角凤拷使锟斤拷ucos,fac_us锟斤拷锟斤拷要使锟斤拷
	    
#ifdef OS_CRITICAL_METHOD 	//锟斤拷锟絆S_CRITICAL_METHOD锟斤拷锟斤拷锟斤拷,说锟斤拷使锟斤拷ucosII锟斤拷.
	reload=SYSCLK/8;		//每锟斤拷锟接的硷拷锟斤拷锟斤拷锟斤拷 锟斤拷位为K
	reload*=1000000/OS_TICKS_PER_SEC;//锟斤拷锟斤拷OS_TICKS_PER_SEC锟借定锟斤拷锟绞憋拷锟�
							//reload为24位锟侥达拷锟斤拷,锟斤拷锟街�:16777216,锟斤拷72M锟斤拷,约锟斤拷1.86s锟斤拷锟斤拷
	fac_ms=1000/OS_TICKS_PER_SEC;//锟斤拷锟斤拷ucos锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟劫碉拷位
	SysTick->CTRL|=1<<1;   	//锟斤拷锟斤拷SYSTICK锟叫讹拷
	SysTick->LOAD=reload; 	//每1/OS_TICKS_PER_SEC锟斤拷锟叫讹拷一锟斤拷
	SysTick->CTRL|=1<<0;   	//锟斤拷锟斤拷SYSTICK
#else
	fac_ms=(u16)fac_us*1000;//锟斤拷ucos锟斤拷,锟斤拷锟斤拷每锟斤拷ms锟斤拷要锟斤拷systick时锟斤拷锟斤拷
#endif
}								    

#ifdef OS_CRITICAL_METHOD 	//锟斤拷锟絆S_CRITICAL_METHOD锟斤拷锟斤拷锟斤拷,说锟斤拷使锟斤拷ucosII锟斤拷.
//锟斤拷时nus
//nus为要锟斤拷时锟斤拷us锟斤拷.
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;	//LOAD锟斤拷值
	ticks=nus*fac_us; 			//锟斤拷要锟侥斤拷锟斤拷锟斤拷
	tcnt=0;
	OSSchedLock();				//锟斤拷止ucos锟斤拷锟饺ｏ拷锟斤拷止锟斤拷锟絬s锟斤拷时
	told=SysTick->VAL;        	//锟秸斤拷锟斤拷时锟侥硷拷锟斤拷锟斤拷值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//锟斤拷锟斤拷注锟斤拷一锟斤拷SYSTICK锟斤拷一锟斤拷锟捷硷拷锟侥硷拷锟斤拷锟斤拷锟酵匡拷锟斤拷锟斤拷.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;//时锟戒超锟斤拷/锟斤拷锟斤拷要锟接迟碉拷时锟斤拷,锟斤拷锟剿筹拷.
		}  
	};
	OSSchedUnlock();			//锟斤拷锟斤拷ucos锟斤拷锟斤拷
}
//锟斤拷时nms
//nms:要锟斤拷时锟斤拷ms锟斤拷
void delay_ms(u16 nms)
{	
	if(OSRunning==OS_TRUE)//锟斤拷锟給s锟窖撅拷锟斤拷锟斤拷锟斤拷
	{		  
		if(nms>=fac_ms)//锟斤拷时锟斤拷时锟斤拷锟斤拷锟絬cos锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷
		{
   			OSTimeDly(nms/fac_ms);//ucos锟斤拷时
		}
		nms%=fac_ms;			//ucos锟窖撅拷锟睫凤拷锟结供锟斤拷么小锟斤拷锟斤拷时锟斤拷,锟斤拷锟斤拷锟斤拷通锟斤拷式锟斤拷时
	}
	delay_us((u32)(nms*1000));	//锟斤拷通锟斤拷式锟斤拷时
}
#else//锟斤拷锟斤拷ucos时
//锟斤拷时nus
//nus为要锟斤拷时锟斤拷us锟斤拷.
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; //时锟斤拷锟斤拷锟�
	SysTick->VAL=0x00;        //锟斤拷占锟斤拷锟斤拷锟�
	SysTick->CTRL|=0x01 ;      //锟斤拷始锟斤拷锟斤拷
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//锟饺达拷时锟戒到锟斤拷
	SysTick->CTRL&=0x00;       //锟截闭硷拷锟斤拷锟斤拷
	SysTick->VAL =0X00;       //锟斤拷占锟斤拷锟斤拷锟�
}
////锟斤拷时nms
////注锟斤拷nms锟侥凤拷围
////SysTick->LOAD为24位锟侥达拷锟斤拷,锟斤拷锟斤拷,锟斤拷锟斤拷锟绞蔽�:
////nms<=0xffffff*8*1000/SYSCLK
////SYSCLK锟斤拷位为Hz,nms锟斤拷位为ms
////锟斤拷72M锟斤拷锟斤拷锟斤拷,nms<=1864
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;//时锟斤拷锟斤拷锟�(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //锟斤拷占锟斤拷锟斤拷锟�
	SysTick->CTRL|=0x01 ;          //锟斤拷始锟斤拷锟斤拷
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//锟饺达拷时锟戒到锟斤拷
	SysTick->CTRL&=0x00;       //锟截闭硷拷锟斤拷锟斤拷
	SysTick->VAL =0X00;       //锟斤拷占锟斤拷锟斤拷锟�
}

//void RCCdelay_us(uint32_t udelay)
//{
//  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
//    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
//  do
//  {
//    __NOP();
//  }
//  while (Delay --);
//}
//
//void delay_us(uint32_t t)
//{
//	RCCdelay_us(t);
//}
//
//void delay_ms(uint32_t t)
//{
//	HAL_Delay(t);
//}

#endif
			 



































