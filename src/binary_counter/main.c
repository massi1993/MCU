typedef struct{
        unsigned int AHBENR;
}RCC_Type;

typedef struct{
        unsigned int MODER;
        unsigned int OTYPER;
        unsigned int OSPEEDR;
        unsigned int PUPDR;
        unsigned int IDR;
	        unsigned int ODR;
}GPIO_Type;
	 
#define RCC ((RCC_Type*) 0x40021014)
#define GPIOE ((GPIO_Type*) 0x48001000)
#define GPIOA ((GPIO_Type*) 0x48000000)
 
int contatore=0;
main(){
	//ABILITO LA GPIOE E LA PORTA GPIOA
	RCC->AHBENR|=(1<<21)|(1<<17);
	 
	//IMPOSTO LE PORTE E DEI LED COME USCITA E LA PORTA A COME INGRESSO (di default è settato come input)
	//IMPOSTO COME USCITA I LED DA PE8 A PE15
	GPIOE->MODER|=(1<<16)|(1<<18)|(1<<20)|(1<<22)|(1<<24)|(1<<26)|(1<<28)|(1<<30); 
	 
	while(1)
	{
	  while(((GPIOA->IDR) & (1)) == 0); //FINCHè IL TASTO USER NON è PREMUTO NON FARE NULLA
	 
	  //ASPETTO CHE VENGA PREMUTO IL TASTO
	  while(((GPIOA->IDR)&(1))==1);
	  contatore++;
	  GPIOE->ODR=contatore<<8;
	}

}//end main 
