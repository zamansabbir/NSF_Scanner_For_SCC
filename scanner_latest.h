/**
	*@author Md Sabbir Bin Zaman
	*@property of Embedded System Research & Advanced Protoryping Lab(ESARP)
*/

#define freq 6280000 //NSF_SCC_SCANNER 1
//#define freq 4920000 //NSF_SCC_SCANNER 2
//#define R1 220
//#define R2 150

#define bufferSize 256
#define bufferSizeNR 40056 //for 100 data
//#define offlineMode 1
//#define offlineTimeWindowMax 60 // in seconds 
#define offlineSamplingRate 236 // in sps
//#define offlineBufferSize offlineTimeWindowMax*offlineSamplingRate*3 //
#define NumberOfFilterCoefficient 101// depends on the order
#define tout 25
#define timeUnit_TimeWindow 10
int input[NumberOfFilterCoefficient];
void putDDSToSleep();
float h[NumberOfFilterCoefficient]={-0.00020045,-8.7971e-05,7.4004e-05,0.00028657,0.0005473,0.00084978,0.0011834,0.0015334,0.0018809,0.0022038,0.0024774,0.0026754,0.0027714,0.0027404,0.0025605,0.0022143,0.0016908,0.00098683,0.00010833,-0.00092872,-0.0020977,-0.0033614,-0.0046721,-0.0059728,-0.0071986,-0.0082786,-0.0091383,-0.0097026,-0.0098985,-0.0096585,-0.0089239,-0.0076476,-0.005797,-0.0033565,-0.0003291,0.0032623,0.0073744,0.011944,0.016889,0.02211,0.027494,0.032917,0.038247,0.043352,0.048097,0.052357,0.056017,0.058975,0.06115,0.06248,0.062928,0.06248,0.06115,0.058975,0.056017,0.052357,0.048097,0.043352,0.038247,0.032917,0.027494,0.02211,0.016889,0.011944,0.0073744,0.0032623,-0.0003291,-0.0033565,-0.005797,-0.0076476,-0.0089239,-0.0096585,-0.0098985,-0.0097026,-0.0091383,-0.0082786,-0.0071986,-0.0059728,-0.0046721,-0.0033614,-0.0020977,-0.00092872,0.00010833,0.00098683,0.0016908,0.0022143,0.0025605,0.0027404,0.0027714,0.0026754,0.0024774,0.0022038,0.0018809,0.0015334,0.0011834,0.00084978,0.0005473,0.00028657,7.4004e-05,-8.7971e-05,-0.00020045
};

//int getPrescalerForTimer4();
//int getPeriodForTimer4();
float filtered_data=0;
//SPI_HandleTypeDef
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
//UART_HandleTypeDef
UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
ADC_HandleTypeDef hadc1;
//bluetooth string
char time_echo[7]={'0','0','0','0','0',0xff,0xff},b[2]={0,0},bufferA[bufferSize],bufferB[bufferSize],bufferNonReal[bufferSizeNR];//bufferOffline[offlineBufferSize];//,but[500],str[2],
uint8_t  activeBufferA=1,activeBufferB=0,terminationTokenSent=0; //,transmissionDone=0,ss=sizeof(b),reset=0,
char rx_index=0,Rx_data[1],Rx_Buffer[3]={0,0,'\0'};
uint8_t  diseaseType=0, sensorType=0,timeWindowMessage1=0,timeWindowMessage2=0,timeWindowMessage3=0,extraB=0,extraA=0,timeKeeper=0,temporaryTimerWindow=0;//blInit=0, permissionForTransmissionA=0,permissionForTransmissionB=0
uint16_t adcValue=0,bluetoothTransmission=0,numberOfSampleInBufferA=0,numberOfSampleInBufferB=0,loopCounterA=0,loopCounterB=0,loopCounter,loopCounterOffline=0,dataCheckLoopCounter=0;//,transmissionloop
uint16_t dataCheck[offlineSamplingRate];
uint8_t volatile dataCheckDone=0,resetWindowDone=0,timerHandshaking=0;
uint16_t volatile timeWindow1=0,timeWindow2=0;
int timeWindow=0;
uint8_t volatile R1=50,R2=50,handshaking=0;
char adcTransmit[2]={0,0};
int totalSample=0,adcConversion=0;
uint8_t sps=0,offlineMode=1; //30K samples per second: sps=0;100 samples per second: sps=1;1K samples per second: sps=2
//printf formatting
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
		
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,100);
	return ch;
}
/**
	initialize the scanner for DDS frequency, digital potentiometer reading and adc conversion
	@param nothing
	@return nothing
*/
void initializeTimeWindowCounter();
void generateSignal(int frequency);
void setResistor(uint8_t resistorAorB, uint8_t wiperPosition);
void resetAllBuffer();
void resetFilterBuffer();
char checkData();
void storeDataForDataCheck();
float filterData();
void initializeScanerTimer();
//void initialize_DMA();
//static void MX_SPI3_Init(void);
void setR1R2(uint8_t r1,uint8_t r2){
	R1=r1;
	R2=r2;
}
void setToFiveHz(){

	  HAL_TIM_Base_Start_IT(&htim3);//Interrupt triggerd again for 5Hz
}
void resetFiveHz(){
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);

}
void setToOneHz(){
	  HAL_TIM_Base_Start_IT(&htim5);//Interrupt triggerd again for 1Hz
		
}
void resetOneHz(){
	HAL_TIM_Base_Stop_IT(&htim5);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
}
void setSteady(){
	resetOneHz();
	resetFiveHz();
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
}
void resetSteady(){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
}
void showScannerConnected(){
	resetFiveHz();
	resetOneHz();
	resetSteady();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
}

void showScannerDisconnected(){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
					setToFiveHz();
}
void setTimeWindow(){

	adcConversion=0;

	setResistor(1,50);
	setResistor(0,50);
	

	//setSteady();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET); //Green Off for consistency
	setToOneHz();


	loopCounter=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	generateSignal(freq);
	//starting timeWindow Timer: TIMER4
	HAL_TIM_Base_Start_IT(&htim4);
	//Starting Sampling Rate Timer: TIMER2
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buffer, 256);
}
void resetTimeWindow(){
	handshaking=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	resetWindowDone=1;
	putDDSToSleep();
	//Stopping sampling rate Timer:TIMER2
	//HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim2);
	//HAL_ADC_Stop_IT(&hadc1);
	//Stopping timeWindow Timer:TIMER4
	HAL_TIM_Base_Stop_IT(&htim4);
//HAL_TIM_Base_Stop_IT(&htim6);


	//setToFiveHz();
	sps=0;
	diseaseType=0;
	sensorType=0;
	timeWindowMessage1=0;
	timeWindowMessage2=0;
	

					b[0]=0xff;
					b[1]=0xff;

			//if((activeBufferA==1||activeBufferB==1)&&)
			if(!offlineMode){
			if((activeBufferB==1&&activeBufferA==0)&&(loopCounterB<bufferSize&&(loopCounterB>=1))){

					putDDSToSleep();
				//HAL_UART_Transmit_DMA(&huart2, bufferB, numberOfSampleInBufferB);
					HAL_UART_Transmit(&huart2,bufferB,numberOfSampleInBufferB,tout);
					activeBufferA=0;
					activeBufferB=0;
					numberOfSampleInBufferB=0;
				//	b[0]=0xff;
				//	b[1]=0xff;
				//	initialize_DMA();
					HAL_UART_Transmit(&huart2,b,2,100);
			}else if((activeBufferA==1&&activeBufferB==0)&&(loopCounterA<bufferSize&&(loopCounterA>=1))){
				//extraA=1;
				putDDSToSleep();
				//HAL_UART_Transmit_DMA(&huart2, bufferA, numberOfSampleInBufferA);
				HAL_UART_Transmit(&huart2,bufferA,numberOfSampleInBufferA,tout);
								activeBufferA=0;
								activeBufferB=0;
								numberOfSampleInBufferA=0;
							//	b[0]=0xff;
							//	b[1]=0xff;
								//initialize_DMA();
				
						//		HAL_UART_Transmit(&huart2,bufferNonReal,(adcConversion<<1),2000);
								HAL_UART_Transmit(&huart2,b,2,100);
			}else{
			HAL_UART_Transmit(&huart2,b,2,100);
			}
			}
			putDDSToSleep();
			if(offlineMode){
			putDDSToSleep();
			HAL_UART_Transmit(&huart2,bufferNonReal,(adcConversion<<1),1200);
			HAL_UART_Transmit(&huart2,b,2,10);
			}
			

//HAL_UART_Transmit(&huart2,b,2,100);
	//while(extraB!=0&&extraA!=0);
	resetAllBuffer();
	setR1R2(128,128);
	timeWindow1=0;
	timeWindow2=0;
	timeWindowMessage3=0;
	timeWindow=0;
	
	 //CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_ADCEN);
	//printf("ready for next\r\n");
//	HAL_Delay(10);
			putDDSToSleep();
			resetWindowDone=1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//			resetFiveHz();
	//		resetOneHz();
	//		resetSteady();
			//setToFiveHz();
}

void initializeScanner(){
	b[0]=0xff;
	b[1]=0xff;
	//setSteady();
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	handshaking=1;
	setToOneHz();
	timerHandshaking=0;
//HAL_TIM_Base_Start_IT(&htim6);;
//	b[2]=0x0a;
						if (diseaseType==0){
										
										if(Rx_Buffer[0]=='S'&&Rx_Buffer[1]=='A'){
											diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											printf("SA");
											HAL_UART_Transmit(&huart2,b,2,100);
											//printf("SA");
										}else if(Rx_Buffer[0]=='s'&&Rx_Buffer[1]=='a'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("sa");
											HAL_UART_Transmit(&huart2,b,2,100);
										//printf("sa");
										}else if(Rx_Buffer[0]=='B'&&Rx_Buffer[1]=='T'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("BT");
										HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='b'&&Rx_Buffer[1]=='t'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("bt");
										HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='P'&&Rx_Buffer[1]=='D'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("PD");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='p'&&Rx_Buffer[1]=='d'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("pd");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='A'&&Rx_Buffer[1]=='R'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("AR");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='a'&&Rx_Buffer[1]=='r'){
										diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("ar");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='O'&&Rx_Buffer[1]=='S'){
											diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											printf("OS");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='o'&&Rx_Buffer[1]=='s'){
											diseaseType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											printf("os");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else{
											
											//HAL_Delay(1);
											printf("er");
											HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
											while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){
												HAL_Delay(50);
											}//else {
											//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);}
											printf("er");
											resetTimeWindow();
											HAL_UART_Transmit(&huart2,b,2,100);
											diseaseType=0;
											
										}
							}else if (diseaseType==1 && sensorType==0){
										if(Rx_Buffer[0]=='E'&&Rx_Buffer[1]=='C'){
											sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											sps=2;
											offlineMode=0;
											printf("EC");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='e'&&Rx_Buffer[1]=='c'){
											sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											sps=2;
											offlineMode=0;
											printf("ec");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='H'&&Rx_Buffer[1]=='R'){
											sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											printf("HR");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='h'&&Rx_Buffer[1]=='r'){
											sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											printf("hr");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='B'&&Rx_Buffer[1]=='R'){
											sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
											sps=1;
											//R1=200;
											//R2=200;
											setR1R2(128,175);
											printf("BR");
											HAL_UART_Transmit(&huart2,b,2,100);
											
										//printf("BR");
										}else if(Rx_Buffer[0]=='b'&&Rx_Buffer[1]=='r'){
										sensorType=1;
										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("br");
											sps=1;
										//	setR1R2(175,175);
												setResistor(1,10);
	                      setResistor(0,10);
	
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='P'&&Rx_Buffer[1]=='O'){
										sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("PO");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='p'&&Rx_Buffer[1]=='o'){
										sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("po");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='T'&&Rx_Buffer[1]=='P'){
										sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("TP");
											HAL_UART_Transmit(&huart2,b,2,100);
										}else if(Rx_Buffer[0]=='t'&&Rx_Buffer[1]=='p'){
										sensorType=1;
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
										printf("tp");
											HAL_UART_Transmit(&huart2,b,2,100);
											setR1R2(100,100);
										}else{
											
											printf("er");
											HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
											while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){
												HAL_Delay(50);
											}//else {
											//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);}
											//printf("er");
											resetTimeWindow();
											HAL_UART_Transmit(&huart2,b,2,100);
											sensorType=0;
										//	diseaseType=0;
										}
										
						}else if(diseaseType==1&&sensorType==1&&timeWindowMessage1==0&&timeWindowMessage2==0&&timeWindowMessage3==0){
							
							timeWindow1=(Rx_Buffer[0]-48)*10+(Rx_Buffer[1]-48);
							time_echo[0]=(timeWindow1/10)+48;
							time_echo[1]=(timeWindow1%10)+48;
							//printf(time_echo);
							//HAL_UART_Transmit(&huart2,time_echo,4,100);
							timeWindow1*=1000;
							timeWindowMessage1=1;
							
						}else if(diseaseType==1&&sensorType==1&&timeWindowMessage1==1&&timeWindowMessage2==0&&timeWindowMessage3==0){
							timeWindow2=(Rx_Buffer[0]-48)*10+(Rx_Buffer[1]-48);
							time_echo[2]=(timeWindow2/10)+48;
							time_echo[3]=(timeWindow2%10)+48;
							timeWindow2*=10;
						//	HAL_UART_Transmit(&huart2,time_echo,6,100);
							rx_index=1;
							timeWindowMessage2=1;
							
						}else if(diseaseType==1&&sensorType==1&&timeWindowMessage1==1&&timeWindowMessage2==1&&timeWindowMessage3==0){
							timeWindow=(Rx_Buffer[1]-48);
							time_echo[4]=(timeWindow+48);
							//time_echo[5]=(timeWindow%10)+48;
							timeWindow=timeWindow1+timeWindow2+timeWindow;;
							HAL_UART_Transmit(&huart2,time_echo,7,100);		
							timeWindowMessage3=1;
							if(!(timeWindow>0&&timeWindow<=99000)){
									timeWindowMessage1=0;
									timeWindowMessage2=0;
									timeWindowMessage3=0;
									printf("er");
									//printf("er");
											HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
											HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
											while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){
												HAL_Delay(50);
											}//else {
											//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);}
											//printf("er");
											resetTimeWindow();
									HAL_UART_Transmit(&huart2,b,2,100);
								//	sensorType=0;
									//diseaseType=0;
								}
							
						}else if(diseaseType==1&&sensorType==1&&timeWindowMessage1==1&&timeWindowMessage2==1&&timeWindowMessage3==1){
							if((Rx_Buffer[0]=='O'||Rx_Buffer[0]=='o')&&(Rx_Buffer[1]=='K'||Rx_Buffer[1]=='k')){

								if((timeWindow>0&&timeWindow<=99000)){
									initializeTimeWindowCounter();
								//	HAL_TIM_Base_Stop_IT(&htim6);
									initializeScanerTimer();
								
								// datacheck starts here
							//	if(checkData()==1)
								//dataCheck ends here
										{
											//redOrGreen=0;
											setTimeWindow();
										}
								}
								//call function to generate DDS, change value of digital potentiometer and 
							}else if((Rx_Buffer[0]=='N'||Rx_Buffer[0]=='n')&&(Rx_Buffer[1]=='K'||Rx_Buffer[1]=='k')){
								timeWindowMessage1=0;
								timeWindowMessage2=0;
								timeWindowMessage3=0;
								//sensorType=0;
								//diseaseType=0;
								printf("er");
								HAL_UART_Transmit(&huart2,b,2,100);
							}else{
								//timeWindowMessage=0;
								timeWindowMessage1=0;
								timeWindowMessage2=0;
								timeWindowMessage3=0;
								//sensorType=0;
								//diseaseType=0;
								printf("er");
								HAL_UART_Transmit(&huart2,b,2,100);
								
							}
								
							
						}

	
}
/**
void transmitBluetooth(char* s){
	printf(s);
}
*/
void storeToBuffer(uint16_t data){

			
				b[0]=((data)>>8)&0x00FF; //most significant bits
				b[1]=(data)&0x00FF; //least signnificant bits	
		

{
		//buffer implementation bufferA, bufferB, send the inactive buffer data
			if(loopCounterA<bufferSize&&activeBufferA==1){
			
				bufferA[loopCounterA++]=b[0];//most significant byte
				bufferA[loopCounterA++]=b[1]; //least signnificant bits
				//bufferA[loopCounterA++]=b[2];
				numberOfSampleInBufferA=loopCounterA;
				//bufferA[loopCounterA++]=b[3];
				
		}
		if(loopCounterB<bufferSize&&activeBufferB==1){
			
				bufferB[loopCounterB++]=b[0];//most significant byte
				bufferB[loopCounterB++]=b[1]; //least signnificant bits
				//bufferB[loopCounterB++]=b[2];
			//	bufferB[loopCounterB++]=b[3];
			numberOfSampleInBufferB=loopCounterB;
		}
	}
		//buffer implementation done
				
		
		//Original transmission without buffer
		//HAL_UART_Transmit(&huart2,b,ss,100);

////		++bluetoothTransmission;//debug variables

}

void storeAndSendBuffer(uint16_t data){
	
					//STORING
					//b[0]=((data)>>8)&0x00FF; //most significant bits
					//b[1]=(data)&0x00FF; //least signnificant bits	
	if(!offlineMode){
	
					if(activeBufferA==1&&loopCounterA<bufferSize){
							
								bufferA[loopCounterA++]=((data)>>8)&0x00FF;//most significant byte
								bufferA[loopCounterA++]=(data)&0x00FF; ; //least signnificant bits
								numberOfSampleInBufferA=loopCounterA;
								
						}if(activeBufferB==1&&loopCounterB<bufferSize){
							
								bufferB[loopCounterB++]=((data)>>8)&0x00FF; //most significant byte
								bufferB[loopCounterB++]=(data)&0x00FF; //least signnificant bits
								numberOfSampleInBufferB=loopCounterB;
						}
						
					//TRANSMITTING if buffers are Full
				if(loopCounterA>=bufferSize&&activeBufferA==1){
			
				activeBufferA=0;
				activeBufferB=1;
				HAL_UART_Transmit(&huart2,bufferA,bufferSize,tout);
			
				//HAL_UART_Transmit_DMA(&huart2, bufferA, bufferSize);
				loopCounterA=0;
				numberOfSampleInBufferA=0;
				
			}else if(loopCounterB>=bufferSize&&activeBufferB==1){
				activeBufferA=1;
				activeBufferB=0;
				HAL_UART_Transmit(&huart2,bufferB,bufferSize,tout);
			
			//	HAL_UART_Transmit_DMA(&huart2, bufferB, bufferSize);
				loopCounterB=0;
				numberOfSampleInBufferB=0;
			}
			
			//if Buffers are half full
			//taken care of in reset window
		}if(offlineMode){
				bufferNonReal[adcConversion<<1]=((adcValue)>>8)&0x00FF;
				bufferNonReal[(adcConversion<<1)+1]=(adcValue)&0x00FF;
				++adcConversion;
		}
		//Transmission Section
}

/**
	Sets the reistor to a specified resistance of Digital potentiometer: AD5262
	@param choice between resistor 1 or resistor 2 (0/1)
	@param wiperPosition(0-255)
	@return nothing
*/

void setResistor(uint8_t resistorAorB, uint8_t wiperPosition){
//spi2_disable();
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
//HAL_Delay(100);
//spi2_enable();
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
HAL_SPI_Transmit_IT(&hspi2,&resistorAorB,1);
HAL_SPI_Transmit_IT(&hspi2,&wiperPosition,1);
//spi_disable();
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	
}

void write_DDS(uint8_t x){
HAL_SPI_Transmit_IT(&hspi3,&x,1);
	
}
/**
	Sets the frequency of DDS: AD9834
	@param frequency, no more than Fclk(50000000)
	@return nothing
*/
void generateSignal(int frequency){
	
	volatile int x=(frequency*5.38);

	volatile uint16_t lsb=0x0000,msb=0x0000,freqRegister=1;
	volatile uint8_t eightBit=0x00;
	lsb=(freqRegister<<14)|(x&0x00003FFF);//01+14 LSB
	msb=(freqRegister<<14)|(x>>14); //01+14MSB
	//HAL_Delay(50);
	//
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	//spi_disable();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	//HAL_Delay(100);
	//spi_enable();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);

	write_DDS(0x21);
	write_DDS(0x00);
	
	eightBit=lsb>>8;	
	//printf("%d\n",eightBit);
	write_DDS(eightBit);
	eightBit=(lsb&(0x00FF));
	write_DDS(eightBit);
	
	eightBit=msb>>8;	
	write_DDS(eightBit);
	eightBit=(msb&(0x00FF));
	write_DDS(eightBit);

	write_DDS(0xC0);
	write_DDS(0x00);
	
	write_DDS(0x20);
	write_DDS(0x00);
	//spi_disable();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	//HAL_Delay(50);
}

void putDDSToSleep(){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		//HAL_Delay(100);
		//spi_enable();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		write_DDS(0x21);
		write_DDS(0xC0);
		
		//spi_disable
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
}

void initializeTimeWindowCounter(){
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = timeWindow-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
}


/**
inside main: call this function within while looop
*/
/**

void transmitBuffer(){
	if(offlineMode==1){
		
	}else{

		if(loopCounterA>=bufferSize){
			
				activeBufferA=0;
				activeBufferB=1;
				HAL_UART_Transmit(&huart2,bufferA,bufferSize,tout);
				loopCounterA=0;
				numberOfSampleInBufferA=0;
				
			}else if(loopCounterB>=bufferSize){
				activeBufferA=1;
				activeBufferB=0;
				HAL_UART_Transmit(&huart2,bufferB,bufferSize,tout);
				loopCounterB=0;
				numberOfSampleInBufferB=0;
			}else if(extraB==1){
				activeBufferA=1;
				activeBufferB=0;
				//HAL_Delay(100);
				HAL_UART_Transmit(&huart2,bufferB,numberOfSampleInBufferB,tout);
				
				loopCounterB=0;
				numberOfSampleInBufferB=0;

					extraB=0;
					//extraA=0;
					resetAllBuffer();
				
			}else if(extraA==1){
				activeBufferA=0;
				activeBufferB=1;
			//	HAL_Delay(100);
				HAL_UART_Transmit(&huart2,bufferA,numberOfSampleInBufferA,tout);
				
				loopCounterA=0;
				numberOfSampleInBufferB=0;
					
				
					extraA=0;
					//transmission termination token 
				/**
					if(extraA==0&&extraB==0){
							if(terminationTokenSent==0){
							b[0]=0xFF;
							b[1]=0xFF;
							b[2]=0x0A;
							HAL_UART_Transmit(&huart2,b,3,100);
							terminationTokenSent=1;
					}
					}
					
					//termination token Sent
					//extraB=0;
				
					putDDSToSleep();				
					resetAllBuffer();
					  
			}
	}
	

}*/

void resetAllBuffer(){
		
	//buffer resetting done
		time_echo[0]=0; 
		time_echo[1]=0;
		time_echo[2]=0; 
		time_echo[3]=0;
		time_echo[4]=0;
		time_echo[5]=0xff;//time_echo[5]={'0','0','\r','\n','\0'}
		time_echo[6]=0xff;
		b[0]=0;b[1]=0;//b[2]=10;//b[3]={0,0,10} //adc most significant byte and least significant byte ready
		Rx_Buffer[0]=0;Rx_Buffer[1]=0;Rx_Buffer[2]='\0';//Rx_Buffer[3]={0,0,'\0'}
		Rx_data[0]=0;
		rx_index=0;
		loopCounterOffline=0;//offline buffer loopCounter
		terminationTokenSent=0;
		dataCheckDone=0;
		//extraB=0;extraA=0;
		timeKeeper=0;
		offlineMode=1;
		activeBufferA=1;
		activeBufferB=0;
		//delete the variables below, these are for debugging
			totalSample=0;
			adcConversion=0;
			bluetoothTransmission=0;
			loopCounterA=0;
			loopCounterB=0;
			for(int i=0;i<bufferSize;i++){
				bufferA[i]=0;
				bufferB[i]=0;
			}
			sps=0;
			
			
		//debug variables
//		putDDSToSleep();	
		resetFilterBuffer();
		
	}
	
	void resetFilterBuffer(){
		for(int filter_counter=0;filter_counter<NumberOfFilterCoefficient;filter_counter++){
				input[filter_counter]=0;
			}
	}
	float filterData(uint16_t data){
					filtered_data=0;
			for(int i=0;i<NumberOfFilterCoefficient;i++){
				filtered_data+=input[i]*h[NumberOfFilterCoefficient-i];
			}
			//shift operation
			for(int i=0;i<NumberOfFilterCoefficient-1;i++){
				input[i]=input[i+1];
			}
			input[NumberOfFilterCoefficient-1]=data;                       
			return filtered_data; 
	}
	char checkData(){
			generateSignal(freq);
		//generateSignal(9800000);	
			setResistor(1,128);
			setResistor(0,128);
			setSteady();
			temporaryTimerWindow=timeWindow;
			timeWindow=1;
			//initialization done
			for(dataCheckLoopCounter=0;dataCheckLoopCounter<offlineSamplingRate;dataCheckLoopCounter++){
				dataCheck[dataCheckLoopCounter]=0;
			}
			
		HAL_TIM_Base_Start_IT(&htim4);
		//Starting Sampling Rate Timer: TIMER2
		HAL_TIM_Base_Start_IT(&htim2);
		dataCheckDone=1;
		resetFilterBuffer();
		//Putting Original timeWindow data back to the variable
		timeWindow=temporaryTimerWindow;
		return 1;
	}
	void storeDataForDataCheck(uint16_t data){
		filtered_data=filterData(data);
		data=(uint16_t) filtered_data;
		if(dataCheckLoopCounter<offlineSamplingRate){
			dataCheck[dataCheckLoopCounter++]=data;
		}
	}
	

void initializeScanerTimer(){
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(sps==0){
		htim2.Init.Prescaler = 16;
		htim2.Init.Period = 30;
	}else if(sps==1){
		htim2.Init.Prescaler = 16000;
		htim2.Init.Period = 9; //10ms period
	}else if(sps=2){
		htim2.Init.Prescaler = 1600;
		htim2.Init.Period = 8; //1ms period
	}
	
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}