/*
 * tones.c
 *
 *  Created on: Apr 25, 2024
 *      Author: dima
 */
#include "main.h"
#include "tones.h"
#include  <math.h>
#define SIN_SIZE 128

//https://www.youtube.com/watch?v=AZWid4Y10H0
//https://newt.phys.unsw.edu.au/jw/notes.html
//https://computermusicresource.com/MIDI.Commands.html
//http://www.music.mcgill.ca/~ich/classes/mumt306/StandardMIDIfileformat.html#BMA1_1
//http://midi.teragonaudio.com/tech/midispec/aftert.htm

int32_t sinTable[SIN_SIZE];

#define  	targetFreq  51200.0f
int         FREQ  = targetFreq;
#define     SCALE  1

#define     SAMPLE_SUB_BITS 10
#define 	SIGMA_BITS 12

// Simultaneously tones
#define     NUM_TONES   16

struct tone
{
	uint32_t 	sinTablePos;
	uint32_t 	sampleSpeed;
	int32_t		volume;

}Tones[NUM_TONES];

struct midiCommand
{
	uint8_t command;
	uint8_t data1;
	uint8_t data2;
};


#define L_SIZE  256

uint16_t audio_buffer[L_SIZE*2];
volatile int cntFull = 0;
volatile int cntHalf = 0;
#if 1
float sample_notes[] = {
  NOTE_C4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_A4, NOTE_F4, NOTE_C4, NOTE_C4,
  NOTE_C4, NOTE_A4, NOTE_A4, NOTE_AS4,
  NOTE_G4, NOTE_C5, 0, NOTE_C5, NOTE_D4,
  NOTE_D4, NOTE_AS4,NOTE_AS4,NOTE_A4,
  NOTE_G4, NOTE_F4, NOTE_C4, NOTE_A4,
  NOTE_A4, NOTE_G4, NOTE_A4, NOTE_F4
};

int sample_times[] = {
  400,400,400,400,
  400,400,400,400,
  400,400,400,400,
  400,600,20,400,400,
  400,400,400,400,
  400,400,400,400,
  400,400,400,600
};
#endif
#if 0
int sample_notes[] = {
 NOTE_A4, NOTE_E3, NOTE_A4, 0,
 NOTE_A4, NOTE_E3, NOTE_A4, 0,
 NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B4, NOTE_A4, NOTE_B4, NOTE_C4, NOTE_D4,
 NOTE_E4, NOTE_E3, NOTE_A4, 0
};

int sample_times[] = {
 250, 250, 250, 250,
 250, 250, 250, 250,
 125, 125, 125, 125, 125, 125, 125, 125,
 250, 250, 250, 250
};
#endif
#if 0
int sample_notes[] = {
  392, 392, 392, 311, 466, 392, 311, 466, 392,
  587, 587, 587, 622, 466, 369, 311, 466, 392,
  784, 392, 392, 784, 739, 698, 659, 622, 659,
  415, 554, 523, 493, 466, 440, 466,
  311, 369, 311, 466, 392
};
int sample_times[] = {
  350, 350, 350, 250, 100, 350, 250, 100, 700,
  350, 350, 350, 250, 100, 350, 250, 100, 700,
  350, 250, 100, 350, 250, 100, 100, 100, 450,
  150, 350, 250, 100, 100, 100, 450,
  150, 350, 250, 100, 750
};
#endif

void setTone(float freq,float volume,int num)
{
	if(num>=0&&num<NUM_TONES)
	{
		Tones[num].sampleSpeed = freq *(SIN_SIZE<<SAMPLE_SUB_BITS)/FREQ;
		Tones[num].volume = freq==0?0:volume;
		//Tones[num].sinTablePos = 0;
	}
}
int MAX_VOL = 1;
void tones_init(TIM_HandleTypeDef *htim1)
{

	 printf("MAX_VOL %d FREQ %d \n",MAX_VOL,FREQ);
	 FREQ = targetFreq*SCALE;
	 MAX_VOL = ((int)(HAL_RCC_GetSysClockFreq()/FREQ))&~1;
	 FREQ = HAL_RCC_GetSysClockFreq()/MAX_VOL;
	 printf("tick time %d uS\n",(int)(1000000.0*L_SIZE/FREQ));
	 printf("MAX_VOL %d FREQ %d \n",MAX_VOL,FREQ);
     //MAX_VOL = (SYS_CLK_MHZ*1000000/outFreq);
	 TIM1->PSC = 0;
	 TIM1->ARR = MAX_VOL-1;
	 TIM1->DIER |=TIM_DMA_UPDATE;// (1 << 8);   // set UDE bit (update dma request enable)
     HAL_StatusTypeDef stat = HAL_TIM_PWM_Start_DMA(htim1, TIM_CHANNEL_3, (uint32_t*)&audio_buffer[0],L_SIZE*2);

	for(int k=0;k<NUM_TONES;k++)
	{
		Tones[k].sampleSpeed =  1000.0f*(SIN_SIZE<<SAMPLE_SUB_BITS)/FREQ;
		Tones[k].volume = 0;
		Tones[k].sinTablePos = 0;
	}
	for(int k=0;k<SIN_SIZE;k++)
	{
		sinTable[k] = (1<<SIGMA_BITS)*sin(2*M_PI*k/SIN_SIZE)*(MAX_VOL/2)/128;
	}

}
struct  sigmaDeltaStorage_SCALED
{
	int integral;
//	int y;
};
int sigma_delta_SCALED(struct sigmaDeltaStorage_SCALED* st,int x_SCALED)
{
	int y		 =st->integral>>SIGMA_BITS;
	if(y < 0)
		y = 0;
	if(y > (MAX_VOL))
		y = MAX_VOL;

	st->integral+= x_SCALED - (y<<SIGMA_BITS);
	return y;
}

struct  sigmaDeltaStorage_SCALED chanel;
int fmaxs = 0;
int fmins = 0;

void fillSamples(uint16_t* dest)
{
	for(int k=0;k<L_SIZE;k++)
	{
		int summ = 0;
		for(int c=0;c<NUM_TONES;c++)
		{
			Tones[c].sinTablePos+=Tones[c].sampleSpeed; //allow overflow
			int sinSample = sinTable[(Tones[c].sinTablePos>>SAMPLE_SUB_BITS)&(SIN_SIZE-1)];
			summ+= sinSample*Tones[c].volume;
		}
		/*
		if(summ>fmaxs)
		{
			fmaxs = summ;
		}
		if(summ<fmins)
		{
			fmins =summ;
		}*/
		//summ = 0;
#if     SCALE == 1
		int sample = (summ+(MAX_VOL<<SIGMA_BITS)/2)>>SIGMA_BITS;
#else
		int sample = sigma_delta_SCALED(&chanel,summ+(MAX_VOL<<SIGMA_BITS)/2);
#endif

		dest[k] =  sample;
	}
}
void play_test()
{
	for(int k=0;k<sizeof(sample_notes)/sizeof(sample_notes[0]);k++)
	{
			setTone(sample_notes[k]*2,100,0);
			HAL_Delay(sample_times[k]);
	}
	setTone(0,0,0);
	printf("fmaxs=%d ,fmins=%d\n",fmaxs>>SIGMA_BITS,fmins>>SIGMA_BITS);
}
void TIM1_TC1()
{
	cntFull++;
	fillSamples(&audio_buffer[L_SIZE]);
}

void TIM1_HT1()
{
	cntHalf++;
	fillSamples(&audio_buffer[0]);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	TIM1_TC1();
}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	TIM1_HT1();
}
