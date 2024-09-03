/*
** Copyright (C) Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This firmware is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this firmware. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#define CLK 168000000
#define IO_PA2
#define IO_TYPE 0
#define GPTIM TIM6

#define IFTIM TIM2
#define IFTIM_XRES 2
#define IFTIM_ICFL 128
#define IFTIM_ICMR TIM2_CCMR1
#define IFTIM_ICM1 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_16_N_8)
#define IFTIM_ICM2 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8)
#define IFTIM_ICM3 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_4_N_8)
#define IFTIM_ICIE TIM_DIER_CC1IE
#define IFTIM_ICR TIM2_CCR1
#define IFTIM_OCR TIM2_CCR3
#define iftim_isr tim2_isr

#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 1
#define iotim_isr tim1_brk_tim15_isr
#define iodma_isr dma1_channel1_isr

#define USART1_RX_DMA 2
#define USART1_TX_DMA 3
#define usart1_tx_dma_isr dma1_channel3_isr

#define USART2_RX_DMA 1
#define USART2_TX_DMA 6

#define tim1_com_isr tim1_trg_tim17_isr

#define TIM1_CCR5 MMIO32(TIM1_BASE + 0x48)
#define TIM_CCER_CC5E 0x10000


#define CLK_CNT(rate) ((CLK + ((rate) >> 1)) / (rate))
#define CLK_KHZ (CLK / 1000)
#define CLK_MHZ (CLK / 1000000)

#ifdef ANALOG
#define IO_ANALOG (cfg.throt_set < 100)
#elif defined IO_PA2 || defined IO_PA6 || defined ANALOG_CHAN
#define IO_ANALOG (cfg.input_mode == 1)
#else
#define IO_ANALOG 0
#endif

#ifdef ANALOG_CHAN
#define AIN_CHAN ANALOG_CHAN
#elif defined IO_PA6
#define AIN_CHAN 6
#else
#define AIN_CHAN 2
#endif

typedef struct {
	const uint16_t id;
	const char revision;
	const char target_type;
	const char target_name[15];
	const char _null;
	char arm;
	char damp;
	char revdir;
	char brushed;
	char timing;
	char sine_range;
	char sine_power;
	char freq_min;
	char freq_max;
	char duty_min;
	char duty_max;
	char duty_spup;
	char duty_ramp;
	char duty_rate;
	char duty_drag;
	char throt_mode;
	char throt_set;
	char throt_cal;
	uint16_t throt_min;
	uint16_t throt_mid;
	uint16_t throt_max;
	uint16_t analog_min;
	uint16_t analog_max;
	char input_mode;
	char input_chid;
	char telem_mode;
	char telem_phid;
	char telem_poles;
	uint16_t prot_stall;
	char prot_temp;
	char prot_volt;
	char prot_cells;
	char prot_curr;
	char music[256];
	char volume;
	char beacon;
	char bec;
	char led;
} Cfg;

typedef struct {
	int Kp, Ki, Kd, Li, i, x;
} PID;

extern char _cfg[], _cfg_start[], _cfg_end[], _rom[], _ram[], _boot[], _vec[]; // Linker exports
extern const uint16_t sinedata[];
extern const Cfg cfgdata;
extern Cfg cfg;
extern int throt, ertm, erpm, temp, volt, curr, csum, dshotval, beepval;
extern char analog, telreq, telmode, flipdir, beacon, dshotext;

void init(void);
void initio(void);
void initbec(void);
void initled(void);
void inittelem(void);
void ledctl(int x);
void hsictl(int x);
void compctl(int x);
void io_serial(void);
void io_analog(void);
void adctrig(void);
void adcdata(int t, int v, int c, int x);
void delay(int ms);
void kisstelem(void);
void autotelem(void);
int execcmd(char *buf);
char crc8(const char *buf, int len);
char crc8dvbs2(const char *buf, int len);
int scale(int x, int a1, int a2, int b1, int b2);
int smooth(int *s, int x, int n);
void initpid(PID *pid, int x);
int calcpid(PID *pid, int x, int y);
void checkcfg(void);
int savecfg(void);
int resetcfg(void);
int playmusic(const char *str, int vol);

static inline int min(int a, int b) {return a < b ? a : b;}
static inline int max(int a, int b) {return a > b ? a : b;}
static inline int clamp(int x, int a, int b) {return min(max(x, a), b);}


void tim1_com_isr(void);
