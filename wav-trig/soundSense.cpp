/*
 * soundSense.cpp
 *
 * This file is part of the sim-ctl distribution (https://github.com/OpenVetSimDevelopers/sim-ctl).
 * 
 * Copyright (c) 2019 VetSim, Cornell University College of Veterinary Medicine Ithaca, NY
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <ctype.h>
#include <stdbool.h>
#include <signal.h>
#include <stdint.h>

#include <iomanip>
#include <iostream>
#include <vector>
#include <string>  
#include <cstdlib>
#include <sstream>
#include <cmath>

#ifdef USE_BBBGPIO
	#include <GPIO/GPIOManager.h>
	#include <GPIO/GPIOConst.h>

	#define TURN_ON			GPIO::HIGH
	#define TURN_OFF		GPIO::LOW
#else
	#define TURN_ON			GPIO_TURN_ON
	#define TURN_OFF		GPIO_TURN_OFF
#endif

#include "wavTrigger.h"
#include "../cardiac/rfidScan.h"

#include "../comm/simCtlComm.h"
#include "../comm/simUtil.h"
#include "../comm/shmData.h"

wavTrigger wav;
simCtlComm comm(SYNC_PORT );

struct shmData *shmData;

char msgbuf[1024];

/* str_thdata
	structure to hold data to be passed to a thread
*/
typedef struct str_thdata
{
    int thread_no;
    char message[100];
} thdata;

pthread_t threadInfo1;
pthread_t threadInfo2;
pthread_t threadInfo3;

/* prototype for thread routines */
void *sync_thread ( void *ptr );
void runHeart(void );
void runLung(void );
void initialize_timers(void );
timer_t heart_timer;
timer_t breath_timer;
timer_t rise_timer;
struct sigevent heart_sev;
struct sigevent breath_sev;
struct sigevent rise_sev;

#define HEART_TIMER_SIG		(SIGRTMIN+2)
#define BREATH_TIMER_SIG	(SIGRTMIN+3)
#define RISE_TIMER_SIG		(SIGRTMIN+4)

#define TANK_THREASHOLD_LO	1550
#define TANK_THREASHOLD_HI	2400

using namespace std;

#define MAX_BUF	255

char sioName[MAX_BUF];

int debug = 0;
int ldebug = 0;
int monitor = 0;
int soundTest = 0;

void runMonitor(void );

int
setTermios(int fd, int speed )
{
	struct termios tty;
	
	memset (&tty, 0, sizeof tty);
	if ( tcgetattr (fd, &tty) != 0)
	{
		perror("tcgetattr" );
		return -1;
	}

	cfsetspeed(&tty, speed);
	cfmakeraw(&tty);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
	tty.c_iflag &=  ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		perror("tcsetattr");
		return -1;
	}
	return 0;
}

#define MIN_VOLUME		-70
#define MAX_VOLUME		10
#define MAX_MAX_VOLUME	10

#define PULSE_VOLUME_OFF		MIN_VOLUME
#define PULSE_VOLUME_VERY_SOFT	MAX_VOLUME // -20
#define PULSE_VOLUME_SOFT		MAX_VOLUME // -10
#define PULSE_VOLUME_ON			MAX_VOLUME

#define PULSE_TRACK		103

int getPulseVolume(int pressure, int strength );
void doPulse(void);

int heartPlaying = 0;
int lungPlaying = 0;

char trkBuf[256];
	
struct current
{
	int heart_sound_volume;
	int heart_sound_mute;
	int heart_rate;
	char heart_sound[32];
	
	int left_lung_sound_volume;
	int left_lung_sound_mute;
	char left_lung_sound[32];
	int right_lung_sound_volume;
	int right_lung_sound_mute;
	char right_lung_sound[32];
	int respiration_rate;
	
	unsigned int heartCount;
	unsigned int breathCount;
	
	int masterGain;
	int leftLungGain;
	int rightLungGain;
	int heartGain;
	
	int heartStrength;
	int leftLungStrength;
	int rightLungStrength;
	int pea;
};

struct current current;

int lubdub = 0;
int inhL = 0;
int inhR = 0;

#define SOUND_TYPE_UNUSED	0
#define SOUND_TYPE_HEART	1
#define SOUND_TYPE_LUNG		2
#define SOUND_TYPE_PULSE	3
#define SOUND_TYPE_GENERAL	4

#define SOUND_TYPE_LENGTH	8
struct soundType
{
	int type;
	char typeName[SOUND_TYPE_LENGTH];
};
struct soundType soundTypes[] =
{
	{ SOUND_TYPE_UNUSED,	"unused" },
	{ SOUND_TYPE_HEART,		"heart" },
	{ SOUND_TYPE_LUNG,		"lung" },
	{ SOUND_TYPE_PULSE,		"pulse" },
	{ SOUND_TYPE_GENERAL,	"general" }
};

int
typeNameToIndex(char *typeName )
{
	int i;
	
	for ( i = 0 ; i < 5 ; i++ )
	{
		if ( strcmp(typeName, soundTypes[i].typeName ) == 0 )
		{
			return ( soundTypes[i].type );
		}
	}
	return -1;
}
#define SOUND_NAME_LENGTH	32
struct sound
{
	int type;
	int index;
	char name[SOUND_NAME_LENGTH];
	int low_limit;
	int high_limit;
};

// The Tsunami is capable of supporting up to 4096 tracks
#define SOUND_NUM_TRACKS		(4096)
int maxSounds = 0;
struct sound *soundList;
int soundIndex = 0;

int
addSoundToList(int type, int index, const char *name, int low_limit, int high_limit )
{
	int sts;
	if ( soundIndex >= maxSounds )
	{
		sprintf(msgbuf, "Too many tracks in sound list" );
		log_message("", msgbuf );
		sts = -1;
	}
	else if ( soundList[soundIndex].type == SOUND_TYPE_UNUSED )
	{
		soundList[soundIndex].type = type;
		soundList[soundIndex].index = index;
		memcpy(soundList[soundIndex].name, name, SOUND_NAME_LENGTH );
		soundList[soundIndex].low_limit = low_limit;
		soundList[soundIndex].high_limit = high_limit;
		soundIndex++;
		sts = 0;
	}
	else
	{
		sprintf(msgbuf, "bad entry in sound list" );
		log_message("", msgbuf );
		sts = -1;
	}
	return ( sts );
}

int
initSoundList(void )
{
	FILE *file;
	int lines;
	char line[1024];
	char *in;
	char *out;
	char clean[1060];
	
	struct sound sd;
	char typeName[SOUND_TYPE_LENGTH];
	int sts;
	
	file = fopen("/simulator/soundList.csv", "r" );
	if ( file == NULL )
	{
		if ( debug )
		{
			perror("fopen" );
			fprintf(stderr, "Failed to open /simulator/soundList.csv\n" );
		}
		else
		{
			sprintf(msgbuf, "Failed to open /simulator/soundList.csv %s", strerror(errno) );
			log_message("", msgbuf);
		}
		exit ( -2 );
	}
	
	for ( lines = 0 ; fgets(line, 1024, file ) ; lines++ )
	{
	}
	printf("Found %d lines in file\n", lines );
	maxSounds = lines + 1;
	soundList = (struct sound *)calloc(maxSounds, sizeof(struct sound) );
	
	fseek(file, 0, SEEK_SET );
	for ( lines = 0 ; fgets(line, 1024, file ) ; lines++ )
	{
		in = (char *)line;
		out = (char *)clean;
		while ( *in )
		{
			switch ( *in )
			{
				case '\t': case ';': case ',':
					*out = ' ';
					break;
				case ' ':
					*out = '_';
					break;
				default:
					*out = *in;
					break;
			}
			out++;
			in++;
		}
		*out = 0;
		sts = sscanf(clean, "%s %d %s %d %d", 
			typeName,
			&sd.index,
			sd.name,
			&sd.low_limit,
			&sd.high_limit );
		if ( sts == 5 )
		{
			sd.type = typeNameToIndex(typeName );
			addSoundToList( sd.type, sd.index, sd.name, sd.low_limit, sd.high_limit );
		}
		else
		{
			sprintf(msgbuf, "sscanf returns %d for line: \"%s\"\n", sts, line );
			log_message("", msgbuf);
		}
	}
	return ( 0 );
}
#if 0
void
initSoundListStatic(void)
{
	maxSounds = 4096;
	soundList = (struct sound *)calloc(maxSounds, sizeof(struct sound) );
	
	addSoundToList( SOUND_TYPE_HEART, 112, "normal",	0, 60 );
	addSoundToList( SOUND_TYPE_HEART, 113, "normal",	61, 120 );
	addSoundToList( SOUND_TYPE_HEART, 114, "normal",	121, 150  );
	addSoundToList( SOUND_TYPE_HEART, 115, "normal",	151, 180 );
	addSoundToList( SOUND_TYPE_HEART, 116, "normal",	181, 300 );
	
	addSoundToList( SOUND_TYPE_HEART, 112, "none",	 	0, 60 );
	addSoundToList( SOUND_TYPE_HEART, 113, "none",	 	61, 120 );
	addSoundToList( SOUND_TYPE_HEART, 114, "none",	 	121, 150 );
	addSoundToList( SOUND_TYPE_HEART, 115, "none",		151, 180 );
	addSoundToList( SOUND_TYPE_HEART, 116, "none",		181, 500 );
	
	addSoundToList( SOUND_TYPE_HEART, 122, "systolic_murmur",	0, 60 );
	addSoundToList( SOUND_TYPE_HEART, 123, "systolic_murmur",	61, 120 );
	addSoundToList( SOUND_TYPE_HEART, 124, "systolic_murmur",	121, 150 );
	addSoundToList( SOUND_TYPE_HEART, 125, "systolic_murmur",	151, 180 );
	addSoundToList( SOUND_TYPE_HEART, 126, "systolic_murmur",	171, 500 );
	
	addSoundToList( SOUND_TYPE_GENERAL, 1, "Sweeper", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 2, "Mountain", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 3, "Dialog1", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 4, "Dialog2", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 5, "Bark", 				0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 6, "Flute C4", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 7, "Flute E4", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 8, "Flute G4", 			0, 0 );
	addSoundToList( SOUND_TYPE_GENERAL, 9, "A440", 				0, 0 );

	addSoundToList( SOUND_TYPE_PULSE, 102, "pulse", 			0, 0 );
	addSoundToList( SOUND_TYPE_PULSE, 103, "pulse", 			0, 0 );
	
	addSoundToList( SOUND_TYPE_LUNG, 183, "normal",	 	0, 15 );
	addSoundToList( SOUND_TYPE_LUNG, 183, "normal",	 	16, 19 );
	addSoundToList( SOUND_TYPE_LUNG, 184, "normal",	 	20, 22 );
	addSoundToList( SOUND_TYPE_LUNG, 185, "normal",	 	23, 24 );
	addSoundToList( SOUND_TYPE_LUNG, 186, "normal",	 	25, 29 );
	addSoundToList( SOUND_TYPE_LUNG, 187, "normal",	 	30, 32 );
	addSoundToList( SOUND_TYPE_LUNG, 188, "normal",	 	33, 36 );
	addSoundToList( SOUND_TYPE_LUNG, 189, "normal",	 	37, 40 );
	addSoundToList( SOUND_TYPE_LUNG, 190, "normal",	 	41, 44 );
	addSoundToList( SOUND_TYPE_LUNG, 191, "normal",	 	45, 49 );
	addSoundToList( SOUND_TYPE_LUNG, 192, "normal",	 	50, 59 );
	addSoundToList( SOUND_TYPE_LUNG, 193, "normal",	 	60, 999 );
	
	addSoundToList( SOUND_TYPE_LUNG, 183, "none",	 	0, 15 );
	//{ 181, "none",	 	6, 0 );
	//{ 181, "none",	 	12, 0 );
	//{ 182, "none",	 	14, 0 );
	addSoundToList( SOUND_TYPE_LUNG, 183, "none",	 	16, 19 );
	addSoundToList( SOUND_TYPE_LUNG, 184, "none",	 	20, 22 );
	addSoundToList( SOUND_TYPE_LUNG, 185, "none",	 	23, 24 );
	addSoundToList( SOUND_TYPE_LUNG, 186, "none",	 	25, 29 );
	addSoundToList( SOUND_TYPE_LUNG, 187, "none",	 	30, 32 );
	addSoundToList( SOUND_TYPE_LUNG, 188, "none",	 	33, 36 );
	addSoundToList( SOUND_TYPE_LUNG, 189, "none",	 	37, 40 );
	addSoundToList( SOUND_TYPE_LUNG, 190, "none",	 	41, 44 );
	addSoundToList( SOUND_TYPE_LUNG, 191, "none",	 	45, 49 );
	addSoundToList( SOUND_TYPE_LUNG, 192, "none",	 	50, 59 );
	addSoundToList( SOUND_TYPE_LUNG, 193, "none",	 	60, 999 );
	
	addSoundToList( SOUND_TYPE_LUNG, 211, "coarse_crackles",	 	0, 15 );
	addSoundToList( SOUND_TYPE_LUNG, 211, "coarse_crackles",	 	16, 19 );
	addSoundToList( SOUND_TYPE_LUNG, 212, "coarse_crackles",	 	20, 22 );
	addSoundToList( SOUND_TYPE_LUNG, 213, "coarse_crackles",	 	23, 24 );
	addSoundToList( SOUND_TYPE_LUNG, 214, "coarse_crackles",	 	25, 29 );
	addSoundToList( SOUND_TYPE_LUNG, 215, "coarse_crackles",	 	30, 32 );
	addSoundToList( SOUND_TYPE_LUNG, 216, "coarse_crackles",	 	33, 36 );
	addSoundToList( SOUND_TYPE_LUNG, 217, "coarse_crackles",	 	37, 40 );
	addSoundToList( SOUND_TYPE_LUNG, 218, "coarse_crackles",	 	41, 44 );
	addSoundToList( SOUND_TYPE_LUNG, 219, "coarse_crackles",	 	45, 49 );
	addSoundToList( SOUND_TYPE_LUNG, 220, "coarse_crackles",	 	50, 59 );
	addSoundToList( SOUND_TYPE_LUNG, 221, "coarse_crackles",	 	60, 999 );
};
#endif

void
showSounds(void )
{
	int i;
	
	for ( i = 0 ; i < maxSounds ; i++ )
	{
		if ( soundList[i].type != SOUND_TYPE_UNUSED )
		{
			printf("%s,%d,%s,%d,%d\n",
				soundTypes[soundList[i].type].typeName, soundList[i].index, soundList[i].name, soundList[i].low_limit, soundList[i].high_limit );
		}
	}
}

void
getFiles(void )
{
	int breathRate = shmData->respiration.rate;
	int hr = shmData->cardiac.rate;
	int i;
	int new_inhL = -1;
	int new_inhR = -1;
	int new_lubdub = -1;
	struct sound *sound;
	
	for ( i = 0 ;  i < maxSounds ; i++ )
	{
		sound = &soundList[i];
		if ( ( sound->type == SOUND_TYPE_HEART ) && ( strcmp(sound->name, current.heart_sound ) == 0 ) && ( sound->low_limit <= hr ) && ( sound->high_limit >= hr ) )
		{
			new_lubdub = sound->index;
			break;
		}
	}
	for ( i = 0 ;  i < maxSounds ; i++ )
	{
		sound = &soundList[i];
		if ( ( sound->type == SOUND_TYPE_LUNG ) && (strcmp(sound->name, current.left_lung_sound ) == 0 ) && ( sound->low_limit <= breathRate ) && ( sound->high_limit >= breathRate ) )
		{
			new_inhL = sound->index;
			break;
		}
	}
	for ( i = 0 ;  i < maxSounds ; i++ )
	{
		sound = &soundList[i];
		if ( ( sound->type == SOUND_TYPE_LUNG ) && (strcmp(sound->name, current.right_lung_sound ) == 0 ) && ( sound->low_limit <= breathRate ) && ( sound->high_limit >= breathRate ) )
		{
			new_inhR = sound->index;
			break;
		}
	}

	if ( new_lubdub == -1 )
	{
		sprintf(msgbuf, "No lubdub file for %s %d", current.heart_sound, shmData->cardiac.rate );
		log_message("", msgbuf);
	}
	else
	{
		lubdub = new_lubdub;
	}
	if ( new_inhL == -1 )
	{
		sprintf(msgbuf, "No inhL file for %s %d", current.left_lung_sound, shmData->respiration.rate );
		log_message("", msgbuf);
	}
	else
	{
		inhL = new_inhL;
	}
	if ( new_inhR == -1 )
	{
		sprintf(msgbuf, "No inhR file for %s %d", current.right_lung_sound, shmData->respiration.rate );
		log_message("", msgbuf);
	}
	else
	{
		inhR = new_inhR;
	}
	sprintf(msgbuf, "Get Files %s : %d, %s : %d, %s : %d", 
		current.heart_sound, lubdub, current.left_lung_sound, inhL, current.right_lung_sound, inhR );
	log_message("", msgbuf);
}

unsigned int heartLast = 0;
int heartState = 0;
unsigned int lungLast = 0;
int lungState = 0;

#ifdef USE_BBBGPIO
int tankPin;
int riseLPin;
int riseRPin;
int fallPin;
//int pulsePin;

GPIO::GPIOManager* gp;
#else
FILE *tankPin;
FILE *riseLPin;
FILE *riseRPin;
FILE *fallPin;
//FILE *pulsePin;
#endif

int pumpOnOff;
int riseOnOff;
int fallOnOff;
int tankVolume;
// int riseTime;

void doReport(void )
{
	//sprintf(msgbuf, "Counts: %d, %d, Tank %d Rise %d Heart State %d, Heart Gain %d Lung State %d Lung Gain R-%d L-%d Master Gain %d  heart %d inh R-%d L-%d", 
	//			current.heartCount, current.breathCount, tankVolume, riseTime, heartState, current.heartGain, 
	sprintf(msgbuf, "Counts: %d, %d, Tank %d Heart State %d, Heart Gain %d Lung State %d Lung Gain R-%d L-%d Master Gain %d  heart %d inh R-%d L-%d", 
				current.heartCount, current.breathCount, 
				tankVolume, 
				heartState, current.heartGain, 
				lungState, current.rightLungGain, current.leftLungGain, 
				current.masterGain,
				lubdub, inhR, inhL );
	
	log_message("", msgbuf);
}

// Volume is a range of 0 to 10, 
// Strength is a range of 0 to 10
// Range of Gain of Tsunami is -70 to 10 
// But -30 is barely audible, so we use -40 as our bottom

#define MAX_CALC_GAIN	10
#define MIN_CALC_GAIN	-40
#define GAIN_CALC_RANGE (MAX_CALC_GAIN - ( MIN_CALC_GAIN ) )
int volumeToGain(int volume, int strength )
{
	int gain;
	int s1;
	int g1;
	int g2;
	
	s1 = (strength + volume)*100;
	g1 = (s1 * GAIN_CALC_RANGE );
	g2 = g1 / 2000;
	gain = g2 + MIN_CALC_GAIN;
	return ( gain );
}

int tankOn = 0;
int tankCount = 0;

void
checkTank()
{
	int ain2;
	int ain0;
	
	ain2 = read_ain(AIR_PRESSURE_AIN_CHANNEL );
	ain0 = read_ain(BREATH_AIN_CHANNEL );
	tankVolume = ain2;
	if ( debug > 2)
	{
		if ( tankCount++ > 50 )
		{
			printf("AIN : %d   %d\n", ain0, ain2 );
			tankCount = 0;
		}
	}
#if 0
	if ( ain2 > TANK_THREASHOLD_HI )
	{
		// Turn off
	#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
	#else
		gpioPinSet(tankPin, TURN_OFF );
	#endif
		if ( debug && tankOn  )
		{
			printf("Tank Off: %d\n", ain2 );
		}
		tankOn = 0;
	}
	if ( ain2 < TANK_THREASHOLD_LO )
	{
		// Turn on
	#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_ON );
	#else
		gpioPinSet(tankPin, TURN_ON );
	#endif
		if ( debug && ! tankOn )
		{
			printf("Tank On: %d\n", ain2 );
		}
		tankOn = 1;
	}
#else
	// Sensor not working. Force on. The pump stalls at around 5 PSI, so it should be ok.
	//gp->setValue(tankPin, TURN_ON );
	
	// Or, configured for external compressor. 
	#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
	#else
		gpioPinSet(tankPin, TURN_OFF );
	#endif
#endif
}
void allAirOff(void )
{
	if ( debug ) printf("ALL OFF\n" );
#ifdef USE_BBBGPIO
	gp->setValue(tankPin, TURN_OFF );
	gp->setValue(riseLPin, TURN_OFF );
	gp->setValue(riseRPin, TURN_OFF );
	gp->setValue(fallPin, TURN_OFF );
#else
	gpioPinSet(tankPin, TURN_OFF );
	gpioPinSet(riseLPin, TURN_OFF );
	gpioPinSet(riseRPin, TURN_OFF );
	gpioPinSet(fallPin, TURN_OFF );
#endif
}
/*
 * Function: ss_signal_handler
 *
 * Handle inbound signals.
 *		SIGHUP is ignored 
 *		SIGTERM closes the process
 *
 * Parameters: sig - the signal
 *
 * Returns: none
 */
int doStop;

void ss_signal_handler(int sig )
{
	switch(sig) {
	case SIGHUP:
		allAirOff();
		log_message("","hangup signal caught");
		break;
	case SIGTERM:
		allAirOff();
		log_message("","terminate signal caught");
		
		exit(0);
		break;
	}
}


int
main(int argc, char *argv[] )
{
	int sfd;
	char buffer[MAX_BUF+1];
	int i;
	int c;
	int val;
	int sts;
	struct sigaction new_action;
	int changed;
	
	if ( argc < 2 )
	{
		cout << "Usage:\n";
		cout << argv[ 0 ] << " [-d] [-m][-t] <tty port>\n";
		cout << "eg: " << argv[ 0 ] << " tty1\n";
		return (-1 );
	}
	while (( c = getopt(argc, argv, "mdt" ) ) != -1 )
	{
		switch ( c )
		{
			case 'd':
				debug = 1;
				break;
			case 'm':
				monitor = 1;
				debug = 1;
				break;
			case 't':
				monitor = 1;
				debug = 2;
				break;
		}
	}
	
	if ( monitor == 0 )
	{
		if ( optind < argc )
		{
			sprintf(sioName, "/dev/%s", argv[optind] );
		}
		else
		{
			cout << "Usage:\n";
			cout << argv[ 0 ] << " [-d] [-m] <tty port>\n";
			cout << "eg: " << argv[ 0 ] << " tty1\n";
			return (-1 );
		}
	}
	printf("Debug %d, Monitor %d, sio '%s'\n", debug, monitor, sioName );
	if ( monitor )
	{
		sts = initSHM(SHM_OPEN );
		if ( sts  )
		{
			perror("initSHM" );
			return (-1 );
		}
		runMonitor();
	}
	initSoundList();
	if ( debug )
	{
		printf("Show Sounds:\n" );
		showSounds();
	}

	// Controls for Chest Rise/Fall

#ifdef USE_BBBGPIO
	gp = GPIO::GPIOManager::getInstance();
	tankPin = GPIO::GPIOConst::getInstance()->getGpioByKey("P8_11" ); // GPIO_45
	riseLPin = GPIO::GPIOConst::getInstance()->getGpioByKey("P8_13" ); // GPIO_23
	riseRPin = GPIO::GPIOConst::getInstance()->getGpioByKey("P8_8" ); // GPIO_67
	fallPin = GPIO::GPIOConst::getInstance()->getGpioByKey("P8_10" ); // GPIO_68
//	pulsePin = GPIO::GPIOConst::getInstance()->getGpioByKey("P8_45" ); // GPIO_70
	
	gp->exportPin(tankPin );
	gp->setDirection(tankPin, GPIO::OUTPUT );	
	gp->exportPin(riseLPin );
	gp->setDirection(riseLPin, GPIO::OUTPUT );
	gp->exportPin(riseRPin );
	gp->setDirection(riseRPin, GPIO::OUTPUT );
	gp->exportPin(fallPin );
	gp->setDirection(fallPin, GPIO::OUTPUT );
//	gp->exportPin(pulsePin );
//	gp->setDirection(pulsePin, GPIO::OUTPUT );
#else
	tankPin = gpioPinOpen(45, GPIO_OUTPUT );	// P8_11
	riseLPin = gpioPinOpen(23, GPIO_OUTPUT );	// P8_13
	riseRPin = gpioPinOpen(67, GPIO_OUTPUT );	// P8_8
	fallPin = gpioPinOpen(68, GPIO_OUTPUT );	// P8_10
//	pulsePin = gpioPinOpen(70, GPIO_OUTPUT );	// P8_45
#endif
	allAirOff();
	
	if ( ( debug < 1 ) && ( ldebug == 0 ) )
	{
		daemonize();
	}
	if ( debug > 1 )
	{
		printf("Starting Timer\n" );
	}
	new_action.sa_handler = ss_signal_handler;
	sigemptyset (&new_action.sa_mask);
	new_action.sa_flags = 0;
	sigaction (SIGPIPE, &new_action, NULL);
	signal(SIGHUP,ss_signal_handler); /* catch hangup signal */
	signal(SIGTERM,ss_signal_handler); /* catch kill signal */
	
	if ( !ldebug )
	{
		sts = initSHM(SHM_OPEN );
		if ( sts  )
		{
			perror("initSHM");
			allAirOff();
			return (-1 );
		}
	}
	checkTank();
	if ( debug > 1 )
	{
		printf("Looking for WAV Trigger\n" );
	}	
	// When booted, the SIO port may not yet be available. Try every 1 second for 20 sec.
	for ( i = 0 ; i <  20 ; i++ )
	{
		checkTank();	// Start filling tank while waiting for WAV Trigger
		sfd = open(sioName, O_RDWR | O_NOCTTY | O_SYNC );
		if ( sfd < 0 )
		{
			if ( debug > 1 )
			{
				perror("open" );
				printf("Try %d\n", i );
			}
			sleep(1 );
		}
		else
		{
			break;
		}
	}
	if ( debug > 1 )
	{
		printf("Shut Off air\n" );
	}
	allAirOff(); // Make sure pump is off before doing other init operations
	if ( sfd < 0 )
	{
		sprintf(msgbuf, "No SIO Port. Running Silent" );
		log_message("", msgbuf);
	}
	else
	{
		if ( debug > 1 )
		{
			printf("Set TERMIOS\n" );
		}
		setTermios(sfd, B57600); // WAV Trigger defaults to 57600
		
		if ( debug > 1 )
		{
			printf("Start Wav\n" );
		}
	}
	wav.start(sfd );
	usleep(500000);
	
	if ( debug < 4 )
	{
		if ( debug > 1 )
		{
			printf("initialize_timers\n" );
		}
		initialize_timers();
	}
	//printf("Started\n" );
	if ( sfd < 0 )
	{
	}
	else
	{
		if ( debug > 1 )
		{
			printf("Check Tsunami Version\n" );
		}
		val = wav.getVersion(buffer, MAX_BUF );
		
		if ( debug > 1 )
		{
			printf("WAV Trigger Version: Len %d String %.*s\n", val, val-1, &buffer[1] );
		}
		else
		{
			sprintf(msgbuf, "WAV Trigger Version: Len %d String %.*s", val, val-1, &buffer[1] );
			log_message("", msgbuf);
		}
		val = wav.getSysInfo(buffer, MAX_BUF );
		if ( debug > 1 )
		{
			printf("Sys Info: Len %d Voices %d Tracks %d\n", val, buffer[1], buffer[2] );
		}
		else
		{
			sprintf(msgbuf, "Sys Info: Len %d Voices %d Tracks %d", val, buffer[1], buffer[2] );
			log_message("", msgbuf);
		}
		if ( wav.boardType == BOARD_TSUNAMI && wav.tsunamiMode != TSUNAMI_MONO )
		{
			if ( debug > 1 )
			{
				printf("Tsunami is running Stereo Mode. Must be Mono\n" );
			}
			else
			{
				sprintf(msgbuf, "Tsunami is running Stereo Mode. Must be Mono");
				log_message("", msgbuf);
			}
		}
	}
	wav.show();
	wav.ampPower(0 );
	wav.stopAllTracks();
	//wav.masterGain(0);
	wav.channelGain(0, 0 );
	current.masterGain = 0;
	for ( i = 0 ; i < 8 ; i++ )
	{
		wav.channelGain(i,0 );
	}
	wav.trackGain(5, 0 );
	wav.trackGain(113, 0 );
	wav.trackGain(PULSE_TRACK, MAX_MAX_VOLUME );
	wav.trackPlayPoly(0, 5);	// Bark
	while ( wav.getTracksPlaying() > 0 )
	{
		usleep(10000);
	}
	if ( debug > 3 )
	{
		val = getPulseVolume(PULSE_TOUCH_NORMAL, 2 );
		printf("Setting Pulse Volumes to %d\n", val );
		for ( i = 2 ; i < 6 ; i++ )
		{
			wav.channelGain(i, val );
		}
		for ( i = 0 ; i < 2000 ; i++ )
		{
			wav.trackPlayPoly(0, 113); // Pulse
			wav.trackPlayPoly(2, PULSE_TRACK); // Pulse
			wav.trackPlayPoly(3, PULSE_TRACK); // Pulse
			wav.trackPlayPoly(4, PULSE_TRACK); // Pulse
			wav.trackPlayPoly(5, PULSE_TRACK); // Pulse
			usleep(100000);
			while ( wav.getTracksPlaying() > 0 )
			{
				usleep(10000);
			}
			switch ( i % 5 )
			{
				case 0:
#ifdef USE_BBBGPIO
					gp->setValue(fallPin, TURN_ON );
#else
					gpioPinSet(fallPin, TURN_ON );
#endif
					break;
				case 1:
#ifdef USE_BBBGPIO
					gp->setValue(riseLPin, TURN_ON );
#else
					gpioPinSet(riseLPin, TURN_ON );
#endif
					break;
				case 2:
#ifdef USE_BBBGPIO
					gp->setValue(riseRPin, TURN_ON );	
#else
					gpioPinSet(riseRPin, TURN_ON );
#endif				
					break;
				case 3:
#ifdef USE_BBBGPIO
					gp->setValue(tankPin, TURN_ON );
#else
					gpioPinSet(tankPin, TURN_ON );
#endif
					break;
				case 4:
#ifdef USE_BBBGPIO
					gp->setValue(tankPin, TURN_OFF );
					gp->setValue(riseLPin, TURN_OFF );
					gp->setValue(riseRPin, TURN_OFF );
					gp->setValue(fallPin, TURN_OFF );
//					gp->setValue(pulsePin, TURN_OFF );
#else
					gpioPinSet(tankPin, TURN_OFF );
					gpioPinSet(riseLPin, TURN_OFF );
					gpioPinSet(riseRPin, TURN_OFF );
					gpioPinSet(fallPin, TURN_OFF );
//					gpioPinSet(pulsePin, TURN_OFF );
#endif				
					break;
			}

			printf("%d\n", i );
		}
		wav.trackPlayPoly(0, 5);
		allAirOff();
		exit ( 0 );
	}
	//wav.masterGain(MIN_VOLUME);
	//current.masterGain = MIN_VOLUME;
	current.heartCount = 0;
	current.breathCount = 0;
	current.leftLungGain = -65;
	current.rightLungGain = -65;
	current.heartGain = -65;
	
	wav.trackGain(PULSE_TRACK, MAX_MAX_VOLUME );
	sts = comm.openListen(LISTEN_INACTIVE );
	if ( sts )
	{
		perror("comm.openListen()" );
		allAirOff();
		exit ( -2 );
	}

	pthread_create (&threadInfo1, NULL, &sync_thread,(void *) NULL );
	
	// Main loop monitors the volumes and keeps them set
	// Also gets the track info updated

	if ( debug )
	{
		sprintf(msgbuf, "Running" );
		if ( debug > 1 )
		{
			printf("%s\n", msgbuf );
		}
		else
		{
			log_message("", msgbuf);
		}
	}
	
	wav.trackPlayPoly(0, 5);	// Bark
	while ( wav.getTracksPlaying() > 0 )
	{
		usleep(10000);
	}
	
	while ( 1 )
	{
		// Master off based on active auscultation
		if ( soundTest )
		{
			if ( current.masterGain != MAX_VOLUME )
			{
				wav.channelGain(0, MAX_VOLUME);
				current.masterGain = MAX_VOLUME;
			}
			shmData->auscultation.col  = 1;
			shmData->auscultation.row  = 1;
			shmData->auscultation.side = 1;
			shmData->auscultation.heartStrength = 10;
			shmData->auscultation.leftLungStrength = 10;
			shmData->auscultation.rightLungStrength = 0;
		}
		else
		{
		if ( ( shmData->auscultation.side == 0 ) && ( current.masterGain != MIN_VOLUME ) )
		{
			wav.channelGain(0, MIN_VOLUME);
			current.masterGain = MIN_VOLUME;
			if ( debug )
			{
				printf("Master Off\n" );
			}
			sprintf(msgbuf, "Set Off: %d, %d, Heart Gain %d, Lung Gains %d / %d, Master Gain %d", 
				current.heartCount, current.breathCount, current.heartGain, current.rightLungGain, current.leftLungGain, current.masterGain );
			log_message("", msgbuf);
		}
		else if ( ( shmData->auscultation.side != 0 ) && ( current.masterGain != MAX_VOLUME ) )
		{
			wav.channelGain(0, MAX_VOLUME);
			current.masterGain = MAX_VOLUME;
			if ( debug  )
			{
				printf("Master On\n" );
			}
			sprintf(msgbuf, "Set On: %d, %d, Heart Gain %d, Lung Gains %d / %d (%d), Master Gain %d", 
				current.heartCount, current.breathCount, current.heartGain, current.rightLungGain, current.leftLungGain, shmData->respiration.left_lung_sound_volume, current.masterGain );
			log_message("", msgbuf);
		}
		}
		// Check Air Reservoir
		checkTank();
		changed = 0;
		
		// Check for heart/lung changes
		if ( ( current.heart_rate != shmData->cardiac.rate ) || 
			 ( strcmp(current.heart_sound, shmData->cardiac.heart_sound) != 0 ) )
		{
			sprintf(msgbuf, "Cardiac %d:%d, %s, %s", 
				 current.heart_rate, shmData->cardiac.rate,
				 current.heart_sound, shmData->cardiac.heart_sound	 );
			log_message("", msgbuf);		
			current.heart_rate = shmData->cardiac.rate;
			memcpy(current.heart_sound, shmData->cardiac.heart_sound, 32 );
			changed = 1;
		}
		if ( ( current.respiration_rate != shmData->respiration.rate ) ||
			 ( strcmp(current.left_lung_sound, shmData->respiration.left_lung_sound) != 0 ) ||
			 ( strcmp(current.right_lung_sound, shmData->respiration.right_lung_sound) != 0 ) )
		{
			sprintf(msgbuf, "Resp %d:%d, %s, %s, %s, %s", 
				 current.respiration_rate, shmData->respiration.rate,
				 current.left_lung_sound, shmData->respiration.left_lung_sound,
				 current.right_lung_sound, shmData->respiration.right_lung_sound );
			log_message("", msgbuf);
			current.respiration_rate = shmData->respiration.rate;
			memcpy(current.left_lung_sound, shmData->respiration.left_lung_sound, 32 );
			memcpy(current.right_lung_sound, shmData->respiration.right_lung_sound, 32 );
			changed = 1;
		}
		if ( changed )
		{
			getFiles();
			doReport();
		}
	
		runLung();
		runHeart();
		usleep(10000 );
	}
}

void *
sync_thread ( void *ptr )
{
	int sts;
	
	sts = comm.openListen(LISTEN_ACTIVE );
	if ( sts != 0 )
	{
		perror("comm.openListen" );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -4 );
	}
	while ( 1 )
	{
		sts = comm.wait("" );
		switch ( sts )
		{
			case SYNC_PULSE:
			case SYNC_PULSE_VPC:
				current.heartCount += 1;
				break;
			case SYNC_BREATH:
				current.breathCount += 1;
				break;
		}
	}
}
void
setHeartVolume(int force )
{
	int gain = current.heartGain;
	
	if ( force ||
		 ( current.heart_sound_mute != shmData->cardiac.heart_sound_mute ) ||
		 ( current.heart_sound_volume != shmData->cardiac.heart_sound_volume ) ||
		 ( current.heartStrength != shmData->auscultation.heartStrength ) ||
		 ( current.pea != shmData->cardiac.pea ) )
	{
		current.heart_sound_mute = shmData->cardiac.heart_sound_mute;
		current.heart_sound_volume = shmData->cardiac.heart_sound_volume;
		current.heartStrength  = shmData->auscultation.heartStrength;
		current.pea = shmData->cardiac.pea;
		//if ( current.heart_sound_mute )
		//{
		//	current.heartGain = MIN_VOLUME;
		//}
		//else
		{
			if ( current.pea )
			{
				current.heartGain = MIN_VOLUME;
			}
			else
			{
				current.heartGain = volumeToGain(current.heart_sound_volume, current.heartStrength );
			}
		}
	}
	if ( force || ( gain != current.heartGain ) )
	{
		wav.trackGain(lubdub, current.heartGain );
	}
}
void
setLeftLungVolume(int force )
{
	int gain = current.leftLungGain;
	
	if ( force ||
		 ( current.left_lung_sound_mute != shmData->respiration.left_lung_sound_mute ) ||
		 ( current.left_lung_sound_volume != shmData->respiration.left_lung_sound_volume ) ||
		 ( current.leftLungStrength != shmData->auscultation.leftLungStrength ) )
	{
		current.left_lung_sound_mute = shmData->respiration.left_lung_sound_mute;
		current.left_lung_sound_volume = shmData->respiration.left_lung_sound_volume;
		current.leftLungStrength = shmData->auscultation.leftLungStrength;
		//if ( current.left_lung_sound_mute )
		//{
		//	current.leftLungGain = MIN_VOLUME;
		//}
		//else
		{
			current.leftLungGain = volumeToGain(current.left_lung_sound_volume, current.leftLungStrength );
		}
	}
	
	if ( force || ( gain != current.leftLungGain ) )
	{
		if ( shmData->auscultation.side != 2 )
		{
			wav.trackGain(inhL, current.leftLungGain );
		}
	}
}
void
setRightLungVolume(int force )
{
	int gain = current.rightLungGain;
	
	if ( force ||
		 ( current.right_lung_sound_mute != shmData->respiration.right_lung_sound_mute ) ||
		 ( current.right_lung_sound_volume != shmData->respiration.right_lung_sound_volume ) ||
		 ( current.rightLungStrength != shmData->auscultation.rightLungStrength ) )
	{
		current.left_lung_sound_mute = shmData->respiration.left_lung_sound_mute;
		current.right_lung_sound_volume = shmData->respiration.right_lung_sound_volume;
		current.rightLungStrength = shmData->auscultation.rightLungStrength;
		//if ( current.right_lung_sound_mute )
		//{
		//	current.rightLungGain = MIN_VOLUME;
		//}
		//else
		{
			current.rightLungGain = volumeToGain(current.right_lung_sound_volume, current.rightLungStrength );
		}
	}
	if ( force || ( gain != current.rightLungGain ) )
	{
		if ( shmData->auscultation.side != 1 )
		{
			wav.trackGain(inhR, current.rightLungGain );
		}
	}
}

void 
runHeart ( void )
{
	struct itimerspec its;
	
	if ( shmData->auscultation.side != 0 && shmData->cardiac.pea == 0 )
	{
		setHeartVolume(1 );	// Force volume setting as the track may have changed
	}
	else
	{
		setHeartVolume(1 );	// Set volume only if a change occurred
	}
	switch ( heartState )
	{
		case 0:
			if ( heartLast != current.heartCount )
			{
				heartLast = current.heartCount;
#ifdef USE_BBBGPIO
//				gp->setValue(pulsePin, TURN_ON );
#else
//				gpioPinSet(pulsePin, TURN_ON );
#endif
				//if ( shmData->auscultation.side != 0 )
				//{
					its.it_interval.tv_sec = 0;
					its.it_interval.tv_nsec = 0;
					
					// Set First expiration as the interval plus the delay
					its.it_value.tv_sec = 0;
					its.it_value.tv_nsec = LUB_DELAY;
					if (timer_settime(heart_timer, 0, &its, NULL) == -1)
					{
						perror("runHeart: timer_settime");
						//sprintf(msgbuf, "runHeart: timer_settime: %s", strerror(errno) );
						//log_message("", msgbuf );
#ifdef USE_BBBGPIO
						gp->setValue(tankPin, TURN_OFF );
#else
						gpioPinSet(tankPin, TURN_OFF );
#endif
						exit ( -1 );
					}
				//}
			}
			break;
		case 1:
			if ( shmData->cardiac.pea == 0 )
			{
				//if ( shmData->auscultation.side != 0 )
				//{
#ifdef USE_BBBGPIO
//					gp->setValue(pulsePin, TURN_OFF );
#else
//					gpioPinSet(pulsePin, TURN_OFF );
#endif
					wav.trackPlayPoly(0, lubdub);
					//sprintf(msgbuf, "runHeart: lub (%d) Gain is %d", lub, heartGain );
					//log_message("", msgbuf );
					heartState = 0;
					heartPlaying = 1;
				//}
				//else
				//{
				//	heartState = 0;
				//	heartPlaying = 0;
				//}
				
				// Check pulse palpation
				doPulse();
			}
			break;
			
		default:
			break;
	}
}

static void
delay_handler(int sig, siginfo_t *si, void *uc)
{
	if ( sig == HEART_TIMER_SIG )
	{
		if ( heartState == 0 )
		{
			heartState = 1;
		}
		else if ( heartState == 2 )
		{
			heartState = 3;
		}
	}
	else if ( sig == BREATH_TIMER_SIG )
	{
		if ( lungState == 0 )
		{
			lungState = 1;
		}
	}	
}
#define EXH_LIMIT 400
int exhLimit = EXH_LIMIT;

static void
rise_handler(int sig, siginfo_t *si, void *uc)
{
	if ( shmData->respiration.chest_movement )
	{
		// Stop rise
	#ifdef USE_BBBGPIO
		gp->setValue(riseLPin, TURN_OFF );
		gp->setValue(riseRPin, TURN_OFF );
	#else
		gpioPinSet(riseLPin, TURN_OFF );
		gpioPinSet(riseRPin, TURN_OFF );
	#endif
		riseOnOff = 0;
		usleep(10000);	// Delay 10 MSEC before fall
	#ifdef USE_BBBGPIO
		gp->setValue(fallPin, TURN_ON );
	#else
		gpioPinSet(fallPin, TURN_ON );
	#endif
		fallOnOff = 1;
	}
	else
	{
	#ifdef USE_BBBGPIO
		gp->setValue(riseLPin, TURN_OFF );
		gp->setValue(riseRPin, TURN_OFF );
		gp->setValue(fallPin, TURN_OFF );
	#else
		gpioPinSet(riseLPin, TURN_OFF );
		gpioPinSet(riseRPin, TURN_OFF );
		gpioPinSet(fallPin, TURN_OFF );
	#endif
		riseOnOff = 0;
		fallOnOff = 0;
	}
	exhLimit = EXH_LIMIT;
}

void
initialize_timers(void )
{
	struct sigaction new_action;
	sigset_t mask;
	
	printf("\n\ninitialize_timers\n\n" );
	// Pulse Timer Setup
	new_action.sa_flags = SA_SIGINFO;
	new_action.sa_sigaction = delay_handler;
	sigemptyset(&new_action.sa_mask);
	if (sigaction(HEART_TIMER_SIG, &new_action, NULL) == -1)
	{
		perror("sigaction");
		sprintf(msgbuf, "sigaction() fails for Pulse Timer: %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	// Block timer signal temporarily
	sigemptyset(&mask);
	sigaddset(&mask, HEART_TIMER_SIG);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
	{
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Pulse Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	// Create the Timer
	heart_sev.sigev_notify = SIGEV_SIGNAL;
	heart_sev.sigev_signo = HEART_TIMER_SIG;
	heart_sev.sigev_value.sival_ptr = &heart_timer;
	
	if ( timer_create(CLOCK_MONOTONIC, &heart_sev, &heart_timer ) == -1 )
	{
		perror("timer_create" );
		sprintf(msgbuf, "timer_create() fails for Pulse Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit (-1);
	}
    if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
    {
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Pulse Timer%s ", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	
	// Breath Timer Setup
	new_action.sa_flags = SA_SIGINFO;
	new_action.sa_sigaction = delay_handler;
	sigemptyset(&new_action.sa_mask);
	if (sigaction(BREATH_TIMER_SIG, &new_action, NULL) == -1)
	{
		perror("sigaction");
		sprintf(msgbuf, "sigaction() fails for Breath Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit(-1 );
	}
	// Block timer signal temporarily
	sigemptyset(&mask);
	sigaddset(&mask, BREATH_TIMER_SIG);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
	{
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Breath Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	// Create the Timer
	breath_sev.sigev_notify = SIGEV_SIGNAL;
	breath_sev.sigev_signo = BREATH_TIMER_SIG;
	breath_sev.sigev_value.sival_ptr = &breath_timer;
	
	if ( timer_create(CLOCK_MONOTONIC, &breath_sev, &breath_timer ) == -1 )
	{
		perror("timer_create" );
		sprintf(msgbuf, "timer_create() fails for Breath Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit (-1);
	}
	
    if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
    {
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Breath Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	
	// Rise Timer Setup
	new_action.sa_flags = SA_SIGINFO;
	new_action.sa_sigaction = rise_handler;
	sigemptyset(&new_action.sa_mask);
	if (sigaction(RISE_TIMER_SIG, &new_action, NULL) == -1)
	{
		perror("sigaction");
		sprintf(msgbuf, "sigaction() fails for Rise Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit(-1 );
	}
	// Block timer signal temporarily
	sigemptyset(&mask);
	sigaddset(&mask, RISE_TIMER_SIG);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
	{
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Rise Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
	// Create the Timer
	rise_sev.sigev_notify = SIGEV_SIGNAL;
	rise_sev.sigev_signo = RISE_TIMER_SIG;
	rise_sev.sigev_value.sival_ptr = &rise_timer;
	
	if ( timer_create(CLOCK_MONOTONIC, &rise_sev, &rise_timer ) == -1 )
	{
		perror("timer_create" );
		sprintf(msgbuf, "timer_create() fails for Rise Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit (-1);
	}
	
    if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
    {
		perror("sigprocmask");
		sprintf(msgbuf, "sigprocmask() fails for Rise Timer %s", strerror(errno) );
		log_message("", msgbuf );
#ifdef USE_BBBGPIO
		gp->setValue(tankPin, TURN_OFF );
#else
		gpioPinSet(tankPin, TURN_OFF );
#endif
		exit ( -1 );
	}
}

/* Lung State:
	0 - Idle. Waiting for Sync. When Sync Received:
		Start Inflation. 
		Start timer to end inflation.
		Start 40 ms timer to wait for sound
	1 - Play Sound
	
*/
#define FLIP_LUNG	1
void 
runLung( void )
{
	struct itimerspec its;
	long int delayTime;	// Delay in ns
	double periodSeconds;
	double fractional;
	double integer;
	
	if ( ! shmData->respiration.chest_movement )
	{
		allAirOff();
	}

	if ( shmData->auscultation.side != 0  )
	{
		current.respiration_rate = shmData->respiration.rate;
		setLeftLungVolume(1 );	// Force volume setting as the track may have changed
		setRightLungVolume(1 );	// Force volume setting as the track may have changed
	}
	else
	{
		setLeftLungVolume(0 );	// Set volume only if a change occurred
		setRightLungVolume(0 );	// Set volume only if a change occurred
	}
	switch ( lungState )
	{
		case 0:
			
			if ( lungLast != current.breathCount )
			{
				lungLast = current.breathCount;
			// Breath Timer
				// Clear the interval, run single execution
				its.it_interval.tv_sec = 0;
				its.it_interval.tv_nsec = 0;
				
				// Set First expiration as the interval plus the delay
				its.it_value.tv_sec = 0;
				delayTime = 40000000;	// Delay in ns
				its.it_value.tv_nsec = delayTime;
				if (timer_settime(breath_timer, 0, &its, NULL) == -1)
				{
					perror("runLung: timer_settime");
					sprintf(msgbuf, "runLung: timer_settime: %s", strerror(errno) );
					log_message("", msgbuf );
#ifdef USE_BBBGPIO
					gp->setValue(tankPin, TURN_OFF );
#else
					gpioPinSet(tankPin, TURN_OFF );
#endif
					exit ( -1 );
				}
#ifdef USE_BBBGPIO
				gp->setValue(fallPin, TURN_OFF );
				fallOnOff = 0;
				usleep(10000);
				if ( shmData->respiration.chest_movement )
				{
					if ( debug ) printf("ON\n" );
					gp->setValue(riseLPin, TURN_ON );
					gp->setValue(riseRPin, TURN_ON );
				}
#else
				gpioPinSet(fallPin, TURN_OFF );
				fallOnOff = 0;
				usleep(10000);
				if ( shmData->respiration.chest_movement )
				{
					if ( debug ) printf("ON\n" );
					gpioPinSet(riseLPin, TURN_ON );
					gpioPinSet(riseRPin, TURN_ON );
				}
#endif
				riseOnOff = 1;
			// Rise Timer
				// Clear the interval, run single execution
				its.it_interval.tv_sec = 0;
				its.it_interval.tv_nsec = 0;
				
				// The duration should be 30% of the respiration period
#define INH_PERCENT		(0.30)
				periodSeconds = ( 1 / (double)shmData->respiration.rate ) * 60;
				periodSeconds *= INH_PERCENT;
				if ( periodSeconds < 0 )
				{
					sprintf(msgbuf, "runLung: rise periodSeconds is negative period %f rate %d", periodSeconds, shmData->respiration.rate );
					periodSeconds = 1;
					log_message("", msgbuf );
				}
				fractional = modf(periodSeconds, &integer );
				its.it_value.tv_sec = (int)integer;
				delayTime = fractional * 1000000000; //Seconds to nanoseconds

				if ( delayTime < 0 )
				{
					sprintf(msgbuf, "runLung: rise delayTime is negative for period %f rate %d", periodSeconds, shmData->respiration.rate );
					delayTime = 0;
					log_message("", msgbuf );
				}
				its.it_value.tv_nsec = delayTime;
				
				/*
				riseTime = ((int)integer * 1000 ) + ( delayTime /1000/1000 );  // In msec

				if ( riseTime < 0 )
				{
					sprintf(msgbuf, "runLung: rise riseTime is negative for period %f rate %d", periodSeconds, shmData->respiration.rate );
					riseTime = 0;
					log_message("", msgbuf );
				}
				*/
				if (timer_settime(rise_timer, 0, &its, NULL) == -1)
				{
					//perror("runLung: rise timer_settime");
					sprintf(msgbuf, "runLung: rise timer_settime: %s", strerror(errno) );
					log_message("", msgbuf );
					sprintf(msgbuf, "runLung: periodSeconds %f, (%ld : %ld)", periodSeconds, its.it_value.tv_sec, its.it_value.tv_nsec );
					log_message("", msgbuf );
#ifdef USE_BBBGPIO
					gp->setValue(tankPin, TURN_OFF );
#else
					gpioPinSet(tankPin, TURN_OFF );
#endif
					exit ( -1 );
				}
			}
			else if ( current.respiration_rate == 0 )
			{
				if ( exhLimit-- == 0 )
				{
					if ( debug ) printf("OFF\n" );
#ifdef USE_BBBGPIO
					gp->setValue(fallPin, TURN_OFF );
					fallOnOff = 0;
					gp->setValue(riseLPin, TURN_OFF );
					gp->setValue(riseRPin, TURN_OFF );
#else
					gpioPinSet(tankPin, TURN_OFF );
					fallOnOff = 0;
					gpioPinSet(riseLPin, TURN_OFF );
					gpioPinSet(riseRPin, TURN_OFF );
#endif
					sprintf(msgbuf, "runLung: exhLimit Hit" );
					log_message("", msgbuf );
				}
			}
			break;
		case 1:
			if ( shmData->auscultation.side != 0 )
			{
				if ( shmData->auscultation.side == 1 )
				{
					wav.trackPlayPoly(0, inhL);
				}
				else
				{
					wav.trackPlayPoly(0, inhR);
				}

				if ( debug > 1 )
				{
					printf("inh: %d-%d\n", inhR, inhL );
				}
				lungState = 0;
				lungPlaying = 1;
			}
			else
			{
				lungState = 0;
				lungPlaying = 0;
			}
			break;
#if 0
		case 2: // No longer used
			if ( shmData->auscultation.side == 0 )
			{
				wav.trackStop(inh );
				lungState = 0;
				lungPlaying = 0;
			}
			else
			{
				// INH playing. Check for completion
				if ( wav.checkTrack(inh ) == 0 )
				{
					// inh done
					lungState = 0;
					lungPlaying = 0;
				}
			}
			break;
#endif	
		default:
			break;
	}
}

int getPulseVolume(int pressure, int strength )
{
	int pulseVolume;
	
	switch ( pressure )
	{
		case PULSE_TOUCH_NONE:
		default:
			pulseVolume = PULSE_VOLUME_OFF;
			break;
		case PULSE_TOUCH_EXCESSIVE:
			pulseVolume = PULSE_VOLUME_VERY_SOFT;
			break;
		case PULSE_TOUCH_HEAVY:
			pulseVolume = PULSE_VOLUME_SOFT;
			break;
		case PULSE_TOUCH_NORMAL:
			pulseVolume = PULSE_VOLUME_ON;
			break;
		case PULSE_TOUCH_LIGHT:
			pulseVolume = PULSE_VOLUME_SOFT;
			break;
	}
	switch ( strength )
	{
		case 0: // None
			pulseVolume = PULSE_VOLUME_OFF;
			break;
		case 1: // Weak
			pulseVolume -= 10;
			break;
		case 2: // Normal
			break;
		case 3: // Strong
		default:
			pulseVolume += 10;
			break;
	}
	return ( pulseVolume );
}

void doPulse(void )
{
	int pulseVolume;
	
	if ( shmData->cardiac.pea )
	{
		return;
	}
	
	if ( shmData->pulse.right_dorsal && shmData->cardiac.right_dorsal_pulse_strength > 0 )
	{
			pulseVolume = getPulseVolume(shmData->pulse.right_dorsal, shmData->cardiac.right_dorsal_pulse_strength );
			pulseVolume = pulseVolume - 25;
			shmData->pulse.volume[PULSE_RIGHT_DORSAL] = pulseVolume;
			wav.channelGain(5, pulseVolume );
			wav.trackPlayPoly(5, PULSE_TRACK);
	}
	else
	{
		wav.channelGain(5, PULSE_VOLUME_OFF );
		shmData->pulse.volume[PULSE_RIGHT_DORSAL] = PULSE_VOLUME_OFF;
	}
	if ( shmData->pulse.left_dorsal && shmData->cardiac.left_dorsal_pulse_strength > 0 )
	{
			pulseVolume = getPulseVolume(shmData->pulse.left_dorsal, shmData->cardiac.left_dorsal_pulse_strength );
			pulseVolume = pulseVolume - 25;
			shmData->pulse.volume[PULSE_LEFT_DORSAL] = pulseVolume;
			wav.channelGain(4, pulseVolume );
			wav.trackPlayPoly(4, PULSE_TRACK);
	}
	else
	{
		wav.channelGain(4, PULSE_VOLUME_OFF );
		shmData->pulse.volume[PULSE_LEFT_DORSAL] = PULSE_VOLUME_OFF;
	}
	if ( shmData->pulse.right_femoral && shmData->cardiac.right_femoral_pulse_strength > 0 )
	{
			pulseVolume = getPulseVolume(shmData->pulse.right_femoral, shmData->cardiac.right_femoral_pulse_strength );
			pulseVolume = pulseVolume - 25;
			shmData->pulse.volume[PULSE_RIGHT_FEMORAL] = pulseVolume;
			wav.channelGain(3, pulseVolume );
			wav.trackPlayPoly(3, PULSE_TRACK);
	}
	else
	{
		wav.channelGain(3, PULSE_VOLUME_OFF );
		shmData->pulse.volume[PULSE_RIGHT_FEMORAL] = PULSE_VOLUME_OFF;
	}
	if ( shmData->pulse.left_femoral && shmData->cardiac.left_femoral_pulse_strength > 0 )
	{
			pulseVolume = getPulseVolume(shmData->pulse.left_femoral, shmData->cardiac.left_femoral_pulse_strength );
			pulseVolume = pulseVolume - 25;
			shmData->pulse.volume[PULSE_LEFT_FEMORAL] = pulseVolume;
			wav.channelGain(2, pulseVolume );
			wav.trackPlayPoly(2, PULSE_TRACK);
	}
	else
	{
		wav.channelGain(2, PULSE_VOLUME_OFF );
		shmData->pulse.volume[PULSE_LEFT_FEMORAL] = PULSE_VOLUME_OFF;
	}
	/*
	switch ( shmData->pulse.position )
	{
		case PULSE_NOT_ACTIVE:
		default:
			pulseChannel = 0;
			break;
			
		case PULSE_LEFT_FEMORAL:
			pulseChannel = 2;
			pulseStrength = shmData->cardiac.left_femoral_pulse_strength;
			break;
		
		case PULSE_RIGHT_FEMORAL:
			pulseChannel = 3;
			pulseStrength = shmData->cardiac.right_femoral_pulse_strength;
			break;
		
		case PULSE_LEFT_DORSAL:
			pulseChannel = 4;
			pulseStrength = shmData->cardiac.left_dorsal_pulse_strength;
			break;
		
		case PULSE_RIGHT_DORSAL:
			pulseChannel = 5;
			pulseStrength = shmData->cardiac.right_dorsal_pulse_strength;
			break;
	}
		
	if ( pulseChannel > 0 )
	{
		pulseVolume = PULSE_VOLUME_ON;
	
		switch ( shmData->pulse.pressure )
		{
			case PULSE_TOUCH_NONE:
				pulseVolume = PULSE_VOLUME_OFF;
				break;
			case PULSE_TOUCH_EXCESSIVE:
				pulseVolume = PULSE_VOLUME_VERY_SOFT;
				break;
			case PULSE_TOUCH_HEAVY:
				pulseVolume = PULSE_VOLUME_SOFT;
				break;
			case PULSE_TOUCH_NORMAL:
				pulseVolume = PULSE_VOLUME_ON;
				break;
			case PULSE_TOUCH_LIGHT:
				pulseVolume = PULSE_VOLUME_SOFT;
				break;
		}
		switch ( pulseStrength )
		{
			case 0: // None
				pulseVolume = PULSE_VOLUME_OFF;
				break;
			case 1: // Very Weak
				pulseVolume -= 20;
				break;
			case 2: // Weak
				pulseVolume -= 10;
				break;
			case 3: // Strong
			default:
				break;
		}
		
		if ( debug )
		{
			sprintf(msgbuf, "Pulse: Channel %d Strength %d Volume %d", 
				pulseChannel,
				pulseStrength,
				pulseVolume );
			if ( debug > 1 )
			{
				printf("%s\n", msgbuf );
			}
			else
			{
				log_message("", msgbuf);
			}
		}
		if ( pulseVolume > PULSE_VOLUME_OFF )
		{
			wav.channelGain(pulseChannel, pulseVolume );
			wav.trackPlayPoly(pulseChannel, PULSE_TRACK);
		}
	}
	*/
}

char lastTag[STR_SIZE];

void
runMonitor(void )
{
	
	while ( 1 )
	{
		if ( debug > 1 )
		{
			if ( strcmp(shmData->auscultation.tag, lastTag ) != 0 )
			{
				memcpy(lastTag, shmData->auscultation.tag, STR_SIZE );
				printf("Tag %s\n", lastTag );
			}
		}
		else
		{
			printf( "sense %d:%d:%d:%d  %d:%d:%d:%d  %d:%d:%d:%d  %d:%d:%d:%d  Tag: '%s', %d/%d %s\n", 
					shmData->pulse.base[1], shmData->pulse.ain[1], shmData->pulse.touch[1], shmData->pulse.volume[1],
					shmData->pulse.base[2], shmData->pulse.ain[2], shmData->pulse.touch[2], shmData->pulse.volume[2],
					shmData->pulse.base[3], shmData->pulse.ain[3], shmData->pulse.touch[3], shmData->pulse.volume[3],
					shmData->pulse.base[4], shmData->pulse.ain[4], shmData->pulse.touch[4], shmData->pulse.volume[4], 
					shmData->auscultation.tag, 
					shmData->manual_breath_ain, shmData->manual_breath_baseline, shmData->respiration.manual_breath ? " - Breath" : "" );
		}
		usleep(500000 );
	}
}
