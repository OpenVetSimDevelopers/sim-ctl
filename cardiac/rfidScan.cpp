/* 
 * rfidScan.cpp
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

/*
 * Supports the ID-Innovations devices:
	ID-2LA, ID-12LA, ID-20LA
		These devices use a serial IO sequence:
			STX			02h
			Data		10 byte ASCII sequence example: "0C000621a58E"
			Checksum	2 bytes ASCII
			CR			13h
			LF			10h
			ETX			03h
			
 * Supports the simpler Seeed-Studio device. This device sends
	5 Hex values as the serial data.
			Start		00h
			Data		3 bytes ID
			Checksum	1 byte checksum
			
*/

#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <sys/ipc.h>
#include <sys/msg.h>
#include <time.h>
#include <errno.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <termios.h>
#include <syslog.h>
#include <signal.h>
#include <string.h>
#include <stdint.h>

#include <libxml/xmlreader.h>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

#ifndef LIBXML_READER_ENABLED
	Error LIBXML is not included or Reader is not Enabled
#endif
#ifdef USE_BBBGPIO
	#include <GPIO/GPIOManager.h>
	#include <GPIO/GPIOConst.h>
#else

#endif

#include "rfidScan.h"

#include "../comm/shmData.h"
#include "../comm/simUtil.h"

/*
#include "../comm/simCtlComm.h"
simCtlComm comm(SYNC_PORT );
*/


#define SCAN_CONFIG "/simulator/rfid.xml"
#define PARSE_STATE_NONE	0
#define PARSE_STATE_TAG		1
#define TAG_BUF_LEN 100

using namespace std;

struct shmData *shmData;

struct rfidData *rfidData;

int tagCheck(uint64_t newid);
int tagParse(const char *elem,  const char *value, struct rfidTag *tag );
static void startParseState(int lvl, char *name );
static void saveData(const xmlChar *xmlName, const xmlChar *xmlValue );
static int readConfig(const char *filename);

int kbhit(int file);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
void cleanString(char *strIn );
void catchFaults(void );

struct xml_level xmlLevels[10];
int xml_current_level = 0;
int line_number = 0;
const char *xml_filename;

int parse_state = PARSE_STATE_NONE;
int verbose = 0;
char msgbuf[2048];

char tagBuffer[TAG_BUF_LEN];

const char *parse_states[] =
{
	"PARSE_STATE_NONE", "PARSE_STATE_TAG"
};
int parseTagNum = -1;

int debug = 0;

struct stat configStat;
#define LOOP_SLEEP_MS	10
#define LOOP_SLEEP_US	(LOOP_SLEEP_MS*1000)
#define LOOPS_PER_SEC	(1000/LOOP_SLEEP_MS)
#define LOOPS_PER_10SEC	(10*LOOPS_PER_SEC)

void
ttyPurge(int ttyfd )
{
	
}

int checkBitValidationSEED(char *data)
{
	if( data[4] == ( data[0]^data[1]^data[2]^data[3] ) )
	{
		return 1;
	} else
	{
		return 0;
	}
}

unsigned long cardNumberSEEED(char *data)
{
	unsigned long sum = 0;
	unsigned long sum2 = 0;
	
	if ( 0 != data[0] ) 
	{
		sum = sum + data[0];
		sum = sum<<24;
	}
	sum = sum + data[1];
	sum = sum << 16;

	sum2 = sum2  + data[2];
	sum2 = sum2 <<8;
	sum2 = sum2  + data[3];

	sum = sum + sum2;

    return sum;
}

int main(int argc, char *argv[])
{
	int sts;
	int i;
	uint64_t newid;
	uint32_t tagIndex;
	struct termios tty;
	int count;
	int state;
	int detect;
	struct stat statCheck;
	int lcount;
	int ttyfd = -1;
	
	if ( argc > 1 )
	{
		if ( strncmp("-d", argv[1], 2 ) == 0 )
		{
			debug = 1;
		}
	}
	if ( debug ) 
	{
		printf("Debug (%d)\n", debug );
	}
	if ( ! debug )
	{
		daemonize();
	}
	else
	{
		catchFaults();
	}
	sts = initSHM(SHM_OPEN );
	
	if ( sts )
	{
		sprintf(msgbuf, "SHM Failed (%d) - Exiting", sts );
		if ( debug )
		{
			printf("%s\n", msgbuf );
		}
		log_message("", msgbuf );
		exit ( -1 );
	}
	rfidData = (struct rfidData *)calloc(sizeof(struct rfidData ), 1 );
	if ( ! rfidData )
	{
		sprintf(msgbuf, "calloc Failed - Exiting" );
		if ( debug )
		{
			printf("%s\n", msgbuf );
		}
		log_message("", msgbuf );
		exit ( -1 );
	}
	if ( debug ) 
	{
		printf("Reading Config\n" );
	}
	// Read the configuration file to find the RFID tags
	readConfig(SCAN_CONFIG );

	// Monitor the Tag Detected signal from the reader. When it goes high, we wait on 
	// a mesage from the serial port.
	// we wait on a serial port for the sensor to be reported.
#ifdef USE_BBBGPIO
	int detectPin;

	GPIO::GPIOManager* gp = GPIO::GPIOManager::getInstance();
	detectPin = GPIO::GPIOConst::getInstance()->getGpioByKey("P9_23" );
	gp->exportPin(detectPin );
	gp->setDirection(detectPin, GPIO::INPUT );
#else
	FILE *detectPin;

	detectPin = gpioPinOpen(49, GPIO_INPUT );	// P9_23
	fclose(detectPin );
#endif
	
  
	
	// Serial port used to read from RFID sensor
	const char *portname = "/dev/ttyO1";
	while ( ttyfd < 0 )
	{
		ttyfd = open (portname, O_RDWR | O_NOCTTY | O_NONBLOCK );
		if (ttyfd < 0)
		{
			sprintf(msgbuf, "error %d opening %s: %s", errno, portname, strerror (errno));
			if ( debug )
			{
				printf("%s\n", msgbuf );
			}
			log_message("", msgbuf );
			sleep(10 );
		}
		
	}
	cfsetospeed (&tty, B9600);
	cfsetispeed (&tty, B9600);
	if (tcgetattr (ttyfd, &tty) < 0)
	{
        sprintf(msgbuf, "error %d tcgetattr %s: %s", errno, portname, strerror (errno));
		if ( debug )
		{
			printf("%s\n", msgbuf );
		}
		log_message("", msgbuf );
		return EXIT_FAILURE;
    }
    tty.c_cflag |= (CLOCAL | CREAD | IGNPAR | CS8 );
	tty.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
    tty.c_iflag |= IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_lflag &= ~(ICANON | ECHO);
    tty.c_oflag  = 0;
	tty.c_cc[VMIN]=1;
	tty.c_cc[VTIME]=1;
	
	/* Make raw */
	cfmakeraw(&tty);

    tcflush (ttyfd, TCIOFLUSH);
    if (tcsetattr (ttyfd, TCSANOW, &tty) )
	{
        sprintf(msgbuf, "error %d tcsetattr %s: %s", errno, portname, strerror (errno));
		if ( debug )
		{
			printf("%s\n", msgbuf );
		}
		log_message("", msgbuf );
		return EXIT_FAILURE;
    }
	
	state = 0;
	rfidData->tagDetected = 0;
	shmData->auscultation.side = 0;
	lcount = 0;

#ifdef USE_BBBGPIO
	detect = gp->getValue(detectPin );
#else
	gpioPinRead(49, &detect );

#endif
	
	sprintf(msgbuf, "Detect Check %d", detect );
	log_message("", msgbuf);
	if ( debug )
	{
		printf("%s\n", msgbuf );
	}
	
	while ( 1 )
	{
		if ( lcount++ >= LOOPS_PER_10SEC )
		{
			sts = stat(SCAN_CONFIG, &statCheck );
			if ( statCheck.st_mtime != configStat.st_mtime )
			{
				readConfig(SCAN_CONFIG );
			}
			lcount = 0;
		}

#ifdef USE_BBBGPIO
		detect = gp->getValue(detectPin );
#else
		gpioPinRead(49, &detect );
#endif

		switch ( state )
		{
			case 0: // Waiting for detect
				if ( detect )
				{
					state = 2;
					count = 0;
					tagBuffer[0] = 0;
					sprintf(msgbuf, "Detect %d : State 2", detect );
					if ( verbose )
					{
						log_message("", msgbuf);
					}
					if ( debug )
					{
						printf("%s\n", msgbuf );
					}
					
				}
				else
				{
					shmData->auscultation.side = 0;
				}
				break;

			case 2: // Detect Received, Reading string from reader
				if ( ! detect )
				{
					if ( rfidData->tagDetected == 1 )
					{
						if ( debug )
						{
							printf("End\n" );
						}
						if ( verbose )
						{
							sprintf(msgbuf, "End (2)" );
							log_message("", msgbuf);
						}
						rfidData->tagDetected = 0;						
					}
					shmData->auscultation.side = 0;
					if ( verbose )
					{
						sprintf(msgbuf, "Detect  0 State 2 to 0 Count %d", count );
						log_message("", msgbuf);
					}
					state = 0;
					count = 0;
				}
				else
				{
					if ( count > TAG_BUF_LEN )
					{
						state = 0;
						count = 0;
					}
					else
					{
						sts = read(ttyfd, &tagBuffer[count], TAG_BUF_LEN - count );
						if ( sts > 0 )
						{
							count += sts;
							if ( count == 5 )
							{
								// Test for valid data from SEED reader
								if ( checkBitValidationSEED(tagBuffer ) )
								{
									newid = cardNumberSEEED(tagBuffer );
									rfidData->tagDetected = 1;
									
									tagIndex = tagCheck(newid );
									sprintf(msgbuf, "Tag %lld - %d", newid, tagIndex );
									log_message("", msgbuf);
									if ( debug )
									{
										printf(" Tag %lld - %d\n", newid, tagIndex );
									}
									sprintf(shmData->auscultation.tag, "%lld", newid );
									state = 3;
									count = 0;
								}
							}
							if ( tagBuffer[count-1] == 0x03 )
							{
								// Full tag received - Report and continue looking for STX
								if ( count >= 13 )
								{
									// Find the tagID in the table and set it as active
									rfidData->tagDetected = 1;
									newid = 0;

									for ( i = 1 ; i < 15 ; i++ )
									{
										if ( debug )
										{
											if ( isprint(tagBuffer[i] ) )
											{
												printf("%c", tagBuffer[i] );
											}
											else 
											{
												switch ( tagBuffer[i] )
												{
													case 0x0A:
													case 0x0D:
													case 0x03:
														break;
													default:
														printf(" %02xh", tagBuffer[i] );
														break;
												}
											}

											if ( i == 9 )
											{
												printf(" : " );
											}
										}
										if ( ( i > 2 ) && ( i < 11 ) )
										{
											if ( ( tagBuffer[i] >= '0' ) && ( tagBuffer[i] <= '9' ) )
											{
												newid = ( newid << 4 ) + ( tagBuffer[i] - '0' );
											}
											else if ( ( tagBuffer[i] >= 'A' ) && ( tagBuffer[i] <= 'F' ) )
											{
												newid = ( newid << 4 ) + (( tagBuffer[i] - 'A' ) + 10 );	
											}
										}
									}
									tagIndex = tagCheck(newid );
									sprintf(msgbuf, "Tag %lld - %d", newid, tagIndex );
									log_message("", msgbuf);
									if ( debug )
									{
										printf(" Tag %lld - %d\n", newid, tagIndex );
									}
									sprintf(shmData->auscultation.tag, "%lld", newid );
									
									state = 3;
									count = 0;
								}
							}
						}
					}
				}
				break;

			
			case 3: // Wait for loss of detect
				if ( ! detect )
				{
					if ( rfidData->tagDetected == 1 )
					{
						if ( debug )
						{
							printf("End\n" );
						}
						if ( verbose )
						{
							sprintf(msgbuf, "End (3)" );
							log_message("", msgbuf);
						}
						rfidData->tagDetected = 0;						
					}
					shmData->auscultation.side = 0;
					if ( verbose )
					{
						sprintf(msgbuf, "Detect 0 State 3 to 0" );
						log_message("", msgbuf);
					}
					state = 0;
					count = 0;
				}
				else
				{
					// Just to purge any extra characters
					sts = read(ttyfd, &tagBuffer[0], TAG_BUF_LEN );
				}
		}
		usleep(LOOP_SLEEP_US); // 10 ms delay between checks.
	}

	return 0;
}

int
tagCheck(uint64_t newid)
{
	unsigned int tagIndex;
	
	for ( tagIndex = 0 ; tagIndex < rfidData->tagCount ; tagIndex++ )
	{
		if ( newid == rfidData->tags[tagIndex].tagId )
		{
			//shmData->cardiac.heart_sound_volume = rfidData->tags[tagIndex].heartStrength;
			//shmData->cardiac.heart_sound_mute   = 0;
			
			//shmData->respiration.left_lung_sound_volume = rfidData->tags[tagIndex].leftLungStrength;
			//shmData->respiration.left_lung_sound_mute   = 0;
			
			//shmData->respiration.right_lung_sound_volume = rfidData->tags[tagIndex].rightLungStrength;
			//shmData->respiration.right_lung_sound_mute   = 0;
			
			shmData->auscultation.col  = rfidData->tags[tagIndex].xPosition;
			shmData->auscultation.row  = rfidData->tags[tagIndex].yPosition;
			shmData->auscultation.side = rfidData->tags[tagIndex].side;
			shmData->auscultation.heartStrength = rfidData->tags[tagIndex].heartStrength;
			shmData->auscultation.leftLungStrength = rfidData->tags[tagIndex].leftLungStrength;
			shmData->auscultation.rightLungStrength = rfidData->tags[tagIndex].rightLungStrength;
			return ( tagIndex );
		}
	}
	// Tag not found
	shmData->auscultation.side = 0;
	return ( -1 );
}

/* 
 * FUNCTION: tagParse
 *
 * Called from XML parser to save a defined tag
*/
	
int
tagParse(const char *elem,  const char *value, struct rfidTag *tag )
{
	int sts = 0;
	
	if ( ( ! elem ) || ( ! value) || ( ! tag ) )
	{
		return ( -11 );
	}
/*
	uint64_t id;			// 40-bit ID of the tag
	int side;				// 0 is left side, 1 is right side
	int heartStrength;		// Strength of Heart sound
	int leftLungStrength;	// Strength of Left Lung sound
	int rightLungStrength;	// Strength of Right Lung sound
	int xPosition;
	int yPosition;
*/
	if ( strcmp(elem, ("tagId" ) ) == 0 )
	{
		tag->tagId = atoll(value);
	}
	else if ( strcmp(elem, ("side" ) ) == 0 )
	{
		tag->side = atoi(value );
	}
	else if ( strcmp(elem, ("heartStrength" ) ) == 0 )
	{
		tag->heartStrength = atoi(value );
	}
	else if ( strcmp(elem, ("leftLungStrength" ) ) == 0 )
	{
		tag->leftLungStrength = atoi(value );
	}
	else if ( strcmp(elem, ("rightLungStrength" ) ) == 0 )
	{
		tag->rightLungStrength = atoi(value );
	}
	else if ( strcmp(elem, ("rightLungStrength" ) ) == 0 )
	{
		tag->rightLungStrength = atoi(value );
	}
	else if ( strcmp(elem, ("xPosition" ) ) == 0 )
	{
		tag->xPosition = atoi(value );
	}
	else if ( strcmp(elem, ("yPosition" ) ) == 0 )
	{
		tag->yPosition = atoi(value );
	}
	else
	{
		sts = 1;
	}
	return ( sts );
}

/**
 * startParseState:
 * @lvl: New level from XML
 * @name: Name of the new level
 *
 * Process level change.
*/

static void
startParseState(int lvl, char *name )
{
	if ( verbose )
	{
		printf("startParseState: Cur State %s: Lvl %d Name %s   ", parse_states[parse_state], lvl, name );
	}
	switch ( lvl )
	{
		case 0:		// Top level - no actions
			break;
			
		case 1:	// header has no action
			if ( strcmp(name, "tag" ) == 0 )
			{
				parse_state = PARSE_STATE_TAG;
				parseTagNum++;
			}
			break;
			
		default:
			break;
	}
	if ( verbose )
	{
		printf("New State %s: \n", parse_states[parse_state] );
		printf("\n" );
	}
}
static void
endParseState(int lvl )
{
	if ( verbose )
	{
		printf("endParseState: Lvl %d    State: %s\n", lvl,parse_states[parse_state] );
	}
	switch ( lvl )
	{
		case 0:	// Parsing Complete
			break;
			
		case 1:	// Section End
			parse_state = PARSE_STATE_NONE;
			break;
			
		default:
			break;
	}
}
/**
 * processNode:
 * @reader: the xmlReader
 *
 * Dump information about the current node
 */
static void
processNode(xmlTextReaderPtr reader)
{
    const xmlChar *name;
	const xmlChar *value;
	int lvl;
	
    name = xmlTextReaderConstName(reader);
    if ( name == NULL )
	{
		name = BAD_CAST "--";
	}
	line_number = xmlTextReaderGetParserLineNumber(reader);
	
    value = xmlTextReaderConstValue(reader);
	// Node Type Definitions in http://www.gnu.org/software/dotgnu/pnetlib-doc/System/Xml/XmlNodeType.html
	
	switch ( xmlTextReaderNodeType(reader) )
	{
		case 1: // Element
			xml_current_level = xmlTextReaderDepth(reader);
			xmlLevels[xml_current_level].num = xml_current_level;
			if ( xmlStrlen(name) >= PARAMETER_NAME_LENGTH )
			{
				fprintf(stderr, "XML Parse Error: %s[%d]: Name %s exceeds Max Length of %d\n",
					xml_filename, line_number, name, PARAMETER_NAME_LENGTH-1 );
					
			}
			else
			{
				sprintf(xmlLevels[xml_current_level].name, "%s", name );
			}
			// printf("Start %d %s\n", xml_current_level, name );
			startParseState(xml_current_level, (char *)name );
			break;
		
		case 3:	// Text
			cleanString((char *)value );
			if ( verbose )
			{
				for ( lvl = 0 ; lvl <= xml_current_level ; lvl++ )
				{
					printf("[%d]%s:", lvl, xmlLevels[lvl].name );
				}
				printf(" %s\n", value );
			}
			saveData(name, value );
			break;
			
		case 13: // Whitespace
		case 14: // Significant Whitespace 
		case 8: // Comment
			break;
		case 15: // End Element
			// printf("End %d\n", xml_current_level );
			endParseState(xml_current_level );
			xml_current_level--;
			break;
		
			
		case 0: // None
		case 2: // Attribute
		case 4: // CDATA
		case 5: // EntityReference
		case 6: // Entity
		case 7: // ProcessingInstruction
		case 9: // Document
		case 10: // DocumentType
		case 11: // DocumentTragment
		case 12: // Notation
		case 16: // EndEntity
		case 17: // XmlDeclaration
		
		default:
			if ( verbose )
			{
				printf("%d %d %s %d %d", 
					xmlTextReaderDepth(reader),
					xmlTextReaderNodeType(reader),
					name,
					xmlTextReaderIsEmptyElement(reader),
					xmlTextReaderHasValue(reader));
			
				if (value == NULL)
				{
					printf("\n");
				}
				else
				{
					if (xmlStrlen(value) > 60)
					{
						printf(" %.80s...\n", value);
					}
					else
					{
						printf(" %s\n", value);
					}
				}
			}
	}
}
/**
 * saveData:
 * @xmlName: name of the entry
 * @xmlValue: Text data to convert and save in structure
 *
 * Take the current Text entry and save as appropriate in the currently
 * named data structure
*/
static void
saveData(const xmlChar *xmlName, const xmlChar *xmlValue )
{
	// char *name = (char *)xmlName;
	char *value = (char *)xmlValue;
	int sts = 0;

	switch ( parse_state )
	{
		case PARSE_STATE_NONE:
			if ( verbose )
			{
				printf("STATE_NONE: Lvl %d Name %s, Value, %s\n",
							xml_current_level, xmlLevels[xml_current_level].name, value );
			}
			break;
			
		case PARSE_STATE_TAG:
			sts = tagParse(xmlLevels[xml_current_level].name, value, &rfidData->tags[parseTagNum] );
			break;
			
		default:
			break;
	}
	if ( sts && verbose )
	{
		printf("saveData STS %d: Lvl %d: %s, Value  %s, \n", sts, xml_current_level, xmlLevels[xml_current_level].name, value );
	}
}

/**
 * readConfig:
 * @filename: the file name to parse
 *
 * Parse the Configuration file
 */
static int
readConfig(const char *filename)
{
    xmlTextReaderPtr reader;
    int ret;
	int sts = 0;
	unsigned int tagIndex;
	
	// Save file stat for update chages later
	sts = stat(filename, &configStat );
	
	parseTagNum = -1;
	
    xmlLineNumbersDefault(1);
    
	reader = xmlReaderForFile(filename, NULL, 0);
	if ( reader != NULL )
	{
		while ( ( ret = xmlTextReaderRead(reader) ) == 1 )
		{
            processNode(reader);
        }
        xmlFreeTextReader(reader);
        if (ret != 0)
		{
            fprintf(stderr, "%s : failed to parse\n", filename);
			sts = -1;
        }
    } 
	else
	{
        fprintf(stderr, "Unable to open %s\n", filename);
		sts = -1;
    }
	xmlCleanupParser();
	
    // this is to debug memory for regression tests
    xmlMemoryDump();
	
	if ( sts == 0 )
	{
		rfidData->tagCount = parseTagNum + 1;
		if ( debug )
		{
			// Show the config
			for ( tagIndex = 0 ; tagIndex < rfidData->tagCount ; tagIndex++ )
			{
				printf("Tag %d: %llu Volumes (%d %d %d) Side %d Position (%d,%d)\n",
					tagIndex,
					rfidData->tags[tagIndex].tagId,
					rfidData->tags[tagIndex].heartStrength,
					rfidData->tags[tagIndex].leftLungStrength,
					rfidData->tags[tagIndex].rightLungStrength,
					rfidData->tags[tagIndex].side,
					rfidData->tags[tagIndex].xPosition,
					rfidData->tags[tagIndex].yPosition );
			}
		}
	}
	else
	{
		rfidData->tagCount = 0;
	}
	
	return ( sts );
}
