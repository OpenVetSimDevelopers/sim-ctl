/*
 * ainmon.cpp
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

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <ctype.h>

#include <time.h>
#include <errno.h>
#include <termios.h>
#include <libgen.h>

#include <string.h>

int debug = 1;

#define PATH_MAX	512
char ain_path[PATH_MAX];
int ain_path_found = 0;

int findAINPath(void )
{
	FILE *fp;
	
	fp = popen("find /sys/devices -name AIN0", "r" );
	
	while ( fgets(ain_path, PATH_MAX, fp) != NULL)
	{
		sprintf(ain_path, "%s", dirname(ain_path ) );	// Return the directory
		if ( debug > 1 )
		{
			printf("%s\n", ain_path);
		}
	}
	pclose(fp );
	
	if ( strlen(ain_path ) > 0 )
	{
		ain_path_found = 1;
		if ( debug )
		{
			printf("AIN Path is %s\n", ain_path );
		}
	}
	return ( 0 );
}


int
read_ain(int chan )
{
	int fd;
	int val = 0;
	char name[256];
	int sts;
	char buf[8];
	
	if ( ain_path_found == 0 )
	{
		findAINPath();
	}
	if ( ain_path_found == 1 )
	{
		sprintf(name, "%s/AIN%d", ain_path, chan);
		fd = open (name, O_RDONLY );
		if ( fd < 0 )
		{
			if ( debug )
			{
				fprintf(stderr, "Failed to open %s", name );
			}
			perror("open" );
			exit ( -2 );
		}
		
		sts = read(fd, buf, 4 );
		if ( sts == 4 )
		{
			val = atoi(buf );
		}
		close(fd);
	}

	return ( val );
}

int
main(int argc, char *argv[] )
{
	int ain0;
	int ain1;
	int ain2;
	int ain3;
	int ain4;
	int ain5;
	while ( 1 )
	{
		ain0 = read_ain(0 );
		ain1 = read_ain(1 );
		ain2 = read_ain(2 );
		ain3 = read_ain(3 );
		ain4 = read_ain(4 );
		ain5 = read_ain(5 );
		printf("%5d, %5d, %5d, %5d, %5d, %5d\n", ain0, ain1, ain2, ain3, ain4, ain5 );
		
		usleep(1000000 );
	}
}
