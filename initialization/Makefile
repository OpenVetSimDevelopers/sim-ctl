#
# This file is part of the sim-ctl distribution (https://github.com/OpenVetSimDevelopers/sim-ctl).
# 
# Copyright (c) 2019 VetSim, Cornell University College of Veterinary Medicine Ithaca, NY
# 
# This program is free software: you can redistribute it and/or modify  
# it under the terms of the GNU General Public License as published by  
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but 
# WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

default: all

all: 

install: all
	sudo cp -u simctl /etc/init.d/simctl
	sudo chmod ug+x /etc/init.d/simctl
	# sudo insserv simctl
	sudo update-rc.d simctl defaults
	if [ ! -d /simulator ]; then \
		sudo mkdir /simulator; \
		sudo chown debian:debian /simulator; \
		cp rfid.xml /simulator; \
		cp soundList.csv /simulator; \
	fi
	
	if [ ! -f /simulator/soundList.csv ]; then \
		cp soundList.csv /simulator; \
	fi

clean:
 