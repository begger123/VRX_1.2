#include <vehicle_compass/compass_class.h>

//the purpose of this class is to assemble the ROS xsense node and the compass node
//into a single state message
//hacked in fucntionality that needs to be streamlined - also publishes a nav_msgs/Odometry message
serial_compass::serial_compass(ros::NodeHandle &nh) : compass_nh_(&nh)
{
	compass_pub = compass_nh_->advertise<geometry_msgs::Quaternion>("/compass", 10);
	this->port_setup();
}

serial_compass::~serial_compass()
{

}

int serial_compass::port_setup()
{
	//Initializing the COM port
    //Make the FD.
    cmp_fd = open(OSC_FILE_DES, O_RDWR | O_NOCTTY | O_NONBLOCK);

    //Check to see that "open" call was successful.
    if (cmp_fd < 0)
	{ //If it broke....
        ROS_ERROR("open");
        return (EXIT_FAILURE);
    }
    //Print saying that was the case.
    ROS_DEBUG("Opened COM port to OS5000:%s\r\n", OSC_FILE_DES);

    //Get current attributes for cmp_fd.
    if (tcgetattr(cmp_fd, &old_cmp_tio) < 0)
	{ //If it broke...
        ROS_ERROR("tcgetattr -> cmp_fd:");
        return (EXIT_FAILURE);
    }

    //Make a new termios structure with the settings we want.
    new_cmp_tio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    new_cmp_tio.c_iflag = IGNPAR;
    new_cmp_tio.c_oflag = 0;
    new_cmp_tio.c_lflag = 0;
    new_cmp_tio.c_cc[VMIN]=1;
    new_cmp_tio.c_cc[VTIME]=0;

    //Flush whatever is in the buffer for that file.
    if (tcflush(cmp_fd, TCIFLUSH) < 0)
	{ //If it broke...
        ROS_ERROR("tcflush -> cmp_fd:");
        return (EXIT_FAILURE);
    }
    //Set new attributes.
    if (tcsetattr(cmp_fd, TCSANOW, &new_cmp_tio) < 0)
	{ //If it broke...
        ROS_ERROR("tcsetattr -> cmp_fd:");
        return (EXIT_FAILURE);
    }
}

bool serial_compass::checksumcheck(char * NMEAstring)
{
	//returns true or false
	char * starpose;
	int i = 1;
	char checksumchar[2];
	int  checksum_fromstring;
	int checksum_fromcalc = 0;

	if(NMEAstring[0] != '$')
	{
		return false;
	}
	starpose = strchr(NMEAstring, '*');
	if(starpose == NULL)
	{
		return false;	
	}
	
	strncpy(checksumchar, NMEAstring+(starpose-NMEAstring)+1, 2);
	checksumchar[2] = '\0';

	sscanf(checksumchar,"%x",&checksum_fromstring);
	for(i=1; i<starpose-NMEAstring; i++)
	{
		checksum_fromcalc ^= NMEAstring[i];
	}

	if(checksum_fromstring == checksum_fromcalc)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void serial_compass::parsing_compass(char string[50])
{
	//use sscanf to parse compass ASCII message
	i = sscanf(string, "%c %c  %f   %c   %f  %c  %f     %c   %f  %c   %d", &letterdol, &letterC, &Yaw, &letterP, &Pitch, &letterR, &Roll, &letterT, &Temperature, &letteraster, &checksum);
}

int serial_compass::loop()
{
	while(ros::ok())
	{
	   	//declare timeout structure if no new data in buffer
		//must be redeclared in loop to avoid select erroR
		struct timeval timeout = {1,0};
		char c;
		fd_set fds;
		int ret1, ret2;

		//Zero file descriptor set, then append all types.
		FD_ZERO(&fds);
		FD_SET(cmp_fd, &fds);
		FD_SET(STDIN_FILENO, &fds);
		ret1 = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);
		if (ret1 == -1)
		{ //If select returns an error.
			ROS_ERROR("select");
			need_exit = 1;
		}
		else if (ret1 > 0)
		{
        	if (FD_ISSET(cmp_fd, &fds))
		{
            	do{ //Read while we get errors that are due to signals.
        	        ret2 = read(cmp_fd, &c, 1);
                    }while (ret2 < 0 && errno == EINTR);
                if (ret2 == 1)
				{
                	data_str[cmp_ind] = c;
					cmp_ind++;
					if(c == '\n')
					{
						cmp_ind = 0;
						compassFlag = this->checksumcheck(data_str);
						if(compassFlag == true)
						{
							this->parsing_compass(data_str);
							this->orientation=tf::createQuaternionMsgFromRollPitchYaw(Roll*M_PI/180.0,Pitch*M_PI/180.0,Yaw*M_PI/180.0);  // Create this quaternion from yaw (in radians)
							compass_pub.publish(this->orientation);
							compassFlag = false;
						}
					}
				}
			}
			else if(ret2 == -1)
			{ //If we're at the start of a new string.
		        	ROS_ERROR("read");
				need_exit = 1;
			}
		}
		else if (ret1 == 0)
		{
            ROS_ERROR("select: timeout expired.\r\n");
        }
    }
    return 01;
}
