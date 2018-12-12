/**
 *  epuck_btconnect.h
 *
 *  Setup/shutdown of serial connection to epuck via bluetooth.
 *
 *  2007-10-16 cianci
 *
 **/


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


#define BUFFSIZE 80
typedef unsigned int    uint;
typedef unsigned short  ushort;
typedef unsigned char   uchar;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int epuck_btconnect(ushort r) 
{
    int fd;                             // the file descriptor for the port.

    static ostringstream portname;      // build the port name (i.e. "epuck77")
    portname.str("");                   //  from the id number, passed as a 
    portname << "/dev/rfcomm" << r;     //  parameter.

    fd = open(portname.str().c_str(),   // attempt to open the port.
          O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("epuck_connect failed:");// report an error if unable.
    } else {
        fcntl(fd, F_SETFL, 0);          // otherwise activate the port.
    }

    struct termios options;             // get the control structure for the 
    tcgetattr(fd, &options);            //  port.

    cfsetispeed(&options, B115200);     // set baudrate in,
    cfsetospeed(&options, B115200);     //  and out.
    options.c_cflag |= (CLOCAL | CREAD);// enable receiver & set local mode

    options.c_cflag &= ~PARENB;         // set mode "8N1"
    options.c_cflag &= ~CSTOPB;         //
    options.c_cflag &= ~CSIZE;          //
    options.c_cflag |= CS8;             //

    tcsetattr(fd, TCSANOW, &options);   // send the modified control structure 
                                        //  back; apply changes.

    return (fd);                        // return the open port (or fail==-1).
}

int epuck_btdisconnect(int fd)
{
    return close(fd);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#if 0
string epuck_command( int epuck, epuck_command_t e )
//epuck_command_t epuck_command( int epuck, epuck_command_t e )
{
    char buffer[BUFFSIZE];              // for passing to/from serial channel.
    ostringstream os;                   // formulating request (outgoing).
    istringstream is;                   // parsing response (incoming).

    os.str("");                         // clear the request buffer,
    os << e.command();                  //  and build the command.
    vector<int> v = e.value();          //
    if (v.size()>0)                     //
        for (uint i=0;i<v.size();i++)   //
            os << ',' << v.at(i);       //
    os << '\n';                         //

    int n =                             // send command to the robot, 
        write(epuck, os.str().c_str(),  //
                os.str().length());     //
    if (n < 0)                          //  and make sure the send succeeds.
        return "error!!!\n"; // TODO!!! //

    cout << "e:  " << e << endl;
    cout << "os: " << os.str();         // (print sent command for debugging)

    //fcntl(epuck, F_SETFL, FNDELAY);   // this allows skipping reads which
                                        //  would otherwise block. a bad
                                        //  idea here, as the program seems
                                        //  to terminate (nearly
                                        //  instantaneously) before all the
                                        //  desired input has been read.

    sleep(1);                           // give the robot enough time to repond,
    n = read(epuck,buffer,BUFFSIZE);    //  and then read the response,
    assert(buffer[n-1]=='\n');          //  which should end with a newline.

    buffer[n-1] = 0;                    // terminate the string,
    is.str(buffer);                     //  and put it in the extractor.

    return buffer;                      //  and return it.
}
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

