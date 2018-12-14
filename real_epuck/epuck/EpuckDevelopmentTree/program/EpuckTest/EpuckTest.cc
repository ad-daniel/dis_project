/**
 *  EpuckTest.cc
 *
 *  Mini test suite for checking basic e-puck functionality with SerCom. 
 *
 *  2007-10-16 cianci
 *
 **/


#include "epuck_sercom.h"
#include "epuck_btconnect.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

#if 0
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#endif


#define BUFFSIZE 80
typedef unsigned int    uint;
typedef unsigned short  ushort;
typedef unsigned char   uchar;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

epuck_command_t epuck_command( const int epuck, const epuck_command_t e )
{
    char buffer[BUFFSIZE];              // for passing to/from serial channel.
    ostringstream os;                   // formulating request (outgoing).

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
        return 'Z';                     //

    //cout << "e:  " << e << endl;
    //cout << "os: " << os.str();         // (print sent command for debugging)

    //fcntl(epuck, F_SETFL, FNDELAY);   // this allows skipping reads which
                                        //  would otherwise block. a bad
                                        //  idea here, as the program seems
                                        //  to terminate (nearly
                                        //  instantaneously) before all the
                                        //  desired input has been read.

    usleep(250000);                     // give the robot enough time to repond,
    n = read(epuck,buffer,BUFFSIZE);    //  and then read the response,
    assert(buffer[n-1]=='\n');          //  which should end with a newline.

    buffer[n-1] = 0;                    // terminate the string,

    return string(buffer);              //  and return it.
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define NUMSAMPLES  10
#define GRAVITY     750
#define EPSILON     125
#define RESTVAL     2100
typedef enum _xyz { X=0,Y=1,Z=2 } xyz;
typedef enum _sgn { POS=1,NEG=-1 } sgn;

void test_accelerometer(const int ep, const xyz dir, const sgn sign)
{
    vector<int> avg(3,0);
    int restval;

    for (int i=0; i<NUMSAMPLES; i++)
    {
        cout << '.' << flush;
        epuck_command_t e = epuck_command(ep,'A');
        //cout << "        " << e << endl;
        for (int j=0; j<3; j++)
            avg.at(j) += e.value().at(j);
    }
    cout << endl;
    for (int j=0; j<3; j++)
    {
        if (j==dir) restval = RESTVAL+(sign*GRAVITY); else restval = RESTVAL;
        avg.at(j) /= NUMSAMPLES;
        cout << "    acc(" << j << "): " << setw(4) << avg.at(j) << "  ("
             << setw(5) << avg.at(j)-restval << " --> " 
             << ((abs(avg.at(j)-restval)<EPSILON)?"PASS":"FAIL")
             << " )" << endl;
        avg.at(j)=0;
    }
    cout << endl;
}

void epuck_test_accelerometer(const int ep)
{
    char ready; 

    cout << "===== ACCELEROMETER =====" << endl << endl;

    cout << "(1) Testing Z+ [upright position]" << endl;
    test_accelerometer(ep,Z,POS);

    cout << "(2) Testing Z- [upside down]   **PRESS ENTER**" << flush;
    cin >> noskipws >> ready; 
    test_accelerometer(ep,Z,NEG);

    cout << "(3) Testing X+ [right side]    **PRESS ENTER**" << flush;
    cin >> noskipws >> ready; 
    test_accelerometer(ep,X,POS);

    cout << "(4) Testing X- [left side]     **PRESS ENTER**" << flush;
    cin >> noskipws >> ready; 
    test_accelerometer(ep,X,NEG);

    cout << "(5) Testing Y+ [back down]     **PRESS ENTER**" << flush;
    cin >> noskipws >> ready; 
    test_accelerometer(ep,Y,POS);

    cout << "(6) Testing Y- [front down]    **PRESS ENTER**" << flush;
    cin >> noskipws >> ready; 
    test_accelerometer(ep,Y,NEG);

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void epuck_test_motors(const int ep)
{
    char ready; 

    cout << "======== MOTORS =========" << endl << endl;

    cout << "(1) Spin robot CW with speed 50.       **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',50,-50));

    cout << "(2) Spin robot CW with speed 100.      **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',100,-100));

    cout << "(3) Spin robot CW with speed 200.      **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',200,-200));

    cout << "(4) Spin robot CW with speed 400.      **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',400,-400));

    cout << "(5) Spin robot CW with speed 800.      **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',800,-800));

    cout << "(6) Spin robot CW with speed 1200.     **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',1200,-1200));

    cout << "(7) Stop robot.                        **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('S'));


    cout << "(8) Spin robot CCW with speed 50.      **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-50,50));

    cout << "(9) Spin robot CCW with speed 100.     **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-100,100));

    cout << "(10) Spin robot CCW with speed 200.    **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-200,200));

    cout << "(11) Spin robot CCW with speed 400.    **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-400,400));

    cout << "(12) Spin robot CCW with speed 800.    **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-800,800));

    cout << "(13) Spin robot CCW with speed 1200.   **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('D',-1200,1200));

    cout << "(14) Stop robot.                       **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    epuck_command(ep,epuck_command_t('S'));

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define NUMSENSORS      8
#define EPSILON_DIST    200

void test_distance(const int ep)
{
    vector<int> avg(NUMSENSORS,0);

    for (int i=0; i<NUMSAMPLES; i++)
    {
        cout << '.' << flush;
        epuck_command_t e = epuck_command(ep,'N');
        //cout << "        " << e << endl;
        for (int j=0; j<NUMSENSORS; j++)
            avg.at(j) += e.value().at(j);
    }
    cout << endl;
    for (int j=0; j<NUMSENSORS; j++)
    {
        avg.at(j) /= NUMSAMPLES;
        cout << "    dist(" << j << "): " << setw(5) << avg.at(j) << "  ( "
             << ((abs(avg.at(j))<EPSILON_DIST)?"PASS":"FAIL") << " )" << endl;
        avg.at(j)=0;
    }
    cout << endl;
}

void epuck_test_distance(const int ep)
{
    char ready; 

    cout << "======= DISTANCE ========" << endl << endl;

    cout << "(1) Place robot away from obstacles.   **PRESS ENTER**" << flush;
    cin >> noskipws >> ready;
    test_distance(ep);

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int main ( int argc, char *argv[] )
{
    int e=1, ep;
    /*
    ostringstream filename; 
    fstream outfile; 
    filename.str("");
    filename << 'r' << id[e] << ".out";
    outfile.open(filename.str().c_str(),fstream::out);
    */
        
    ep = epuck_btconnect(atoi(argv[e]));
    assert(ep != -1);
    epuck_command_t z;
    do {
        z = epuck_command(ep,'A');
        //cout << z << endl;
    } while (z.command()=='z');


    epuck_test_accelerometer(ep);
    epuck_test_motors(ep);
    epuck_test_distance(ep);


    epuck_btdisconnect(ep);
    //outfile.close();

    return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

