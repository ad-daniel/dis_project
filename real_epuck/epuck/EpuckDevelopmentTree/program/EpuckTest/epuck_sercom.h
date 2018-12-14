/**
 *  epuck_sercom.h
 *
 *  Data structure for holding SerCom commands of the form ("char,int,int,...").
 *
 *  2007-10-16 cianci
 *
 **/


#include <string>
#include <sstream>
#include <vector>
using namespace std;

#include <stdarg.h>  /* Variable length argument list processing */


#define BUFFSIZE 80
typedef unsigned int    uint;
typedef unsigned short  ushort;
typedef unsigned char   uchar;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class epuck_command_t {
    
    private:
        char C;
        vector<int> V;

    private:
        short argc(const char c) const
        {
            short n;

            switch(c)                   // base on command, set number of args.
            {                           // capitals = requests (outgoing), 
                                        //  & lowercase = responses (incoming).
              // 0 parameters
              case 'A':                 // Accelerometer
              case 'b':                 // Body led
              case 'C':                 // Selector position
              case 'd':                 // Set motor speed
              case 'E':                 // Get motor speed
              case 'f':                 // Front led
              case 'I':                 // Get camera parameter
              case 'j':                 // Set camera parameter
              case 'l':                 // Led
              case 'N':                 // Proximity
              case 'O':                 // Light sensors
              case 'p':                 // Set motor position
              case 'Q':                 // Get motor position
              case 'R':                 // Reset
              case 'S':                 // Stop
              case 's':                 // Stop
              case 't':                 // Play sound
              case 'U':                 // Get microphone amplitude
              case 'V':                 // Version of SerCom
                n = 0; 
                break;

              // 1 parameter
              case 'B':                 // Body led
              case 'c':                 // Selector position
              case 'F':                 // Front led
              case 'T':                 // Play sound
                n = 1; 
                break;
                
              // 2 parameters
              case 'D':                 // Set motor speed
              case 'e':                 // Get motor speed
              case 'L':                 // Led
              case 'P':                 // Set motor position
              case 'q':                 // Get motor position
                n = 2; 
                break;
                
              // 3 parameters
              case 'a':                 // Accelerometer
              case 'u':                 // Get microphone amplitude
                n = 3; 
                break;

              // 4 parameters
              case 'J':                 // Set camera parameter
                n = 4; 
                break;

              // 5 parameters
              case 'i':                 // Get camera parameter
                n = 5; 
                break;

              // 8 parameters
              case 'n':                 // Proximity
              case 'o':                 // Light sensors
                n = 8; 
                break;

              // unused or unknown commands
              case 'G':                 // IR receiver
              case 'g':                 // IR receiver
              case 'H':                 // Help
              case 'h':                 // Help
              case 'K':                 // Calibrate proximity sensors
              case 'k':                 // Calibrate proximity sensors
              case 'r':                 // Reset
              case 'v':                 // Version of SerCom
              case 'z':                 // Unknown command
              default:                  // Unknown command
                n = 0;
                break;
            }

            return n;
        }

    public:
        epuck_command_t()
        : C('Z'), V(0) {}

        epuck_command_t(string s)
        {
            replace(s.begin(),          // convert commas to spaces (default
                    s.end(),',',' ');   //  formatted input stream delimiter),
            istringstream is(s);        //  and load string into stream.

            is >> C;                    // get the command name

            int x;                      //
            for(int i=0;i<argc(C);i++)  // extract appropriate number of ints
            {                           //
                is >> x;                //  from the string,
                V.push_back(x);         //  pushing them into the vector.
            }                           //
            
        }
        
        epuck_command_t(char c,...)
        : C(c)
        {
            va_list ap;                 // alloc variable length argument list,
            va_start(ap,c);             //  and skip 1st arg.

            for(int i=0;i<argc(c);i++)  // push appropriate number of args
                V.push_back(            //  onto the vector.
                    va_arg(ap,int));    //

            va_end(ap);                 // clean up arglist when done.
        };
        
        const char command( void ) const
        { return C; }
        
        vector<int> value( void ) const
        { return V; }

        epuck_command_t &operator= (const epuck_command_t &copy)
        {
            this->C = copy.C;
            this->V = copy.V;
            return *this;
        }

        friend ostream &operator<< (ostream &os, const epuck_command_t e)
        { 
            char c = e.C;
            os << c; 
            for(uint i=0;i<e.V.size();i++)
                os << ',' << e.V.at(i);
            return os;
        }
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

