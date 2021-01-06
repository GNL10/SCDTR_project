
#include <iostream>
#include "serial_comm.hpp"

using namespace std;
using namespace boost;


int main(int argc, char* argv[])
{
    try {

        SimpleSerial serial("/dev/ttyUSB0",115200);

        serial.writeString("Hello world\n");

        cout<<serial.readLine()<<endl;

    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}