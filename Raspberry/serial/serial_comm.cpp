
#include <iostream>
#include "serial_comm.hpp"
#include <bitset>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

using namespace std;

void serial_conn::timer_handler(const boost::system::error_code &ec)
{
	// Timer expired – Launch new read
	async_read_until(serial_port, cmd_buf, '\n', listener_handler);
}

void serial_conn::arduino_listener(){
	async_read_until(serial_port, cmd_buf, '\n', listener_handler);
}

void serial_conn::console_listener()
{
	async_read_until(serial_port, console_buf, '\n', console_handler);
}

void serial_conn::get_values_from_system(std::string cmd)
{
    boost::system::error_code error;
    std::string crnl("\r\n");
    std::string serial(cmd);
    std::string send = serial + crnl;

    boost::asio::async_write(serial_port, boost::asio::buffer(send.c_str(), send.size()),
        boost::bind(& serial_conn::handle_get_values, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void serial_conn::handle_get_values(boost::system::error_code &error)
{
    if (error)
    {
        exit(EXIT_FAILURE);
    }
    else std::cout << "Request sent to arduino"<<std::endl;
}
// cmsd <func> <var> <id>
void process_commands(std::string cmd)
{
    auto func = cmd.at(0);


    if (func == RESTART_SYSTEM)
    {

    }
    else // command has 3 fields
    {
        if (func != GET_VALUE)
        {
            get_values_from_system();
        }
        else
        {
            switch(func)
            {
                case SET_O: case SET_U: case SET_C:
                    std::cout << "About to set value " <<std::endl;
                    break;
            }
        }
        auto var = cmd.at(1);
        auto id = cmd.at(2);
    }


    auto variable=cmd[0],value = cmd[0],func = cmd[1];

    if (argc > 3)
    {
        variable = cmd[2];
        value = cmd[3];
    }

}


void serial_conn::console_handler(const boost::system::error_code &ec)
{
    // Extract the newline-delimited message from the buffer.
    std::string line, rqst;
    std::istream is(&console_buf);
    std::getline(is, line);

    rqst = line + std::string("\n");

    std::cout << "Command: " << rqst << std::endl;

    process_commands(rqst);


}
// Opens arduino serial port
void serial_conn::open_port(string port,int baud_rate)
{
    do
    {
        serial_port.open(port,error_code);
    } while (error_code);
    
    // Set desired baud_rate value
    serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),error_code);

    std::cout << "Port opened successfully" << std::endl;

}


int main(int argc, char* argv[])
{
    try {
        cout << "Hello"<<endl;

        serial_conn serial("/dev/ttyACM0",115200);

        serial.io_context.run();


    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}