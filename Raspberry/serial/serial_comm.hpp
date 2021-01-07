#include <unistd.h>
#include <iostream>
#include <list>

#define RESTART_SYSTEM 'r'
#define GET_VALUE 'g'
#define  SET_O 'o'
#define SET_U 'u'
#define SET_C 'c'
#define STREAM_ 's'

typedef struct req
{

    unsigned long timestamp;

    float lux;

    float duty_cycle;

    float lux_ref;

    float c_err;

    float c_var;

}REQUEST;

extern std::list <REQUEST> last_min_buffer;

class serial_conn
{
public:
    boost::asio::io_service io_context;
    boost::system::error_code error_code;
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    serial_conn(string port, unsigned int baud_rate): io_context(), input_(io_context, ::dup(STDIN_FILENO))
    {
        open_port(port,baud_rate);
    }

    void timer_handler(const boost::system::error_code &error_code);
    void arduino_listener();
    void console_listener();
    void listener_handler(const boost::system::error_code &error_code);
    void console_handler(const boost::system::error_code &error_code);
    void get_values_from_system(std::string cmd);
    void serial_conn::handle_get_values(boost::system::error_code &error);
    void open_port(string port,int baud_rate);




private:

    boost::asio::serial_port serial_port;
    boost::asio::streambuf cmd_buf {256};
    boost::asio::streambuf console_buf {1024};
    boost::asio::posix::stream_descriptor input_;
};