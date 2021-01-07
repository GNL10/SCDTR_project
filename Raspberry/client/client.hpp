#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ec = const boost::system::error_code &;

class client
{
    public:
        bool stopped_;
		ip::tcp::socket socket_;
		deadline_timer deadline_;
		deadline_timer heartbeat_timer_;
		enum {max_length=1024};
		char send_buffer_[max_length];

        ip::tcp::resolver res; // to find out the server coordinates
        ip::tcp::socket sock; // socket object
        streambuf input_buffer; //buffer to store the incoming server messages
        posix::stream_descriptor input; // console object
        streambuf console_buffer; // buffer to store the console input
        char send_buffer[ 1024 ]; // buffet to store the data to send to server
        client(io_context & io): 
            stopped_ {false},
            //res {io}, 
            sock {io}, 
            input {io, ::dup(STDIN_FILENO)}, 
            console_buffer ( 100000 )
            { }


        void start_connect_server( char *host, char *port );
        void start_read_server() ;
        void start_read_console();


        void start( char *host, char *port)
        {
            start_connect_server(host, port);
            start_read_console();
        }
};