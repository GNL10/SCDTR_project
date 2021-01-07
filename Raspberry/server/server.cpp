#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
#define PORT 2000

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class tcp_connection: public boost::enable_shared_from_this<tcp_connection>
{
	public:
		typedef boost::shared_ptr<tcp_connection> pointer;

		static pointer create(boost::asio::io_context& io_context)
		{
		return pointer(new tcp_connection(io_context));
		}

		tcp::socket& socket()
		{
		return socket_;
		}

		// Write a message to socket
		void write(std::string msg)
		{
			boost::asio::async_write(socket_, boost::asio::buffer(msg,sizeof(msg)),
			boost::bind(&tcp_connection::handle_write, shared_from_this(),
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));

		}
	void start()
	{

		std::cout << "Server started!" << std::endl;
		std::string welcome_message_ = "Greetings from server!\n";

	
		// Write welcome message to client
		write(welcome_message_);


		// Listen for client's messages
		//start_read();
	}

	// Sends response to received message
	// Reads user input command
	/*void start_read()
	{
		std::cout << "start_read_console" << std::endl;
		boost::asio::async_read_until(input_, recv_buffer_, '\n', boost::bind(&tcp_connection::handle_read,shared_from_this(),
		boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
	}
	

	// Sends response to received message
	void handle_read(const boost::system::error_code& ec, std::size_t length)
	{ 		
		if (!ec)
		{
			// Extract the newline-delimited message from the buffer.
			std::string line, terminated_line;
			std::istream is(&recv_buffer_);
			std::getline(is, line);

		// Empty messages are heartbeats and so ignored.
		if (!line.empty())
		{
			std::cout << "Sending: " << line << "\n";
			terminated_line = line + std::string("\n");
			std::size_t n = terminated_line.size();
			terminated_line.copy(send_buffer_, n);

			// Send back response to the client
			std::cout << "Enviando resposta: " << terminated_line <<std::endl;

			write(send_buffer_);

			//boost::asio::async_write(socket_, boost::asio::buffer(send_buffer_,n),boost::bind(&tcp_connection::handle_send, this, _1, _2));
			
		}
		}
		else
		{
			std::cout << "Error on handle_read_console: " << ec.message() << "\n";

		}
		start_read();
	}*/

	private:
		tcp_connection(boost::asio::io_context& io_context): socket_(io_context)
		{
		}

		void handle_write(const boost::system::error_code& ec,size_t n)
		{
			if(!ec)
			{
				std::cout << "Sent message" << std::endl;
			}else delete this;
		}

		tcp::socket socket_;
		//boost::asio::streambuf recv_buffer_,_input_buffer_;
		std::string message_,data_recv;
		//boost::asio::posix::stream_descriptor input_;
		enum {max_length=1024};
		char send_buffer_[max_length];
};

class tcp_server
{
public:
  tcp_server(boost::asio::io_context& io_context)
    : io_context_(io_context),
      acceptor_(io_context, tcp::endpoint(tcp::v4(), PORT))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    tcp_connection::pointer new_connection =
      tcp_connection::create(io_context_);

    acceptor_.async_accept(new_connection->socket(),
        boost::bind(&tcp_server::handle_accept, this, new_connection,
          boost::asio::placeholders::error));
  }

  void handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_connection->start();
    }

    start_accept();
  }

  boost::asio::io_context& io_context_;
  tcp::acceptor acceptor_;
};

int main()
{
  try
  {
    boost::asio::io_context io_context;
    tcp_server server(io_context);
    io_context.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}