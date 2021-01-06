//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2019 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace boost::asio;
using namespace std;

#define PORT 2000


string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class tcp_connection: public boost::enable_shared_from_this<tcp_connection>
{
	public:
		typedef boost::shared_ptr<tcp_connection> pointer;

		static pointer create(io_context& io_context)
		{
			return pointer(new tcp_connection(io_context));
		}

		tcp::socket& socket()
		{
			return socket_;
		}

		void start()
		{
			message_ = "TCP test successful";

			async_write(socket_, buffer(message_),
					boost::bind(&tcp_connection::handle_write,
					shared_from_this(),
					placeholders::error,
					placeholders::bytes_transferred));

			async_read(socket_, buffer(data,max_length),
				boost::bind(&tcp_connection::handle_read,
				shared_from_this(),
				placeholders::error,
				placeholders::bytes_transferred));
		}

	private:

		tcp::socket socket_;
		string message_;
		enum { max_length = 1024 };
		char data[max_length];
		tcp_connection(io_context& io_context):
			socket_(io_context)
			{
			}

		void handle_write(const boost::system::error_code& err_code,size_t nbytes)
		{
			cout << "ALIVE!" << endl;
			if (!err_code)
			{
				cout << "Server sent ack!"<< endl;
			}
			else
			{
				std::cerr << "error: " << err_code.message() << endl;
				socket_.close();
			}
		}

		void handle_read(const boost::system::error_code& err_code,size_t nbytes)
		{
			cout << "ALIVE!" << endl;
			if (!err_code)
			{
				cout << data << endl;
			}
			else
			{
				std::cerr << "error: " << err_code.message() << endl;
				socket_.close();
			}
		}
};

class tcp_server
{
	public:
		tcp_server(io_context& io_context):
			io_context_(io_context),
			acceptor_(io_context, tcp::endpoint(tcp::v4(), PORT)) // endpoint to listen to any ip
			{
				start_accept();
			}

	private:
		io_context& io_context_;
		tcp::acceptor acceptor_;
		void start_accept()
		{
			tcp_connection::pointer new_connection = tcp_connection::create(io_context_);

			acceptor_.async_accept(new_connection->socket(),boost::bind(&tcp_server::handle_accept, this, new_connection,
			placeholders::error));
		}

		void handle_accept(tcp_connection::pointer new_connection,const boost::system::error_code& error)
		{
			if (!error)
			{
			new_connection->start();
			cout << "Accepted connection!" <<endl;
			}

			start_accept();
		}
};


void restart_system()
{

}
/*
Function to process a user command
param @cmd: user command in command line
*/
void parse_commands(int argc,const char **argv)
{
	// Invalid command
	if (argc < 2)
	{
		exit(EXIT_FAILURE);

		cout << "Usage: ./server <command>" << endl;
	}

	
	// Convert argv to string
	vector<string> cmd(argv, argv + argc);

	auto variable=cmd[0],value = cmd[0],func = cmd[1];

	if (argc > 3)
	{
		variable = cmd[2];
		value = cmd[3];
	}

	cout << "Function: " << func <<endl;
	cout << "variable: " << variable <<endl;
	cout << "value: " << value <<endl;

	cout << func[0]<<endl;

	/*char *ch = func.c_str();
	switch (*ch)
	{
		case "g":
			cout << "Getter!"<<endl;

			// getter code 
			break;
		case "r":
			cout << "Restarting system!"<<endl;
			restart_system();

		
		// Stop stream

		// Start stream

		
		default:
			break;
	}*/
}

int main()
{
	try
	{
		io_context io_context;
		tcp_server server(io_context);
		io_context.run();
	}
	catch (exception& e)
	{
		cerr << e.what() << endl;
	}

	return 0;
}