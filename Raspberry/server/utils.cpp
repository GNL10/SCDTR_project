#include <stdio.h>
#include <pthread.h>
#include <boost/asio.hpp>
#include <boost/tokenizer.hpp>
#include <iostream>

using namespace std;
using namespace boost;


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



int main(int argc , const char **argv)
{
    parse_commands(argc,argv);

    return 0;
}
