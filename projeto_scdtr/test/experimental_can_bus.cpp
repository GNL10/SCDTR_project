#include <SPI.h>
#include <EEPROM.h>
#include "can_frame_stream.h"
#include "configs.h"

#define WAIT_ID_TIME (unsigned long) 5000000 // 5 secs
#define BROADCAST_MSG (uint32_t) 0x00000200

can_frame_stream *cf_stream = new can_frame_stream();
MCP2515 mcp2515(10); //SS pin 10

void load_EEPROM_vars();
// variables saved in EEPROM
byte ID;
float C1, m, b; 
byte id_vec[2];
byte node_ctr = 0;

/* Serial read interface*/
char serial_input[SERIAL_INPUT_SIZE+1];
int serial_input_index = 0;

/**
 * reads 1 char from serial and concatenates it into the string serial_input
 * @returns 1 when the enter key is pressed else 0
 */
int serial_read_lux () {
  if (Serial.available()) {
    char read_byte = Serial.read();
    if(read_byte == 10){ // 10 -> new line
      serial_input[serial_input_index] = '\0';
      return 1;
    }
    serial_input[serial_input_index++] = read_byte;
  }
  return 0;
}

//notification flag for ISR and loop()
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

void irqHandler()
{
    can_frame frm;
    uint8_t irq = mcp2515.getInterrupts();
    //check messages in buffer 0
    if ( irq & MCP2515::CANINTF_RX0IF ) {
        mcp2515.readMessage( MCP2515::RXB0, & frm );
    if( !cf_stream->put( frm ) ) //no space
        arduino_overflow = true;
    }
    //check messages in buffer 1
    if ( irq & MCP2515::CANINTF_RX1IF ) {
        mcp2515.readMessage( MCP2515::RXB1, & frm);
    if( !cf_stream->put( frm ) ) //no space
        arduino_overflow = true;
    }
    irq = mcp2515.getErrorFlags(); //read EFLG
    if( (irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR) ) {
        mcp2515_overflow = true;
        mcp2515.clearRXnOVRFlags();
    }
    mcp2515.clearInterrupts();
    interrupt = true; //notify loop()
} //end irqHandler()

//union to pack/unpack long ints into bytes
union my_can_msg {
unsigned long value;
unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, uint32_t val) {
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 4;
    my_can_msg msg;
    msg.value = val; //pack data
    for( int i = 0; i < 4; i++ ) //prepare can message
        frame.data[i] = msg.bytes[i];
    
    //send data
    return mcp2515.sendMessage(&frame);
}

void process_recvd_msg(char msg[]) {
    switch (msg[0])
    {
    case 'a': // add new node to network
        id_vec[node_ctr++] = msg[1]; // id of new node

        break;
    
    default:
        break;
    }
}

//uint32_t mask0 = 0xFFFFFFFF;
uint32_t mask0 = CAN_SFF_MASK;
//               0x000007FF
uint32_t mask1 = 0x00000400;
uint32_t filt0 = 0x00000002;


void setup()
{
    Serial.begin(115200);
    load_EEPROM_vars();

    SPI.begin();
    attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
    //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
    SPI.usingInterrupt(0);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

    mcp2515.setFilterMask(MCP2515::MASK0, 0, CAN_SFF_MASK);
    mcp2515.setFilter(MCP2515::RXF0, 0, (uint32_t)ID);
    mcp2515.setFilter(MCP2515::RXF1, 0, filt1);
    
    mcp2515.setFilterMask(MCP2515::MASK1, 0, mask1);
    mcp2515.setFilter(MCP2515::RXF2, 0, filt1); //accepts msg with ID ending with 1010
    /*mcp2515.setFilter(MCP2515::RXF3, 0, filt2); //accepts msg with ID ending with 0011
    mcp2515.setFilter(MCP2515::RXF4, 0, 0); //accepts msg with ID ending with 0000
    mcp2515.setFilter(MCP2515::RXF5, 1, 0); //accepts extended frames with ID ending with 0000
    */
    mcp2515.setNormalMode();
    // use mcp2515.setLoopbackMode() for local testing
    //mcp2515.setLoopbackMode();

    
}

enum state : byte {start, send_id_broadcast, wait_for_ids, calibrate}; // states of the system
state curr_state = start;

unsigned long counter = 0;
unsigned long todel = 100;
void loop() {
    //send a few msgs in a burst
    
    /*
    for( int i = 0; i < 4 ; i++ ) {
        Serial.print( "Sending: " );
        Serial.println( counter );
        if( write( i, counter++) != MCP2515::ERROR_OK )
            Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
    }*/
    

    if(serial_read_lux()){ // command is ready to be processed
        serial_input_index = 0;
        Serial.print("Sending to id ");
        Serial.print(serial_input[0]);
        Serial.print(": ");
        //for (int i = 1; i < 4; i++)
        Serial.println(serial_input[1]);
        if( write( serial_input[0] - '0', 10) != MCP2515::ERROR_OK )
            Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
            
    }

    

    switch (curr_state)
    {
    case start:
        curr_state = send_id_broadcast;
        break;
    case send_id_broadcast:
        Serial.println("Printing own id");
        if( write(BROADCAST_MSG | ID, 0xFFFFFFFF) != MCP2515::ERROR_OK )
            Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
        curr_state = wait_for_ids;
        break;
    case wait_for_ids:
        if (micros() > WAIT_ID_TIME) { // move on to calibration
            curr_state = calibrate;
        }
        // else

        // add more arduinos to the network
        // figure out how to add more memory dynamically 
        

        break;
    case calibrate:
        Serial.println("Calibrating.");
        delay(5000);
        break;
    default:
        break;
    }

    if( interrupt ) {
        interrupt = false;
        if( mcp2515_overflow ) {
            Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
            mcp2515_overflow = false;
        }

        if( arduino_overflow ) {
            Serial.println( "\t\t\t\tArduino Buffers Overflow" );
            arduino_overflow = false;
        }
        can_frame frame;
        bool has_data;
        cli(); has_data = cf_stream->get( frame ); sei();
        while (has_data) {
            my_can_msg msg;
            for( int i = 0 ; i < 4 ; i++ )
                msg.bytes[ i ] = frame.data[ i ];
            Serial.print( "\t\tReceiving: ID :");
            Serial.print(frame.can_id);
            Serial.print(" -> ");
            Serial.println(msg.value);
            cli(); has_data = cf_stream->get(frame); sei();
            //must process the message here!!!!!
            //process_recvd_msg();
        }
    }
    delay(10); //some time to breathe
}


/**
 * Loads the values from the EEPROM to the variables C1, m and b of the program
 */
void load_EEPROM_vars() {
  int address = 0;
  

  EEPROM.get(address, ID);
  address += sizeof(ID);
  EEPROM.get(address, C1);
  address += sizeof(C1);
  EEPROM.get(address, m);
  address += sizeof(m);
  EEPROM.get(address, b);
  address += sizeof(b);

  Serial.println();
  Serial.println("### EEPROM VALUES ###");
  Serial.print("ID : ");
  Serial.println(ID);
  Serial.print("C1 : ");
  Serial.println(C1, 7);
  Serial.print("m : ");
  Serial.println(m, 7);
  Serial.print("b : ");
  Serial.println(b, 7);
  Serial.println("### END OF VALUES ###");
}