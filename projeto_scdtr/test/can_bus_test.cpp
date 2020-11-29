#include <SPI.h>
#include <EEPROM.h>
#include "mcp2515.h"

MCP2515 mcp2515(10); //SS pin 10

class can_frame_stream {
        //10 slots buffer - increase if needed
        static const int buffsize = 10;
        can_frame cf_buffer[ buffsize ];
        int read_index; //where to read next message
        int write_index; //where to write next message
        bool write_lock; //buffer full
    public:
        can_frame_stream() : read_index( 0 ) , write_index( 0 ), write_lock( false ) {};
        int put( can_frame & );
        int get( can_frame & );
} volatile cf_stream; //create one object to use


inline int can_frame_stream::put( can_frame &frame ) {
    if( write_lock )
        return 0; //buffer full
    cf_buffer[ write_index ] = frame;
    write_index = ( ++write_index ) % buffsize;
    if( write_index == read_index)
        write_lock = true; //cannot write more
    return 1;
}

inline int can_frame_stream::get( can_frame &frame ) {
    if( !write_lock && ( read_index== write_index ) )
        return 0; //empty buffer
    if( write_lock && ( read_index == write_index ) )
        write_lock = false; //release lock
    frame = cf_buffer[ read_index ];
    read_index = ( ++read_index ) % buffsize;
    return 1;
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
    if( !cf_stream.put( frm ) ) //no space
        arduino_overflow = true;
    }
    //check messages in buffer 1
    if ( irq & MCP2515::CANINTF_RX1IF ) {
        mcp2515.readMessage( MCP2515::RXB1, & frm);
    if( !cf_stream.put( frm ) ) //no space
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

void setup()
{
    Serial.begin(115200);
    SPI.begin();
    attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
    //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
    SPI.usingInterrupt(0);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    //mcp2515.setNormalMode();
    // use mcp2515.setLoopbackMode() for local testing
    mcp2515.setLoopbackMode();
}

unsigned long counter = 0;
void loop() {
    //send a few msgs in a burst
    for( int i = 0; i < 4 ; i++ ) {
        Serial.print( "Sending: " );
        Serial.println( counter );
        if( write( i , counter++ ) != MCP2515::ERROR_OK )
            Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
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
        cli(); has_data = cf_stream.get( frame ); sei();
        while (has_data) {
            my_can_msg msg;
            for( int i = 0 ; i < 4 ; i++ )
                msg.bytes[ i ] = frame.data[ i ];
            Serial.print( "\t\tReceiving: " ); Serial.println( msg.value );
            cli(); has_data = cf_stream.get( frame ); sei();
        }
    }
    delay(1); //some time to breath
}