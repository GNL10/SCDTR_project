NODE ID CAN NOT BE 0, because we are sending id as char in can bus 

Mask and filter basic ideas:
    for node with ID 1

    for main node
    0000 0000 mask => must accept all messages
    xxxx xxxx filter

    Broadcast!!!
    maybe 255?

    RTR to request info from a certain node


Running platformio remote client on raspberry pi:
    Installation: 
        https://community.platformio.org/t/howto-raspberry-pi-3-as-remote-agent-oct-2020/16700
    Quick Commands:
        pio remote device list
        pio remote run --upload-port <PORT NAME> --target upload
        pio remote device monitor --baud 115200 -p <PORT NAME>

        <PORT NAME> example : /dev/ttyACM1

        raspberry bia
        ssh -p 43000 pi@94.62.143.69