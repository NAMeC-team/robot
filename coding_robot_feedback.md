# Notes
Only `RX_ADDR_P0` is used for transmission,
only the pipe with a similar name is used as well.

# New feedback data ?
- Register '08 OBSERVE_TX'
Counter for lost transmitted packets
- Carrier Detect (CD) nRF24L01 -> Received Power Detector nRF24L01+ (RPD)
See section 6.4 page 24

# ACK payload method
## Steps
- Enable Auto acknowledgement on 
- Disable Auto retransmit (register '04 SETUP_RETR' [3:0])
- Enable ACK payload
Activate using the reg '1D FEATURE' register

# TX/RX antennas on base station
## Steps
Robot
- Receive packet on robot
- Stop listening (CE=0)
- Switch to TX settling
    - Load packet into TX_FIFO
    - Set PRIM_RX=0
    - Set CE=1
    - 130 μs load time
    - TX mode
- Switch directly to TX
- Transmit on RX_ADDR_P0 on the second frequency
- Return to RX