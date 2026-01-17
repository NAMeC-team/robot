serializing motor transmission
- download nanopb
- remove feedback processing in mainboard & feedback sending of each motor
- create new Protobuf packet with all motor commands
    
    requires new field "id" inside "MainBoardToBrushless" message
        Brushless gets its ID at compile time
        ID definition :
            - TOP_LEFT = 1
            - BOTTOM_LEFT = 2
            - BOTTOM_RIGHT = 3
            - TOP_RIGHT = 4
    TX only Mainboard -> all brushless  
