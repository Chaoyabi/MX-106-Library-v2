#include "mbed.h"
#include "Mx106v2.h"
#include "CRC_cal.h"

unsigned char   Instruction_Packet_Array[35];   // Array to hold instruction packet data 
unsigned char   Status_Packet_Array[15];        // Array to hold returned status packet data

// Because printing the packet directly maybe clean the data in UART,
// We need to copy packet into debug_packet
// and print the debug_packet after reading all data in packet we need.
unsigned char   debug_Instruction_Packet_Array[35];   // Array to debug instruction packet data
unsigned char   debug_Status_Packet_Array[15];        // Array to debug status packet data

unsigned short  packet_length = 0;
unsigned short  crc = 0;

unsigned char   Status_Return_Value = READ;     // Status packet return states ( NON , READ , ALL )

//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------
void DynamixelClass_v2::debugInstructionframe(void) {
    for (int i = 0; i < 10; i++) {
        debug_Instruction_Packet_Array[i] = Instruction_Packet_Array[i];
    }
    //    for (int i = 0; i < 10; i++) printf("%x, ",debug_Instruction_Packet_Array[i]);
    //    printf("\r\nyou transmit!\r\n"); 
}

void DynamixelClass_v2::debugStatusframe(void) {
    for (int i = 0; i < 15; i++) {
        debug_Status_Packet_Array[i] = Status_Packet_Array[i];
    }
    for (int i = 0; i < 15; i++) printf("%x, ", debug_Status_Packet_Array[i]);
    printf("\r\nyou recieved!\r\n");
}


void DynamixelClass_v2::transmitInstructionPacket(void) {                           // Transmit instruction packet to Dynamixel
    bool tx_complete = 0;

    UART4->SR &= 0xFFFFFFBF;
    servoSerialDir->write(1);                                                   // Set TX Buffer pin to HIGH    
    
    for (int i = 0; i < packet_length + 7; i++) {
        servoSerial->putc(Instruction_Packet_Array[i]);                         // Write Instuction & Parameters (if there is any) & check sum to serial
    }
    do {
        tx_complete = (UART4->SR >> 6) & 0x00000001;
    } while (tx_complete == 0);
    
    servoSerialDir->write(0);                                                   // Set TX Buffer pin to LOW after data has been sent
//  debugInstructionframe();
}


void DynamixelClass_v2::readStatusPacket(void) {
    static unsigned char InBuff[20];
    unsigned char i, j, RxState;

    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;
    Status_Packet_Array[3] = 0x00;
    i = 0; RxState = 0; j = 0; InBuff[0] = 0;

    while (RxState < 3) {
        // Wait for " header + header + header + reserved " RX data 
        if (servoSerial->readable()) {
            switch (RxState) {
            case 0:
            {
                InBuff[i] = servoSerial->getc();
                if (InBuff[0] == 0xFF) {
                    i++;                                                // When we have the first header we starts to inc data to inbuffer
                    if (i > 3) { RxState++; }                           //read header
                }
            }
            break;

            case 1:
            { // Read ID, frame length, instruction, error 
                if ((InBuff[j] == 0xFF) && (InBuff[j + 1] == 0xFF) && (InBuff[j + 2] == 0xFD) && (InBuff[j + 3] == 0x00)) {
                    // checkes that we have got the buffer
                    for (j = 0; j < 5; j++) {
                        Status_Packet_Array[j] = servoSerial->getc();
                    }
                    packet_length = (Status_Packet_Array[2] << 8) | Status_Packet_Array[1];
                    RxState++;//led2->write(0);
                }
            }
            break;

            case 2:
            { // Read param, crc
                for (j = 5; j < packet_length + 3; j++) {
                    Status_Packet_Array[j] = servoSerial->getc();
                }
                RxState++;
            }
            break;
            } // switch 
        } // if
    } //while..

//    debugStatusframe();
}


//------------------------------------------------------------------------------
// Public Methods 
//------------------------------------------------------------------------------

DynamixelClass_v2::DynamixelClass_v2(int baud, PinName D_Pin, PinName tx, PinName rx) {
    servoSerial = new Serial(tx, rx);
    servoSerial->baud(baud);
    servoSerialDir = new DigitalOut(D_Pin);
    servoSerialDir->write(0);

    Instruction_Packet_Array[0] = HEADER1;
    Instruction_Packet_Array[1] = HEADER2;
    Instruction_Packet_Array[2] = HEADER3;
    Instruction_Packet_Array[3] = 0x00;
}


DynamixelClass_v2::~DynamixelClass_v2() {
    if (servoSerial != NULL) delete servoSerial;
    if (servoSerialDir != NULL) delete servoSerialDir;
}


unsigned int DynamixelClass_v2::ReadPosition(unsigned char ID) {
    packet_length = 0x0007;

    Instruction_Packet_Array[4] = ID;
    Instruction_Packet_Array[5] = READ_POS_LENGTH;
    Instruction_Packet_Array[6] = 0x00;
    Instruction_Packet_Array[7] = COMMAND_READ_DATA;
    Instruction_Packet_Array[8] = RAM_PRESENT_POSITION_1;
    Instruction_Packet_Array[9] = 0x00;
    Instruction_Packet_Array[10] = READ_FOUR_BYTE_LENGTH;
    Instruction_Packet_Array[11] = 0x00;

    crc = update_crc(crc, Instruction_Packet_Array, packet_length + 5);

    Instruction_Packet_Array[12] = crc & 0x00FF;
    Instruction_Packet_Array[13] = (crc >> 8) & 0x00FF;

    if (servoSerial->readable()) {
        while (servoSerial->readable()) servoSerial->getc(); //empty buffer
    }

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[4] == 0) {               // If there is no status packet error return value
        return (Status_Packet_Array[8] << 24 | Status_Packet_Array[7] << 16 | Status_Packet_Array[6] << 8 | Status_Packet_Array[5]); // Return present position value
    }
    else {
        return (Status_Packet_Array[4]);            // If there is a error Returns error value
    }
}


/*
unsigned int DynamixelClass_v2::SyncReadPosition(unsigned char *ID_set) {
    packet_length = 0x0007;

    Instruction_Packet_Array[4] = ID;
    Instruction_Packet_Array[5] = READ_POS_LENGTH;
    Instruction_Packet_Array[6] = 0x00;
    Instruction_Packet_Array[7] = COMMAND_READ_DATA;
    Instruction_Packet_Array[8] = RAM_PRESENT_POSITION_1;
    Instruction_Packet_Array[9] = 0x00;
    Instruction_Packet_Array[10] = READ_FOUR_BYTE_LENGTH;
    Instruction_Packet_Array[11] = 0x00;

    crc = update_crc(crc, Instruction_Packet_Array, packet_length + 5);

    Instruction_Packet_Array[12] = crc & 0x00FF;
    Instruction_Packet_Array[13] = (crc >> 8) & 0x00FF;

    if (servoSerial->readable()) {
        while (servoSerial->readable()) servoSerial->getc(); //empty buffer
    }

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[4] == 0) {               // If there is no status packet error return value
        return (Status_Packet_Array[8] << 24 | Status_Packet_Array[7] << 16 | Status_Packet_Array[6] << 8 | Status_Packet_Array[5]); // Return present position value
    }
    else {
        return (Status_Packet_Array[4]);            // If there is a error Returns error value
    }
}
*/


int DynamixelClass_v2::test() {
    int b;
    b = test_get_a();
    return b;
}
