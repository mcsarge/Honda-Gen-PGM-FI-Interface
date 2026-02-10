#include <pgmfi_dlc.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>

using namespace DLC;


Pgmfi_Dlc::Pgmfi_Dlc(): rx_index(0), msg_available(false) {}

void Pgmfi_Dlc::begin(uint8_t rx_pin, uint8_t tx_pin) {
    this->rx_pin = rx_pin;
    this->tx_pin = tx_pin;

    pinMode(rx_pin, INPUT);
    pinMode(tx_pin, OUTPUT);

    //Serial1.begin(UART_BAUD, SERIAL_8N1, rx_pin, tx_pin, true);
    Serial1.begin(UART_BAUD, SERIAL_8N1, rx_pin, tx_pin, false); //temporary
}

void Pgmfi_Dlc::loop(void) {
    if (!Serial1.available()) {
        return;
    }
    while (true) {
        int current_byte = Serial1.read();
        if (current_byte == -1) {
            return;
        }
        
        uint8_t byte = (uint8_t)current_byte;

        if (byte == VT_MSG_START) {
            rx_index = 0;
            return;
        } else if(byte == VT_MSG_END) {
            recieve_message(rx_buffer, rx_index);
        } else {
            rx_buffer[rx_index] = byte;
            rx_index += 1;
        }
    }
}

/// @brief This method is called when a full message is received. It converts the message from 
/// charater encoded hex to binary, decodes the message, and stores the results in the class variables for retrieval by the data() methods.
/// For instance a message recived might look like this:
/// "CB00000018096F8300602F010301F200067E0B" <- this is a TEXT string
/// when decoded each 2 characters represent 1 byte of binary data. 
/// So the actual message is 18 bytes long and looks like this in hex:
/// 0xCB 0x00 0x00 0x00 0x18 0x09 0x6F 0x83 0x00 0x60 0x2F 0x01 0x03 0x01 0xF2 0x00 0x06 0x7E 0x0B
/// the last byte is a checksum.
/// Calculate it in Pyton like this:
///
/// def calculate_checksum(bytes_list):  # bytes_list = list of int (0–255)
///    xor = 0
///    for b in bytes_list[:-1]:        # all except the last (checksum) position
///        xor ^= b
///    return xor
///
/// @param msg a text string 
/// @param len the length of the message in bytes. This should be the length of the charater encoded hex message excluding the start and end terminal characters. So it should be an even number.
void Pgmfi_Dlc::recieve_message(uint8_t * msg, size_t len) {
    
    Serial.write("RX: ");
    Serial.write(msg, len);
    Serial.write("\n");
    if (len % 2 != 0)
        // It must be dividible by 2 to be a valid character encoded hex message .
        return;
    // This should be the message using charater encoded Hex excluding the terminal begin/end.

    //convert this TEXT to bytes.
    size_t binary_len = len / 2;
    uint8_t binary_msg[binary_len];
    char * msg_ptr = (char*)msg; //set the point to the first charater of the string.
    // convert it from hex to binary
    for (size_t i = 0; i < binary_len; i++) {
        // Use sscanf to convert the two-character hex string into the unsigned char
        // "%2hhx" reads exactly two hexadecimal characters
        sscanf(msg_ptr, "%2hhX", &binary_msg[i]);//convert the 2 text characters to a byte and store it in the binary message array.
        msg_ptr += 2; //skip forward 2 places to the next hex byte in the string.
    }

    //OK, we have a message in binary format, lets decode it.
    QueryType query_type;
    if (!PGMFI_Decoder::decode_msg_type(binary_msg, binary_len, query_type))
        return;
    
    bool success = false;
    
    //Now based on the type, try to decode the message.
    switch (query_type) {
        case QueryType::T_ECU_Info1:
            success = PGMFI_Decoder::decode(binary_msg, binary_len, ecu_info1);
            break;
        case QueryType::T_ECU_Info2:
            success = PGMFI_Decoder::decode(binary_msg, binary_len, ecu_info2);
            break;
        case QueryType::T_INV_Master:
            success = PGMFI_Decoder::decode(binary_msg, binary_len, inv_master);
            break;
        case QueryType::T_INV_Slave:
            success = PGMFI_Decoder::decode(binary_msg, binary_len, inv_slave);
            break;
        default:
            break;
    }

    if (!success){
        Serial.println("Failed to decode message");
        return;
    }
    
    // Record the results so they can be obtained by a call to the data methods.
    msg_available_type = query_type;
    msg_available = true;
}

void Pgmfi_Dlc::send_message(uint8_t * msg, size_t len) {
    // Takes a binary string and converts it to Hex and sents it as a terminal message.
    // bin to ascii = *2. + a header, end, and null terminator.
    char tx_buff[(len*2)+3];

    tx_buff[0] = VT_MSG_START;
    size_t index = 1;

    for(size_t i = 0; i < len; i++) {
        // convert it to hex.
        sprintf(tx_buff + index, "%02X", msg[i]);
        index += 2;
    }
    tx_buff[index++] = VT_MSG_END;
    tx_buff[index] = 0x00;

    Serial1.write(tx_buff);
}

void Pgmfi_Dlc::query(QueryType type) {
    uint8_t msg[QUERY_SIZE];
    msg[0] = QUERY_START;
    msg[1] = type;
    msg[2] = 0x00;

    // The last byte i think is a checksum but i dunno. Would be nice to figure that out.
    switch (type) {
        case QueryType::T_ECU_Info1:
            msg[3] = 0x01;
            break;
        case QueryType::T_ECU_Info2:
            msg[3] = 0x02;
            break;
        case QueryType::T_INV_Master:
            msg[3] = 0x00;
            break;
        case QueryType::T_INV_Slave:
            msg[3] = 0x03;
            break;
        default:
            break;
    }

    send_message(msg, QUERY_SIZE);
}

bool Pgmfi_Dlc::available(QueryType type) {
    if (msg_available && msg_available_type == type)
        return true;
    return false;
}

bool Pgmfi_Dlc::data(ECU_Info1 &ecu) {
    if (!(msg_available && msg_available_type == QueryType::T_ECU_Info1))
        return false;
    
    msg_available = false;
    ecu = ecu_info1;
    return true;
}

bool Pgmfi_Dlc::data(ECU_Info2 &ecu) {
    if (!(msg_available && msg_available_type == QueryType::T_ECU_Info2))
        return false;
    
    msg_available = false;
    ecu = ecu_info2;
    return true;
}

bool Pgmfi_Dlc::data(Inverter_Master &inv) {
    if (!(msg_available && msg_available_type == QueryType::T_INV_Master))
        return false;
    
    msg_available = false;
    inv = inv_master;
    return true;
}

bool Pgmfi_Dlc::data(Inverter_Slave &inv) {
    if (!(msg_available && msg_available_type == QueryType::T_INV_Slave))
        return false;
    
    msg_available = false;
    inv = inv_slave;
    return true;
}



Pgmfi_Dlc DLC::dlc_interface;
