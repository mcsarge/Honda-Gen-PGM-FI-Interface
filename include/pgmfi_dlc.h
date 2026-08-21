#ifndef PGMFI_DLC_H
#define PGMFI_DLC_H

#include <stdint.h>
#include <pgmfi_decoder.h>

namespace DLC {

#define UART_BAUD 9600
#define QUERY_SIZE 4
#define RX_BUFF_SIZE 256
#define VT_MSG_START 0x01
#define VT_MSG_END 0x04
// Max number of raw received bytes included in a raw-message-logging callback message.
#define RAW_MESSAGE_LOG_BYTES 7

// Generic message callback, used to notify the application of library events
// (e.g. raw received data, and potentially future error/status messages).
typedef void (*MessageCallback)(const char* message);

// Handles the physical interface of the Honda Data Link Connector port.
class Pgmfi_Dlc {
    public:
        Pgmfi_Dlc();
        void begin(uint8_t rx_pin, uint8_t tx_pin);
        void loop(void);
        void query(QueryType type);
        bool available(QueryType type);
        bool data(ECU_Info1 &ecu);
        bool data(ECU_Info2 &ecu);
        bool data(Inverter_Master &inv);
        bool data(Inverter_Slave &inv);
        void set_message_callback(MessageCallback cb);
        void set_raw_message_logging(bool enabled);
    protected:
        void send_message(uint8_t * msg, size_t len);
        void recieve_message(uint8_t * msg, size_t len);
        void log_raw_message(const char * direction, uint8_t * data, size_t len);
        uint8_t rx_pin;
        uint8_t tx_pin;
        uint8_t rx_buffer[RX_BUFF_SIZE];
        size_t rx_index;
        ECU_Info1 ecu_info1;
        ECU_Info2 ecu_info2;
        Inverter_Master inv_master;
        Inverter_Slave inv_slave;
        bool msg_available;
        QueryType msg_available_type;
        MessageCallback message_cb;
        bool raw_message_logging_enabled;

};

extern Pgmfi_Dlc dlc_interface;

};
#endif