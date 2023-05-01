
#ifndef CY_LIBUSB_H
#define CY_LIBUSB_H

/**
 * @brief Enum to be used by application to identify interface from which data was recieved and 
 * interface to which, data should be sent.
 */
typedef enum{
    COMM_LIBUSB__UNDEFINED = 0,
    COMM_LIBUSB__CDC,
    COMM_LIBUSB__HID,
    COMM_LIBUSB__WEBUSB,
}comm_libusb__interface_e;

/**
 * @brief
 * @details
 */
void libusb_init();

/**
 * @brief   Function prototype for received data parser 
 * @params  *data: Data received by USB
 *          len: length of data received
 *          interface: The USB interface on which data is received. @ref comm_libusb__interface_e
 */
typedef void (*libusb_parser_fptr_t) (const uint8_t *data, const uint16_t len, comm_libusb__interface_e interface);

/**
 * @brief
 * @details
 *
 * @param [in] data Reference to data stream to be sent
 * @param [in] size Length of the data stream
 * @param [in] interface USB interface to which the data stream should be sent. @ref comm_libusb__interface_e
 *
 */
void lusb_write(const uint8_t *data, uint16_t size, comm_libusb__interface_e interface);

/**
 * @brief Register a parser to be called when a new packet data is received.
 * @details The parser is called with the data stream and the length of the data stream.
 *
 * @param func Pointer to the parser function
 *
 * @see libusb_parser_fptr_t, lusb_write(), libusb_init()
 */
void lusb_register_parserFunction(libusb_parser_fptr_t func);
#endif //CY_LIBUSB_H
