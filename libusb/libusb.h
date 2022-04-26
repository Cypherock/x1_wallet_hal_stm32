
#ifndef CY_LIBUSB_H
#define CY_LIBUSB_H

/**
 * @brief
 * @details
 */
void libusb_init();

typedef void (*libusb_parser_fptr_t) (const uint8_t *data, const uint16_t len);

/**
 * @brief
 * @details
 *
 * @param [in] data Reference to data stream to be sent
 * @param [in] size Length of the data stream
 *
 */
void lusb_write(const uint8_t *data, uint16_t size);

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
