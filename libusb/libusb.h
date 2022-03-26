
#ifndef CY_LIBUSB_H
#define CY_LIBUSB_H

/**
 * @brief
 * @details
 */
void libusb_init();

/**
 * @brief
 * @details
 *
 * @param [in] data Reference to data stream to be sent
 * @param [in] size Length of the data stream
 *
 */
void lusb_write(const uint8_t *data, uint16_t size);

#endif //CY_LIBUSB_H
