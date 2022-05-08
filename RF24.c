/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"
#include <string.h>

/****************************************************************************/

void rf24_init(RF24 *rf)
{
    rf->payload_size = 32;
    rf->_is_p_variant = 0;
    rf->_is_p0_rx = 0;
    rf->addr_width = 5;
    rf->dynamic_payloads_enabled = 1;
    rf->csDelay = 5;
    rf->pipe0_reading_address[0] = 0;
    rf->spi_speed = RF24_SPI_SPEED;
}

/****************************************************************************/

void rf24_beginTransaction(RF24* rf)
{
    rf->set_csn_pin(0);
    rf->delay_us(rf->csDelay);
}

/****************************************************************************/

void rf24_endTransaction(RF24* rf)
{
    rf->set_csn_pin(1);
    rf->delay_us(rf->csDelay);
}

/****************************************************************************/

void rf24_read_register_buffer(RF24* rf, unsigned char reg, unsigned char* buf, unsigned char len)
{
    rf24_beginTransaction(rf);
    
    rf->status = rf->spi_transfer(R_REGISTER | reg);
    while (len--) { *buf++ = rf->spi_transfer(0xFF); }

    rf24_endTransaction(rf);
}

/****************************************************************************/

unsigned char rf24_read_register(RF24* rf, unsigned char reg)
{
    unsigned char result;
    rf24_beginTransaction(rf);

    rf->status = rf->spi_transfer(R_REGISTER | reg);
    result = rf->spi_transfer(0xff);

    rf24_endTransaction(rf);
    return result;
}

/****************************************************************************/
void rf24_write_register_buffer(RF24* rf, unsigned char reg, const unsigned char* buf, unsigned char len)
{
    rf24_beginTransaction(rf);
    
    rf->status = rf->spi_transfer(W_REGISTER | reg);
    while (len--) { rf->spi_transfer(*buf++); }

    rf24_endTransaction(rf);
}

/****************************************************************************/

void rf24_write_register(RF24* rf, unsigned char reg, unsigned char value, unsigned char is_cmd_only)
{
    rf24_beginTransaction(rf);
    
    if(is_cmd_only)
    {
        rf->status = rf->spi_transfer(W_REGISTER | reg);
    }
    else
    {
        rf->status = rf->spi_transfer(W_REGISTER | reg);
        rf->spi_transfer(value);
    }

    rf24_endTransaction(rf);
}

/****************************************************************************/

void rf24_write_payload(RF24* rf, const void* buf, unsigned char len, const unsigned char writeType)
{
    const unsigned char* current = (const unsigned char *)(buf);

    unsigned char blank_len = !len ? 1 : 0;
    if (!rf->dynamic_payloads_enabled) {
        len = rf24_min(len, rf->payload_size);
        blank_len = (unsigned char)(rf->payload_size - len);
    }
    else {
        len = rf24_min(len, (unsigned char)(32));
    }

    rf24_beginTransaction(rf);

    rf->status = rf->spi_transfer(writeType);
    while (len--) { rf->spi_transfer(*current++); }

    while (blank_len--) { rf->spi_transfer(0); }

    rf24_endTransaction(rf);
}

/****************************************************************************/

void rf24_read_payload(RF24* rf, void* buf, unsigned char len)
{
    unsigned char* current = (unsigned char*)(buf);

    unsigned char blank_len = 0;
    if (!rf->dynamic_payloads_enabled) {
        len = rf24_min(len, rf->payload_size);
        blank_len = (unsigned char)(rf->payload_size - len);
    }
    else {
        len = rf24_min(len, (unsigned char)(32));
    }

    rf24_beginTransaction(rf);

    rf->status = rf->spi_transfer(R_RX_PAYLOAD);
    while (len--) { *current++ = rf->spi_transfer(0xFF); }
    while (blank_len--) { rf->spi_transfer(0xff); }

    rf24_endTransaction(rf);
}

/****************************************************************************/

unsigned char rf24_flush_rx(RF24* rf)
{
    rf24_write_register(rf, FLUSH_RX, RF24_NOP, 1);
    return rf->status;
}

/****************************************************************************/

unsigned char rf24_flush_tx(RF24* rf)
{
    rf24_write_register(rf, FLUSH_TX, RF24_NOP, 1);
    return rf->status;
}

/****************************************************************************/

unsigned char rf24_get_status(RF24* rf)
{
    rf24_write_register(rf, RF24_NOP, RF24_NOP, 1);
    return rf->status;
}

/****************************************************************************/

void rf24_setChannel(RF24* rf, unsigned char channel)
{
    const unsigned char max_channel = 125;
    rf24_write_register(rf, RF_CH, rf24_min(channel, max_channel), 0);
}

unsigned char rf24_getChannel(RF24* rf)
{
    return rf24_read_register(rf, RF_CH);
}

/****************************************************************************/

void rf24_setPayloadSize(RF24* rf, unsigned char size)
{
    // payload size must be in range [1, 32]
    rf->payload_size = (unsigned char)(rf24_max(1, rf24_min(32, size)));

    // write static payload size setting for all pipes
    for (unsigned char i = 0; i < 6; ++i)
        rf24_write_register(rf, (unsigned char)(RX_PW_P0 + i), rf->payload_size, 0);
}

/****************************************************************************/

unsigned char rf24_getPayloadSize(RF24* rf)
{
    return rf->payload_size;
}

/****************************************************************************/

void rf24_encodeRadioDetails(RF24* rf, unsigned char *encoded_details)
{
    unsigned char end = FEATURE + 1;
    for (unsigned char i = NRF_CONFIG; i < end; ++i) {
        if (i == RX_ADDR_P0 || i == RX_ADDR_P1 || i == TX_ADDR) {
            // get 40-bit registers
            rf24_read_register_buffer(rf, i, encoded_details, 5);
            encoded_details += 5;
        }
        else if (i != 0x18 && i != 0x19 && i != 0x1a && i != 0x1b) { // skip undocumented registers
            // get single byte registers
            *encoded_details++ = rf24_read_register(rf, i);
        }
    }
}

/****************************************************************************/

unsigned char rf24_begin(RF24* rf)
{
    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    rf->delay_ms(5);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See datasheet for a more complete explanation.
    rf24_setRetries(rf, 5, 15);

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware. Since this value occupies the same register as the PA level value, set
    // the PA level to MAX
    rf24_setRadiation(rf, RF24_PA_MAX, RF24_1MBPS, 1); // LNA enabled by default

    // detect if is a plus variant & use old toggle features command accordingly
    unsigned char before_toggle = rf24_read_register(rf, FEATURE);
    rf24_toggle_features(rf);
    unsigned char after_toggle = rf24_read_register(rf, FEATURE);
    rf->_is_p_variant = before_toggle == after_toggle;
    if (after_toggle){
        if (rf->_is_p_variant){
            // module did not experience power-on-reset (#401)
            rf24_toggle_features(rf);
        }
        // allow use of multicast parameter and dynamic payloads by default
        rf24_write_register(rf, FEATURE, 0, 0);
    }
    rf->ack_payloads_enabled = 0;               // ack payloads disabled by default
    rf24_write_register(rf, DYNPD, 0, 0);       // disable dynamic payloads by default (for all pipes)
    rf->dynamic_payloads_enabled = 0;
    rf24_write_register(rf, EN_AA, 0x3F, 0);    // enable auto-ack on all pipes
    rf24_write_register(rf, EN_RXADDR, 3, 0);   // only open RX pipes 0 & 1
    rf24_setPayloadSize(rf, 32);                // set static payload size to 32 (max) bytes by default
    rf24_setAddressWidth(rf, 5);                // set default address length to (max) 5 bytes

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    rf24_setChannel(rf, 76);

    // Reset current status
    // Notice reset and flush is the last thing we do
    rf24_write_register(rf, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), 0);

    // Flush buffers
    rf24_flush_rx(rf);
    rf24_flush_tx(rf);

    // Clear CONFIG register:
    //      Reflect all IRQ events on IRQ pin
    //      Enable PTX
    //      Power Up
    //      16-bit CRC (CRC required by auto-ack)
    // Do not write CE high so radio will remain in standby I mode
    // PTX should use only 22uA of power
    rf24_write_register(rf, NRF_CONFIG, (_BV(EN_CRC) | _BV(CRCO)), 0);
    rf->config_reg = rf24_read_register(rf, NRF_CONFIG);

    rf24_powerUp(rf);

    // if config is not set correctly then there was a bad response from module
    return rf->config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)) ? 1 : 0;
}

/****************************************************************************/

unsigned char rf24_isChipConnected(RF24* rf)
{
    return rf24_read_register(rf, SETUP_AW) == (rf->addr_width - 2);
}

/****************************************************************************/

void rf24_startListening(RF24* rf)
{
    rf24_powerUp(rf);

    rf->config_reg |= _BV(PRIM_RX);
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);
    rf24_write_register(rf, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), 0);
    rf->set_ce_pin(1);

    // Restore the pipe0 address, if exists
    if (rf->_is_p0_rx) {
        rf24_write_register_buffer(rf, RX_ADDR_P0, rf->pipe0_reading_address, rf->addr_width);
    } else {
        rf24_closeReadingPipe(rf, 0);
    }
}

/****************************************************************************/
static const unsigned char child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2,
                                                    ERX_P3, ERX_P4, ERX_P5};

void rf24_stopListening(RF24* rf)
{
    rf->set_ce_pin(0);

    rf->delay_ms(rf->txDelay);
    if (rf->ack_payloads_enabled){
        rf24_flush_tx(rf);
    }

    rf->config_reg = (unsigned char)(rf->config_reg & ~_BV(PRIM_RX));
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);

    rf24_write_register(rf, EN_RXADDR, (unsigned char)(rf24_read_register(rf, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))), 0); // Enable RX on pipe0
}

/****************************************************************************/

void rf24_powerDown(RF24* rf)
{
    rf->set_ce_pin(0); // Guarantee CE is low on powerDown
    rf->config_reg = (unsigned char)(rf->config_reg & ~_BV(PWR_UP));
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void rf24_powerUp(RF24* rf)
{
    // if not powered up then power up and wait for the radio to initialize
    if (!(rf->config_reg & _BV(PWR_UP))) {
        rf->config_reg |= _BV(PWR_UP);
        rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        rf->delay_ms(RF24_POWERUP_DELAY);
    }
}

/******************************************************************/

unsigned char rf24_write(RF24* rf, const void* buf, unsigned char len, const unsigned char multicast)
{
    //Start Writing
    rf24_startFastWrite(rf, buf, len, multicast, 1);

    while (!(rf24_get_status(rf) & (_BV(TX_DS) | _BV(MAX_RT)))) {
    }

    rf->set_ce_pin(0);

    rf24_write_register(rf, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), 0);

    //Max retries exceeded
    if (rf->status & _BV(MAX_RT)) {
        rf24_flush_tx(rf); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
        return 0;
    }
    //TX OK 1 or 0
    return 1;
}

/****************************************************************************/

//For general use, the interrupt flags are not important to clear
unsigned char rf24_writeBlocking(RF24* rf, const void* buf, unsigned char len, unsigned char timeout)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    unsigned int timer = rf->millis();                  // Get the time that the payload transmission started

    while ((rf24_get_status(rf) & (_BV(TX_FULL)))) {    // Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

        if (rf->status & _BV(MAX_RT)) {                 // If MAX Retries have been reached
            rf24_reUseTX(rf);                           // Set re-transmit and clear the MAX_RT interrupt flag
            if (rf->millis() - timer > timeout) {
                return 0;                               // If this payload has exceeded the user-defined timeout, exit and return 0
            }
        }
    }

    //Start Writing
    rf24_startFastWrite(rf, buf, len, 0, 1);            // Write the payload if a buffer is clear

    return 1;                                           // Return 1 to indicate successful transmission
}

/****************************************************************************/

void rf24_reUseTX(RF24* rf)
{
    rf24_write_register(rf, NRF_STATUS, _BV(MAX_RT), 0);  // Clear max retry flag
    rf24_write_register(rf, REUSE_TX_PL, RF24_NOP, 1);
    rf->set_ce_pin(0); // Re-Transfer packet
    rf->delay_us(2);
    rf->set_ce_pin(1);
}

/****************************************************************************/

unsigned char rf24_writeFast(RF24* rf, const void* buf, unsigned char len, const unsigned char multicast)
{
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retries and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high

    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((rf24_get_status(rf) & (_BV(TX_FULL)))) {
        if (rf->status & _BV(MAX_RT)) {
            return 0;                                        //Return 0. The previous payload has not been retransmitted
            // From the user perspective, if you get a 0, just keep trying to send the same payload
        }
    }
    rf24_startFastWrite(rf, buf, len, multicast, 1);                     // Start Writing

    return 1;
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void rf24_startFastWrite(RF24* rf, const void* buf, unsigned char len, const unsigned char multicast, unsigned char startTx)
{//TMRh20
    rf24_write_payload(rf, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    if (startTx) {
        rf->set_ce_pin(1);
    }
}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests

unsigned char rf24_startWrite(RF24* rf, const void* buf, unsigned char len, const unsigned char multicast)
{
    // Send the payload
    rf24_write_payload(rf, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    rf->set_ce_pin(1);
    rf->delay_us(10);
    rf->set_ce_pin(0);
    return !(rf->status & _BV(TX_FULL));
}

/****************************************************************************/

unsigned char rf24_rxFifoFull(RF24* rf)
{
    return rf24_read_register(rf, FIFO_STATUS) & _BV(RX_FULL);
}

/****************************************************************************/

unsigned char rf24_txStandBy(RF24* rf)
{
    while (!(rf24_read_register(rf, FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (rf->status & _BV(MAX_RT)) {
            rf24_write_register(rf, NRF_STATUS, _BV(MAX_RT), 0);
            rf->set_ce_pin(0);
            rf24_flush_tx(rf);    //Non blocking, flush the data
            return 0;
        }
    }

    rf->set_ce_pin(0);               //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

unsigned char rf24_txStandByWithTimeout(RF24* rf, unsigned int timeout, unsigned char startTx)
{
    if (startTx) {
        rf24_stopListening(rf);
        rf->set_ce_pin(1);
    }
    unsigned int start = rf->millis();

    while (!(rf24_read_register(rf, FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (rf->status & _BV(MAX_RT)) {
            rf24_write_register(rf, NRF_STATUS, _BV(MAX_RT), 0);
            rf->set_ce_pin(0); // Set re-transmit
            rf->set_ce_pin(0);
            rf->set_ce_pin(1);
            if (rf->millis() - start >= timeout) {
                rf->set_ce_pin(0);
                rf24_flush_tx(rf);
                return 0;
            }
        }
    }

    rf->set_ce_pin(0);  //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

void rf24_maskIRQ(RF24* rf, unsigned char tx_ok, unsigned char tx_fail, unsigned char rx_ready)
{
     /* clear the interrupt flags */
    rf->config_reg = (unsigned char)(rf->config_reg & ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR));
    /* set the specified interrupt flags */
    rf->config_reg = (unsigned char)(rf->config_reg | tx_fail << MASK_MAX_RT | tx_ok << MASK_TX_DS | rx_ready << MASK_RX_DR);
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);
}

/****************************************************************************/

unsigned char rf24_getDynamicPayloadSize(RF24* rf)
{
    unsigned char result = rf24_read_register(rf, R_RX_PL_WID);

    if (result > 32) {
        rf24_flush_rx(rf);
        rf->delay_ms(2);
        return 0;
    }
    return result;
}

/****************************************************************************/

unsigned char rf24_available(RF24* rf, unsigned char* pipe_num)
{
    // get implied RX FIFO empty flag from status byte
    unsigned char pipe = (rf24_get_status(rf) >> RX_P_NO) & 0x07;
    if (pipe > 5)
        return 0;

    // If the caller wants the pipe number, include that
    if (pipe_num)
        *pipe_num = pipe;

    return 1;
}

/****************************************************************************/

void rf24_read(RF24* rf, void* buf, unsigned char len)
{
    // Fetch the payload
    rf24_read_payload(rf, buf, len);

    //Clear the only applicable interrupt flags
    rf24_write_register(rf, NRF_STATUS, _BV(RX_DR), 0);
}

/****************************************************************************/

void rf24_whatHappened(RF24* rf, unsigned char *tx_ok, unsigned char *tx_fail, unsigned char *rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    rf24_write_register(rf, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT), 0);

    // Report to the user what happened
    *tx_ok = rf->status & _BV(TX_DS);
    *tx_fail = rf->status & _BV(MAX_RT);
    *rx_ready = rf->status & _BV(RX_DR);
}

/****************************************************************************/

void rf24_openWritingPipe(RF24* rf, const unsigned char* address)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    rf24_write_register_buffer(rf, RX_ADDR_P0, address, rf->addr_width);
    rf24_write_register_buffer(rf, TX_ADDR, address, rf->addr_width);
}

/****************************************************************************/
static const unsigned char child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2,
                                             RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

void rf24_openReadingPipe(RF24* rf, unsigned char child, const unsigned char* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(rf->pipe0_reading_address, address, rf->addr_width);
        rf->_is_p0_rx = 1;
    }

    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            rf24_write_register_buffer(rf, pgm_read_byte(&child_pipe[child]), address, rf->addr_width);
        } else {
            rf24_write_register_buffer(rf, pgm_read_byte(&child_pipe[child]), address, 1);
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        rf24_write_register(rf, EN_RXADDR, (unsigned char)(rf24_read_register(rf, EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child]))), 0);
    }
}

/****************************************************************************/

void rf24_setAddressWidth(RF24* rf, unsigned char a_width)
{
    a_width = a_width - 2;
    if (a_width) {
        rf24_write_register(rf, SETUP_AW, a_width % 4, 0);
        rf->addr_width = (a_width % 4) + 2;
    } else {
        rf24_write_register(rf, SETUP_AW, 0, 0);
        rf->addr_width = 2;
    }
}

/****************************************************************************/

void rf24_closeReadingPipe(RF24* rf, unsigned char pipe)
{
    rf24_write_register(rf, EN_RXADDR, (rf24_read_register(rf, EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe]))), 0);
    if (!pipe) {
        // keep track of pipe 0's RX state to avoid null vs 0 in addr cache
        rf->_is_p0_rx = 0;
    }
}

/****************************************************************************/

void rf24_toggle_features(RF24* rf)
{
    rf24_beginTransaction(rf);

    rf->status = rf->spi_transfer(ACTIVATE);
    rf->spi_transfer(0x73);

    rf24_endTransaction(rf);
}

/****************************************************************************/

void rf24_enableDynamicPayloads(RF24* rf)
{
    // Enable dynamic payload throughout the system

    //toggle_features();
    rf24_write_register(rf, FEATURE, rf24_read_register(rf, FEATURE) | _BV(EN_DPL), 0);

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    rf24_write_register(rf, DYNPD, rf24_read_register(rf, DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0), 0);

    rf->dynamic_payloads_enabled = 1;
}

/****************************************************************************/

void rf24_disableDynamicPayloads(RF24* rf)
{
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads

    //toggle_features();
    rf24_write_register(rf, FEATURE, 0, 0);

    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    rf24_write_register(rf, DYNPD, 0, 0);

    rf->dynamic_payloads_enabled = 0;
    rf->ack_payloads_enabled = 0;
}

/****************************************************************************/

void rf24_enableAckPayload(RF24* rf)
{
     // enable ack payloads and dynamic payload features

    if (!rf->ack_payloads_enabled){
        rf24_write_register(rf, FEATURE, rf24_read_register(rf, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL), 0);

        // Enable dynamic payload on pipes 0 & 1
        rf24_write_register(rf, DYNPD, rf24_read_register(rf, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0), 0);
        rf->dynamic_payloads_enabled = 1;
        rf->ack_payloads_enabled = 1;
    }
}

/****************************************************************************/

void rf24_disableAckPayload(RF24* rf)
{
    // disable ack payloads (leave dynamic payload features as is)
    if (rf->ack_payloads_enabled){
        rf24_write_register(rf, FEATURE, (rf24_read_register(rf, FEATURE) & ~_BV(EN_ACK_PAY)), 0);

        rf->ack_payloads_enabled = 0;
    }
}

/****************************************************************************/

void rf24_enableDynamicAck(RF24* rf)
{
    //
    // enable dynamic ack features
    //
    //toggle_features();
    rf24_write_register(rf, FEATURE, rf24_read_register(rf, FEATURE) | _BV(EN_DYN_ACK), 0);
}

/****************************************************************************/

unsigned char rf24_writeAckPayload(RF24* rf, unsigned char pipe, const void* buf, unsigned char len)
{
    if (rf->ack_payloads_enabled){
        const unsigned char* current = (const unsigned char*)(buf);

        rf24_write_payload(rf, current, len, W_ACK_PAYLOAD | (pipe & 0x07));
        return !(rf->status & _BV(TX_FULL));
    }
    return 0;
}

/****************************************************************************/

unsigned char rf24_isAckPayloadAvailable(RF24* rf)
{
    return rf24_available(rf, NULL);
}

/****************************************************************************/

unsigned char rf24_isPVariant(RF24* rf)
{
    return rf->_is_p_variant;
}

/****************************************************************************/

void rf24_setAutoAckAll(RF24* rf, unsigned char enable)
{
    if (enable){
        rf24_write_register(rf, EN_AA, 0x3F, 0);
    }else{
        rf24_write_register(rf, EN_AA, 0, 0);
        // accommodate ACK payloads feature
        if (rf->ack_payloads_enabled){
            rf24_disableAckPayload(rf);
        }
    }
}

/****************************************************************************/

void rf24_setAutoAck(RF24* rf, unsigned char pipe, unsigned char enable)
{
    if (pipe < 6) {
        unsigned char en_aa = rf24_read_register(rf, EN_AA);
        if (enable) {
            en_aa |= (unsigned char)(_BV(pipe));
        }else{
            en_aa = (unsigned char)(en_aa & ~_BV(pipe));
            if (rf->ack_payloads_enabled && !pipe){
                rf24_disableAckPayload(rf);
            }
        }
        rf24_write_register(rf, EN_AA, en_aa, 0);
    }
}

/****************************************************************************/

unsigned char rf24_testCarrier(RF24* rf)
{
    return (rf24_read_register(rf, CD) & 1);
}

/****************************************************************************/

unsigned char rf24_testRPD(RF24* rf)
{
    return (rf24_read_register(rf, RPD) & 1);
}

/****************************************************************************/

void rf24_setPALevel(RF24* rf, unsigned char level, unsigned char lnaEnable)
{
    unsigned char setup = rf24_read_register(rf, RF_SETUP) & (0xF8);
    setup |= _pa_level_reg_value(level, lnaEnable);
    rf24_write_register(rf, RF_SETUP, setup, 0);
}

/****************************************************************************/

unsigned char rf24_getPALevel(RF24* rf)
{
    return (rf24_read_register(rf, RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

unsigned char rf24_getARC(RF24* rf)
{
    return rf24_read_register(rf, OBSERVE_TX) & 0x0F;
}

/****************************************************************************/

unsigned char rf24_setDataRate(RF24* rf, rf24_datarate_e speed)
{
    unsigned char result = 0;
    unsigned char setup = rf24_read_register(rf, RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default
    setup = (setup & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)));
    setup |= _data_rate_reg_value(rf, speed);

    rf24_write_register(rf, RF_SETUP, setup, 0);

    // Verify our result
    if (rf24_read_register(rf, RF_SETUP) == setup) {
        result = 1;
    }
    return result;
}

/****************************************************************************/

rf24_datarate_e rf24_getDataRate(RF24* rf)
{
    rf24_datarate_e result;
    unsigned char dr = rf24_read_register(rf, RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    } else if (dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    } else {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }
    return result;
}

/****************************************************************************/

void rf24_setCRCLength(RF24* rf, rf24_crclength_e length)
{
    rf->config_reg = (rf->config_reg & ~(_BV(CRCO) | _BV(EN_CRC)));

    // switch uses RAM (evil!)
    if (length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above.
    } else if (length == RF24_CRC_8) {
        rf->config_reg |= _BV(EN_CRC);
    } else {
        rf->config_reg |= _BV(EN_CRC);
        rf->config_reg |= _BV(CRCO);
    }
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);
}

/****************************************************************************/

rf24_crclength_e rf24_getCRCLength(RF24* rf)
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    unsigned char AA = rf24_read_register(rf, EN_AA);
    rf->config_reg = rf24_read_register(rf, NRF_CONFIG);

    if (rf->config_reg & _BV(EN_CRC) || AA) {
        if (rf->config_reg & _BV(CRCO)) {
            result = RF24_CRC_16;
        } else {
            result = RF24_CRC_8;
        }
    }

    return result;
}

/****************************************************************************/

void rf24_disableCRC(RF24* rf)
{
    rf->config_reg = (rf->config_reg & ~_BV(EN_CRC));
    rf24_write_register(rf, NRF_CONFIG, rf->config_reg, 0);
}

/****************************************************************************/

void rf24_setRetries(RF24* rf, unsigned char delay, unsigned char count)
{
    rf24_write_register(rf, SETUP_RETR, (rf24_min(15, delay) << ARD | rf24_min(15, count)), 0);
}

/****************************************************************************/

void rf24_startConstCarrier(RF24* rf, rf24_pa_dbm_e level, unsigned char channel)
{
    rf24_stopListening(rf);
    rf24_write_register(rf, RF_SETUP, rf24_read_register(rf, RF_SETUP) | _BV(CONT_WAVE) | _BV(PLL_LOCK), 0);
    if (rf24_isPVariant(rf)){
        rf24_setAutoAckAll(rf, 0);
        rf24_setRetries(rf, 0, 0);
        unsigned char dummy_buf[32];
        for (unsigned char i = 0; i < 32; ++i)
            dummy_buf[i] = 0xFF;

        // use write_register() instead of openWritingPipe() to bypass
        // truncation of the address with the current RF24::addr_width value
        rf24_write_register_buffer(rf, TX_ADDR, dummy_buf, 5);
        rf24_flush_tx(rf);  // so we can write to top level

        // use write_register() instead of write_payload() to bypass
        // truncation of the payload with the current RF24::payload_size value
        rf24_write_register_buffer(rf, W_TX_PAYLOAD, dummy_buf, 32);

        rf24_disableCRC(rf);
    }
    rf24_setPALevel(rf, level, 1);
    rf24_setChannel(rf, channel);

    rf->set_ce_pin(1);

    if (rf24_isPVariant(rf)){
        rf->delay_ms(1); // datasheet says 1 ms is ok in this instance
        rf->set_ce_pin(0);
        rf24_reUseTX(rf);
    }
}

/****************************************************************************/

void rf24_stopConstCarrier(RF24* rf)
{
    /*
     * A note from the datasheet:
     * Do not use REUSE_TX_PL together with CONT_WAVE=1. When both these
     * registers are set the chip does not react when setting CE low. If
     * however, both registers are set PWR_UP = 0 will turn TX mode off.
     */
    rf24_powerDown(rf);  // per datasheet recommendation (just to be safe)
    rf24_write_register(rf, RF_SETUP, (rf24_read_register(rf, RF_SETUP) & ~_BV(CONT_WAVE) & ~_BV(PLL_LOCK)), 0);
    rf->set_ce_pin(0);
}

/****************************************************************************/

void rf24_toggleAllPipes(RF24* rf, unsigned char isEnabled)
{
    rf24_write_register(rf, EN_RXADDR, (isEnabled ? 0x3F : 0), 0);
}

/****************************************************************************/

unsigned char _data_rate_reg_value(RF24* rf, rf24_datarate_e speed)
{
    rf->txDelay = 280;

    if (speed == RF24_250KBPS) {

        rf->txDelay = 505;

        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        return _BV(RF_DR_LOW);
    }
    else if (speed == RF24_2MBPS) {

        rf->txDelay = 240;

        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        return _BV(RF_DR_HIGH);
    }
    // HIGH and LOW '00' is 1Mbs - our default
    return 0;
}

/****************************************************************************/

unsigned char _pa_level_reg_value(unsigned char level, unsigned char lnaEnable)
{
    return (((level > RF24_PA_MAX ? (RF24_PA_MAX) : level) << 1) + lnaEnable);
}

/****************************************************************************/

void rf24_setRadiation(RF24* rf, unsigned char level, rf24_datarate_e speed, unsigned char lnaEnable)
{
    unsigned char setup = _data_rate_reg_value(rf, speed);
    setup |= _pa_level_reg_value(level, lnaEnable);
    rf24_write_register(rf, RF_SETUP, setup, 0);
}
