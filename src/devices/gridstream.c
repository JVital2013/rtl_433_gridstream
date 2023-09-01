/** @file
    Decoder for Gridstream RF devices produced by Landis & Gyr.

    Copyright (C) 2016 Benjamin Larsson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

/**
Landis & Gyr Gridstream Power Meters.

- Center Frequency: 915 Mhz
- Modulation: FSK-PCM
- Bitrates: 9600, 19200, 38400
- Preamble: 0xAAAA
- Syncword v4: 0b0000000001 0b0111111111
- Syncword v5: 0b0000000001 0b11111111111

Datastream is variable length and bitrate depending on type fields
Preamble
Bytes after preamble are encoded with 8N1
Data layouts:
    Subtype 55:
        AAAAAA SSSS TT YY LLLL KK BBBBBBBBBB WWWWWWWWWW II MMMMMMMM KKKK EEEEEEEE KKKK KKKKKK CCCC KKKK XXXX KK
    Subtype D2:
        AAAAAA SSSS TT YY LL K----------K XXXX
    Subtype D5:
        AAAAAA SSSS TT YY LLLL KK DDDDDDDD EEEEEEEE II K----------K CCCC KKKK XXXX
- A - Preamble
- S - Syncword
- T - Type
- Y - Subtype
- L - Length
- B - Broadcast
- D - Dest Address
- E - Source Address
- M - Uptime (time since last outage in seconds)
- I - Counter
- C - Clock
- K - Unknown
- X - CRC (poly 0x1021, init set by provider)

*/

#include "decoder.h"
#include <stdbool.h>

static uint16_t known_crc_init[] = {
0xe623, 0x5fd6, 0xD553, 0x45F8, 0x62C1, 0x23D1, 0x2C22, 0x142A, };

static int gridstream_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t const preamble[] = { 0xAA, 0xAA, 0x00, };
    uint8_t b[BITBUF_COLS];
    uint16_t stream_len;
    uint16_t crc;
    uint16_t crc_count = 0;
    bool crc_ok = false;
    uint64_t wanaddress, destaddress;
    uint32_t srcaddress;
    char srcaddress_str[9+1],wanaddress_str[13+1],destaddress_str[13+1];
    uint32_t uptime;
    uint32_t clock;
    unsigned offset = bitbuffer_search(bitbuffer, 0, 0, preamble, 24) + 16;
    /* TODO: Special handling for V4 and V5 syncwords as V5 is not compatible with extract_bytes_uart() */
    int decoded_len = extract_bytes_uart(bitbuffer->bb[0],offset,bitbuffer->bits_per_row[0]-offset,b);

    if (decoded_len >= 5) {
        switch(b[2]) {
            case 0x2A:
                switch (b[3]) {
                    case 0x55:
                        stream_len = (b[4] << 8) | b[5];
                        if ((sizeof(b) - 6) < stream_len) {
                            return DECODE_ABORT_LENGTH;
                        }
                        crc = ( b[4 + stream_len] << 8 ) | b[5 + stream_len];
                        do {
                            if (crc16(&b[6],stream_len-2,0x1021,known_crc_init[crc_count])==crc) {
                                crc_ok = true;
                            } else
                            {
                                crc_count++;
                            }
                        }
                        while ( crc_count < (sizeof(known_crc_init)/2) && crc_ok == false );
                        if (!crc_ok) {
                            
                            return DECODE_FAIL_MIC;
                        }
                        destaddress = ((uint64_t)b[7] << 40) | ((uint64_t)b[8] << 32) | (b[9] << 24) | (b[10] << 16) | (b[11] << 8) | b[12];
                        wanaddress = ((uint64_t)b[13] << 40) | ((uint64_t)b[14] << 32) | (b[15] << 24) | (b[16] << 16) | (b[17] << 8) | b[18];
                        uptime = ((uint32_t)b[20] << 24) | (b[21] << 16) | (b[22] << 8) | b[23];
                        srcaddress = ((uint32_t)b[26] << 24) | (b[27] << 16) | (b[28] << 8) | b[29];
                        sprintf(destaddress_str, "%12lx", destaddress);
                        sprintf(wanaddress_str, "%12lx", wanaddress);
                        sprintf(srcaddress_str,"%08x", srcaddress);
                        /* clang-format off */
                        data = data_make(
                                "model",      "", DATA_STRING, "LandisGyr GridStream",
                                "id",         "", DATA_STRING,    srcaddress_str,
                                "wanaddress", "", DATA_STRING,    wanaddress_str,
                                "destaddress", "", DATA_STRING,   destaddress_str,
                                "uptime",     "", DATA_INT,    uptime,
                                "mic",        "", DATA_STRING, "CRC", // CRC, CHECKSUM, or PARITY
                                NULL);
                        /* clang-format on */

                        break;
                    case 0xD2:
                        stream_len = b[4];
                        if ((sizeof(b) - 6) < stream_len) {
                            return DECODE_ABORT_LENGTH;
                        }
                        crc = ( b[4 + stream_len] << 8 ) | b[5 + stream_len];
                        do {
                            if (crc16(&b[5],stream_len-2,0x1021,known_crc_init[crc_count])==crc) {
                                crc_ok = true;
                            } else
                            {
                                crc_count++;
                            }
                        }
                        while ( crc_count < (sizeof(known_crc_init)/2) && crc_ok == false );
                        if (!crc_ok) {
                            return DECODE_FAIL_MIC;
                        }

                        /* clang-format off */
                        data = data_make(
                                "model",      "", DATA_STRING, "LandisGyr GridStream",
                                "id",         "", DATA_INT,    0,
                                "mic",        "", DATA_STRING, "CRC", // CRC, CHECKSUM, or PARITY
                                NULL);
                        /* clang-format on */

                        break;
                    case 0xD5:
                        stream_len = (b[4] << 8) | b[5];
                        if ((sizeof(b) - 6) < stream_len) {
                            return DECODE_ABORT_LENGTH;
                        }
                        crc = ( b[4 + stream_len] << 8 ) | b[5 + stream_len];
                        do {
                            if (crc16(&b[6],stream_len-2,0x1021,known_crc_init[crc_count])==crc) {
                                crc_ok = true;
                            } else
                            {
                                crc_count++;
                            }
                        }
                        while ( crc_count < (sizeof(known_crc_init)/2) && crc_ok == false );
                        if (!crc_ok) {
                            return DECODE_FAIL_MIC;
                        }
                        destaddress = ((uint32_t)b[7] << 24) | (b[8] << 16) | (b[9] << 8) | b[10];
                        srcaddress = ((uint32_t)b[11] << 24) | (b[12] << 16) | (b[13] << 8) | b[14];
                        sprintf(destaddress_str,"%08lx", destaddress);
                        sprintf(srcaddress_str,"%08x", srcaddress);
                        if (stream_len == 0x47) {
                            clock = ((uint32_t)b[16] << 24) | (b[17] << 16) | (b[18] << 8) | b[19];
                            uptime = ((uint32_t)b[24] << 24) | (b[25] << 16) | (b[26] << 8) | b[27];
                            wanaddress = ((uint64_t)b[32] << 40) | ((uint64_t)b[33] << 32) | ((uint32_t)b[34] << 24) | (b[35] << 16) | (b[36] << 8) | b[37];
                            sprintf(wanaddress_str, "%12lx", wanaddress);

                            /* clang-format off */
                            data = data_make(
                                    "model",        "", DATA_STRING, "LandisGyr GridStream",
                                    "id",           "", DATA_STRING, srcaddress_str,
                                    "destaddress",  "", DATA_STRING, destaddress_str,
                                    "timestamp",    "", DATA_STRING, clock,
                                    "uptime",       "", DATA_INT,    uptime,
                                    "wanaddress",   "", DATA_STRING, wanaddress_str,
                                    "mic",          "", DATA_STRING, "CRC", // CRC, CHECKSUM, or PARITY
                                    NULL);
                            /* clang-format on */
                            
                        } else
                        {

                            /* clang-format off */
                            data = data_make(
                                    "model",        "", DATA_STRING, "LandisGyr GridStream",
                                    "id",           "", DATA_STRING, srcaddress_str,
                                    "destaddress",  "", DATA_STRING, destaddress_str,
                                    "mic",          "", DATA_STRING, "CRC", // CRC, CHECKSUM, or PARITY
                                    NULL);
                            /* clang-format on */
                        }
                        break;
                }
                break;
            default:
                return DECODE_ABORT_LENGTH;
                break;
        }

        decoder_output_data(decoder, data);

        // Return 1 if message successfully decoded
        return 1;
    }
    else
    {
        return DECODE_FAIL_SANITY;
    }
}

static char const *const output_fields[] = {
        "model",
        "id",
        "wanaddress",
        "destaddress",
        "uptime",
        "srclocation",
        "destlocation",
        "time",
        "mic",
        NULL,
};

r_device const gridstream = {
        .name        = "Gridstream decoder",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 104,
        .long_width  = 104,
        .reset_limit = 20000,
        .decode_fn   = &gridstream_decode,
        .disabled    = 0,
        .fields      = output_fields,
};