// RAK7200-0000 (AC1F09FFFE000000)

/*
 * The Device EUI (DEVEUI) must be in least-significant-byte order.
 * When copying the Device EUI from Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM DEVEUI[8]= {0x00, 0x00, 0x00, 0xFE, 0xFF, 0x09, 0x1F, 0xAC};
//void os_getDevEui(u1_t *buf) {memcpy_P(buf, DEVEUI, 8);}

/*
 * The App EUI (APPEUI) must be in least-significant-byte order.
 * When copying the App EUI from Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM APPEUI[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getArtEui(u1_t *buf) {memcpy_P(buf, APPEUI, 8);}

/*
 * The App Key (APPKEY) must be in most-significant-byte order.
 * When copying the App Key from Console be sure msb: is the byte order selected.
 */
//static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getDevKey(u1_t *buf) {memcpy_P(buf, APPKEY, 16);}
