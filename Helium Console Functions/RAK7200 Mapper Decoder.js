function Decoder(bytes, port) {

    let decoded = {};

    if ((port === 1) && (bytes.length === 23)) {
        decoded.latitude = Number(((bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]) * 1e-7).toFixed(7));
        decoded.longitude = Number(((bytes[4] << 24 | bytes[5] << 16 | bytes[6] << 8 | bytes[7]) * 1e-7).toFixed(7));
        decoded.altitude = Number(((bytes[8] << 24 | bytes[9] << 16 | bytes[10] << 8 | bytes[11]) * 1e-3).toFixed(3));
        decoded.accuracy = Number(((bytes[12] << 24 | bytes[13] << 16 | bytes[14] << 8 | bytes[15]) * 1e-3).toFixed(3));
        decoded.sats = Number(((bytes[16])).toFixed(0));
        decoded.hdop = Number(((bytes[17] << 8 | bytes[18]) * 1e-2).toFixed(0));
    }

    return decoded;
}
