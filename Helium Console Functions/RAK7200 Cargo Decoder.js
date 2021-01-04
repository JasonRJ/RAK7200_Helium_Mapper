function Decoder(bytes, port) {

    let decoded = {};

    if ((port === 1) && (bytes.length === 23)) {
        decoded.latitude = Number(((bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]) * 1e-7).toFixed(7));
        decoded.longitude = Number(((bytes[4] << 24 | bytes[5] << 16 | bytes[6] << 8 | bytes[7]) * 1e-7).toFixed(7));
        decoded.altitude = Number(((bytes[8] << 24 | bytes[9] << 16 | bytes[10] << 8 | bytes[11]) * 1e-3).toFixed(1));
        decoded.speed = Number(((bytes[19] << 24 | bytes[20] << 16 | bytes[21] << 8 | bytes[22]) * 0.002236936).toFixed(1));
    }

    return decoded;
}
