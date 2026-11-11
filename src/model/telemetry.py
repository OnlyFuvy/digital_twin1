# название датчика: (частота (Гц), размер пакета в байтах)
SENSOR_BYTES = {
    "IMU": (200, 48),
    "GNSS": (10, 64),
    "BARO": (25, 8),
    "MAG": (50, 12),
    "LIDAR": (20, 16),
    "RPM": (50, 4),
    "POWER": (10, 12),
    "STATE": (50, 64),
    "ADC": (10, 4),
    "TEMP": (1, 4),
    "CURRENT": (50, 4),
    "VOLTAGE": (10, 4),
    "FUEL": (10, 4),
    "SOC": (1, 4),
    "AIRSPEED": (20, 4),
    "RADAR_ALT": (20, 4),
    "VIBRO": (100, 8),
    "GNSS_VEL": (10, 24),
}

# подсчет объема памяти
def compute_memory_budget(duration_s, overhead=1.3):
    streams = []
    total_bps = 0
    for name, (rate, size) in SENSOR_BYTES.items():
        bps = rate * size
        total_bps += bps
        streams.append({
            "name": name,
            "rate_hz": rate,
            "bytes_per_packet": size,
            "bytes_per_s": bps,
        })
    useful_bytes = total_bps * duration_s
    full_bytes = useful_bytes * overhead
    return {
        "streams": streams,
        "total_bytes_per_s": total_bps,
        "useful_mib": useful_bytes / (1024 ** 2),
        "full_mib": full_bytes / (1024 ** 2),
        "overhead_factor": overhead,
    }