
import argparse
import csv
import struct
import datetime

GPS_COLUMNS = [
    "hour",
    "minute",
    "seconds",
    "latitude",
    "longitude",
    "altitude",
    "speed",
    "fixquality",
    "satellites"
]

IMU_COLUMNS = [
    "timestamp",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z"
]

SLOW_COLUMNS = [
    "timestamp",
    "mag_x",
    "mag_y",
    "mag_z",
    "ms5611_temperature",
    "ms5611_pressure",
    "analog_0",
    "analog_1",
    "analog_2",
    "analog_3",
    "analog_4",
    "analog_5",
    "analog_6",
    "analog_7",
    "analog_8",
    "analog_9"
]

def read_next_message(file, gps, imu, slow):
    try:
        message_type = file.read(1)

        if message_type == b'\x10':
            return read_gps_message(file, gps)
        elif message_type == b'\x20':
            return read_imu_message(file, imu)
        elif message_type == b'\x30':
            return read_slow_message(file, slow)
    except struct.error:
        pass
    except Exception as e:
        print(e)
    return False

def read_gps_message(file, gps):
    values = struct.unpack("<BBBiiffBB", file.read(21))
    gps.writerow({
        "hour": values[0],
        "minute": values[1],
        "seconds": values[2],
        "latitude": values[3],
        "longitude": values[4],
        "altitude": values[5],
        "speed": values[6],
        "fixquality": values[7],
        "satellites": values[8]
    })
    return True

def read_imu_message(file, imu):
    timestamp = read_timestamp(file)
    values = struct.unpack("<ffffff", file.read(24))
    imu.writerow({
        "timestamp": timestamp,
        "accel_x": values[0],
        "accel_y": values[1],
        "accel_z": values[2],
        "gyro_x": values[3],
        "gyro_y": values[4],
        "gyro_z": values[5]
    })
    return True

def read_slow_message(file, slow):
    timestamp = read_timestamp(file)
    values = struct.unpack("<fffffffffffffff", file.read(60))
    slow.writerow({
        "timestamp": timestamp,
        "mag_x": values[0],
        "mag_y": values[1],
        "mag_z": values[2],
        "ms5611_temperature": values[3],
        "ms5611_pressure": values[4],
        "analog_0": values[5],
        "analog_1": values[6],
        "analog_2": values[7],
        "analog_3": values[8],
        "analog_4": values[9],
        "analog_5": values[10],
        "analog_6": values[11],
        "analog_7": values[12],
        "analog_8": values[13],
        "analog_9": values[14]
    })
    return True

def read_timestamp(file):
    values = struct.unpack("<BBBBBBH", file.read(8))
    return "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}".format(*values)

def main(fc_log: str):
    with open(fc_log, "rb") as fc_log_file:
        with open(fc_log + '.gps.csv', 'w') as gps:
            gps_writer = csv.DictWriter(gps, fieldnames=GPS_COLUMNS)
            gps_writer.writeheader()
            with open(fc_log + '.imu.csv', 'w') as imu:
                imu_writer = csv.DictWriter(imu, fieldnames=IMU_COLUMNS)
                imu_writer.writeheader()
                with open(fc_log + '.slow.csv', 'w') as slow:
                    slow_writer = csv.DictWriter(slow, fieldnames=SLOW_COLUMNS)
                    slow_writer.writeheader()
                    while read_next_message(fc_log_file, gps_writer, imu_writer, slow_writer):
                        pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('fc_log')

    args = parser.parse_args()
    main(args.fc_log)
