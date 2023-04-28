import numpy as np
import serial
import datetime as dt
import os
import struct
import datetime as dt


class ReadLine:
    """A class to speed up reading lines with pyserial. See: https://github.com/pyserial/pyserial/issues/216
    
    Inputs:
        s: a pyserial object
        
    Example:
        ser = serial.Serial(...)
        reader = ReadLine(ser)
        while True:
            print(reader.readline())
    """
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if len(data)==0:
                return r
            elif i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
                
def second_line_check(max_attempts, reader, header, n_good_thresh=10):
    attempts = 0
    n_good = 0
    status = 0
    second_line = 1
    while attempts < max_attempts:
        line = reader.readline().decode('utf-8').strip('\r\n')
        data_elements = line.split(',')
        header_elements = header.split(',')
        if len(data_elements) == len(header_elements):
            n_good += 1
        else:
            n_good = 0
            attempts += 1
        
        if n_good > n_good_thresh:
            second_line = 0
            status = 1
            break
    
    return status, second_line


def first_line_check(header_max_attempts, reader, file=None):
    _ = reader.readline().decode('utf-8').strip('\r\n')  # flush old line of data
    header = None
    header_attempts = 0
    status = 0
    read_lines = []
    while header_attempts < header_max_attempts:
        header = reader.readline().decode('utf-8').strip('\r\n')
        header_attempts += 1
        read_lines.append(header)
        if header[0:4] == 'time':
            status = 1
            first_line=0
            second_line=1
            break
    if status == 1 and file is not None:
        file.write(header)
        file.write('\n')

    return status, first_line, second_line, header, header_attempts, read_lines


def shuffle_by_date(lst, date=None, date_format=None):
    if date is not None:
        today = dt.datetime.strptime(date, date_format)
    else:
        today = dt.datetime.now().date()
    date_hash = int(dt.datetime(today.year, today.month, today.day).timestamp())
    np.random.seed(date_hash)
    np.random.shuffle(lst)
    return lst




def packIntAsLong(value):
    """Packs a python 4 byte integer to an arduino long
    Parameters
    ----------
    value : int
        A 4 byte integer
    Returns
    -------
    packed : bytes
        A 4 byte long
    """
    return struct.pack("i", value)