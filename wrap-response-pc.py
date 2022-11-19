#!/usr/bin/env python
# wrap-response-pc.py


import serial.tools.list_ports
import serial
import re, os, time, threading, copy


_cfg_debug_scan = True
_cfg_debug_framing = True

_cfg_tmp_file = os.path.abspath(os.path.dirname(__file__)) + os.path.sep + "tmp_serial_port_cache"


def extract_vid_pid(info_string):
    """
    Try different methods of extracting vendor and product IDs from a string.

    The output from serial.tools.list_ports.comports() varies so
    widely from machine to machine and OS to OS, the only option we
    have right now is to add to a list of patterns that we have seen so
    far and compare the output.

    info_string -- the string possibly containing vendor and product IDs.

    Returns a tuple of (vendor ID, product ID) if a device is found.
    If an ID isn't found, returns None.

    The code is adapted from pygatt/backends/bgapi/util.py .

    Example info_string:
        device  USB VID:PID=1915:520F SER=C2FA1DDFB5D5 LOCATION=1-3:x.0 COM9 USB Serial Device (COM9)
        device  USB VID:PID=1915:520F SER=C01234778899 LOCATION=1-8:x.0 COM10 USB Serial Device (COM10)
    """

    DEVICE_STRING_PATTERNS = [
        # '...VID:PID=XXXX:XXXX SER=XXXXXXXXXXXX...'
        re.compile('.*VID:PID=([0-9A-Fa-f]{0,4}):([0-9A-Fa-f]{0,4}).*SER=([0-9A-Fa-f]{0,12}).*'),

        # '...VID_XXXX...PID_XXXX SER=XXXXXXXXXXXX...'
        re.compile('.*VID_([0-9A-Fa-f]{0,4}).*PID_([0-9A-Fa-f]{0,4}).*SER=([0-9A-Fa-f]{0,12}).*')
    ]

    for p in DEVICE_STRING_PATTERNS:
        match = p.match(info_string)
        if match:
            return int(match.group(1), 16), int(match.group(2), 16), str(match.group(3)).lower()
    return None


class USBSerialDeviceInfo(object):
    pass


def find_working_port_scan(vid=0x1915, pid=0x520F):
    devices = serial.tools.list_ports.comports()

    found_port = None
    for device in devices:
            dev = USBSerialDeviceInfo()
            dev.port_name = device[0]
            dev.device_name = device[1]
            found_device = extract_vid_pid(device[2])

            if _cfg_debug_scan:
                print(" device: ", found_device, " port: ", dev.port_name, " name: ", dev.device_name)
            # device:  (6421, 21007, 'c01122334455')  port:  COM11  name:  USB Serial Device (COM11)
            # device:  (6790, 29987, '')  port:  COM15  name:  USB-SERIAL CH340 (COM15)
            if type(found_device) is tuple and len(found_device) == 3:
                if vid == found_device[0] and pid == found_device[1]:
                    dev.ser = found_device[2]
                    if found_port is None:
                        found_port = [dev]
                    else:
                        found_port.append(dev)

    working_port = None
    if found_port is not None:
        for port in found_port:
            if _cfg_debug_scan:
                print(" found_port: ", port.port_name, port.ser, port.device_name)
            if working_port is None:
                working_port = port
            else:
                print("Error: more than one ports")
    else:
        print(" found_port: none")

    return working_port


def find_working_port_cached():
    # try reuse
    port_opened_ok = False
    port_name = ""
    if os.path.isfile(_cfg_tmp_file):
        try:
            with open(_cfg_tmp_file, "r") as in_f:
                in_line = in_f.readline()
                in_f.close()
                in_line = in_line.strip()
                if len(in_line) > 0:
                    port_name = in_line
            if len(port_name) > 0:
                ser = serial.Serial(port_name, 115200, timeout=0.01)
                port_opened_ok = True
                ser.close()
        except:
            pass
    if not port_opened_ok:
        pn = find_working_port_scan()
        pn_tmp = pn.port_name[0:5]  # max 5 chars. so COM11: is COM11
        try:
            ser = serial.Serial(pn_tmp, 115200, timeout=0.01)
            port_opened_ok = True
            ser.close()
        except:
            pass
        if port_opened_ok:
            port_name = pn_tmp
            try:
                with open(_cfg_tmp_file, "w") as out_f:
                    out_f.write(port_name + "\n")
                    out_f.close()
            except:
                pass
    if port_opened_ok:
        return port_name
    else:
        return None


def find_working_port():
    #tm1 = time.time()
    #device:  (6790, 29987, '')  port:  COM15  name:  USB-SERIAL CH340 (COM15)
    retv = find_working_port_scan(vid=6790, pid=29987)     # .016 sec
    #retv = find_working_port_cached()   # .000 sec
    #tm2 = time.time()
    #print(" find working port consumed %.3f" % (tm2 - tm1))
    return retv


class SerReader(threading.Thread):
    def set_stop(self):
        self._stop_requested = True
    def wait_stopped(self):
        for i in range(50):  # 50 * 20ms = 1sec
            if self._stopped:
                break
            time.sleep(0.02) # 20ms

    def __init__(self, ser):
        super().__init__()
        self._ser = ser
        self._enable_read = None
        self._stopped = None
        self._stop_requested = None
        self._received_raw_bytes = bytearray(0)
        self._returned_raw_bytes = bytearray(0)

    def run(self):
        while True:
            if self._stop_requested == True:
                break
            rv = b''
            if self._enable_read == True:
                rv = self._ser.read(1024)
            if len(rv) > 0:
                self._received_raw_bytes.extend(bytearray(rv))
            elif self._ser.is_open:
                time.sleep(0.001)
            else:
                if _cfg_debug_framing:
                    print("Break due to ser closed")
                break
        self._stopped = True

    def get_data(self, timeout=1.0, up_to_bytes=512, up_to_chars=None): # up_to_char=b'\n'
        if len(self._received_raw_bytes) != len(self._returned_raw_bytes):
            print("Discard bytes: ", " _received_raw_bytes", len(self._received_raw_bytes),
                  " _returned_raw_bytes", len(self._returned_raw_bytes))
        self._received_raw_bytes = bytearray(0)
        tm0 = time.time()
        self._enable_read = True
        runcnt = 0 # index to next char to process
        while True:
            dlen = len(self._received_raw_bytes)
            ending_char_found = False
            if dlen > runcnt and up_to_chars is not None:
                ulen = len(up_to_chars)
                for i in range(runcnt, dlen-ulen+1):
                    runcnt = i+1
                    thechar = self._received_raw_bytes[i:i+ulen]
                    if thechar == up_to_chars:
                        ending_char_found = True
                        break
            if ending_char_found:
                if _cfg_debug_framing:
                    print("  break due to ending char found  ")
                break
            if dlen >= up_to_bytes:
                if _cfg_debug_framing:
                    print("  break due to number of bytes  ")
                break
            if time.time() > tm0 + timeout:
                if _cfg_debug_framing:
                    print("  break due to timeout  ")
                break
            time.sleep(0.010)
        self._enable_read = False
        self._returned_raw_bytes = copy.deepcopy(self._received_raw_bytes)
        return bytes(self._returned_raw_bytes) # from bytearray


class SerialConnection(object):
    def init_ok(self):
        if self._init_ok == True:
            return True
        return False

    def __init__(self):
        self._working_port = find_working_port()
        self._init_ok = None
        if self._working_port is not None:
            if type(self._working_port) is str:
                self._port_file_name = self._working_port
            else: # port-info object
                self._port_file_name = ('%s:' % self._working_port.port_name) [0:5] # max 5 chars. so COM11: is COM11
            try:
                self._ser = None
                ser = serial.Serial(self._port_file_name, 115200, timeout=0.01, writeTimeout=0)
                self._ser = ser
            except:
                pass
            if self._ser is None:
                raise RuntimeError("Error: could not open serial port %s" % self._port_file_name)
            if type(self._working_port) is str:
                print(" working on port name: ", self._ser.name, "  ser: ", "reused")
            else:
                print(" working on port name: ", self._ser.name, "  ser: ", self._working_port.ser)
            self._rxthread = SerReader(self._ser)
            self._rxthread.start()
            self._init_ok = True
        ##self._i2c_address_current = None
        ##self._twi_bus_current = 0 # init 0. change to 1 or 2 after querying vcp
        ##self._twi_bus_select = 1  # default 1. 1 or 2.

    def send_data(self, data_bytes):
        if self._init_ok != True:
            return None
        # prepend one and append two delineators:
        ##self._ser.write(data_bytes + sliplib.END)
        self._ser.write(data_bytes)
        return True

    def recv_data(self, timeout=0.5):
        if self._init_ok != True:
            return None
        ##pkt_end = sliplib.END + sliplib.END
        pkt_end = b"\r\n"
        ret_data = self._rxthread.get_data(timeout=timeout, up_to_chars=pkt_end)
        return ret_data

    def finish(self):
        if self._init_ok != True:
            return None

        self._rxthread.set_stop()
        tm3 = time.time()
        self._rxthread.wait_stopped()
        tm4 = time.time()

        self._ser.close()
        if tm4 - tm3 >= 0.1:
            print("  wait_stop used %.2f" % (tm4-tm3))

        self._init_ok = False # mark finished.


if __name__ == '__main__':

    try:
        ser = SerialConnection()
    except Exception as e:
        ser = None
        print(" exception: ", repr(e))

    if ser is not None and ser.init_ok():
        stats_tx_data_len = 0 # less than 244*2=488
        stats_rx_data_len = 0

        rx_data_bytes = b''
        tm0=time.time()
        ##ser.send_data(data)
        tm1=time.time()
        while True:
            rv = ser.recv_data(timeout=5)
            tm2=time.time()
            tdata = None
            if len(rv) > 0:
                print("  %.2f rv: " % ( tm2 - tm0 ), len(rv), repr(rv))
                # nrf boot:  b'\r\nUART started.\r\n'
                # normal:    b'/GET info\r\n'
                rx_data_bytes += rv
                stats_rx_data_len += len(rv)
                if rx_data_bytes[-1:] == b'\r' or rx_data_bytes[-1:] == b'\n':
                    # including rx_data_bytes[-2:] == b'\r\n'
                    rdata = rx_data_bytes.strip()
                    rx_data_bytes = b''
                    try:
                        tmp_data = rdata.decode()
                        rdata = tmp_data
                    except:
                        pass
                    if rdata.startswith('/GET info'):
                        tdata = [
                            b'{\n "request_type" : "get_info",\n "status":"ack"\n};\n',
                            b'{\n "request_type": "get_info",\n "status": "success"\n};\n',
                        ]
                    elif rdata.startswith('/GET device_list'):
                        tdata = [
                            b'{\n "request_type" : "get_device_list",\n "status":"ack"\n};\n',
                            b'{\n "status": "success",\n "request_type": "get_device_list",\n "result": [\n  { "name": "223344556677", "private": true, "signal-level": "-48" },\n  { "name": "22334466", "private": true, "signal-level": "-52" } \n ]\n};\n',
                        ]
            if tdata is not None:
                for x in tdata:
                    dlen_tx = len(x)
                    dptr = 0
                    if dlen_tx <= 0:
                        continue
                    while dlen_tx > 0:
                        if dlen_tx > 20:
                            y = x[dptr:dptr+20]
                            dptr += 20
                            dlen_tx -= 20
                        else:
                            y = x[dptr:]
                            dlen_tx = 0
                        tm_tx1 = time.time()
                        for idx,_ in enumerate(y):
                            yy = y[idx:idx+1]
                            ser.send_data(yy)
                            #time.sleep(0.001) # 1 byte / 2ms or 40ms/20byte
                        tm_tx2 = time.time()
                        time.sleep(0.060)  # 20 bytes / 60ms
                        tm_tx3 = time.time()
                        print("    sent seg %d %.3f %.3fsec" % (len(y), tm_tx2-tm_tx1, tm_tx3-tm_tx2))
                    ser.send_data(b'\0')
                    stats_tx_data_len += len(x)
                    print("  sent len: ", len(x))
                    print("  sent data: ", repr(x))
                    time.sleep(1)

            #if tm2 - tm1 > 1200: # 12: test duration 12 seconds
            #    break
            time.sleep(0.010)

        ser.finish()

        print("")
        print(" %.2f for initial write. %.2f to finish.  rx-rate %.3f  tx-rate %.3f" % (
                tm1-tm0, tm2-tm0,
                stats_rx_data_len/(tm2-tm0), stats_tx_data_len/(tm2-tm0) ))
        print(" len data received ", stats_rx_data_len)
        print(" len data written ", stats_tx_data_len)


