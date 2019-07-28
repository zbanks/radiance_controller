import usb.core
import struct
import time

dev = usb.core.find(idVendor=0xfffe, idProduct=0x13D5)
if dev is None:
    raise Exception()

def pingpong(data, *args, **kwargs):
    dev.write(0x01, data)
    return dev.read(0x82, 128).tostring()

def read():
    dev.write(0x01, "\x01")
    #raw_count = dev.read(0x82, 2).tostring()
    #count = struct.unpack("<H", raw_count)[0]
    
    keys = ["id", "intensity", "tempo", "pad"]
    output = []
    raw = dev.read(0x82, 512).tostring()
    while raw:
        s, raw = raw[0:8], raw[8:]
        if len(s) < 8:
            break
        vals = struct.unpack("<IHBB", s)
        output.append({ k: v for k, v in zip(keys, vals) })

    return output[::-1]

NAMES = {
    "0x1ed8a9da": "A",
    "0x1edda9da": "B",
    "0x1edaa9da": "C",
    "0x1ee7a9da": "D",
}
def pretty(raw_data):
    intensity = raw_data["intensity"] / float(2 ** 16 - 1) * 100
    name = hex(raw_data["id"])
    name = NAMES.get(name, name)
    tempo = raw_data["tempo"]
    return "{:<16} {:8} {:>8.1f}%".format(name, tempo, intensity)

while 1:
    print [pretty(r) for r in read()]
    time.sleep(0.01)
