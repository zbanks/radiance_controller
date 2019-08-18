import usb.core
import struct
import time
import sys

dev = usb.core.find(idVendor=0xfffe, idProduct=0x13D5)
if dev is None:
    raise Exception("Cannot find dev")

def pingpong(data, *args, **kwargs):
    dev.write(0x01, data)
    return dev.read(0x82, 128).tostring()

def set_led(id_addr, (r, g, b)):
    data = struct.pack("<IIBBBB", id_addr, 0, 0, r, g, b)
    dev.write(0x01, data)

def read():
    #set_led(0x1ee7a9da, (50, 1, 1))
    #dev.write(0x01, "\x01")
    #raw_count = dev.read(0x82, 2).tostring()
    #count = struct.unpack("<H", raw_count)[0]
    
    keys = ["id", "id_parent", "input_parent", "tempo", "intensity"]
    output = []
    raw = dev.read(0x82, 12).tostring()
    vals = struct.unpack("<IIBBH", raw)
    return { k: v for k, v in zip(keys, vals) }

    #while raw:
    #    s, raw = raw[0:12], raw[12:]
    #    if len(s) < 12:
    #        break
    #    vals = struct.unpack("<IIBBH", s)

    #return output[::-1]

NAMES = {
    0xfbd7c5d9: "Screen",
    2039357172: "A",
    0xf6c6f3f5: "B",
    0: "",
}
def name(_id):
    return NAMES.get(_id, hex(_id))

def pretty(raw_data):
    intensity = raw_data["intensity"] / float(2 ** 16 - 1) * 100
    name = hex(raw_data["id"])
    name = NAMES.get(name, name)
    tempo = raw_data["tempo"]
    return "{:<16} {:8} {:>8.1f}%".format(name, tempo, intensity)

nodes = {}
c = 50
t = 0
i = 0
while 1:
    #print [pretty(r) for r in read()]
    #time.sleep(0.1)
    i += 1
    if nodes:
        nid, n = nodes.items()[i % len(nodes)]
        t = max(0, min(255, (n["tempo"] - 127) * 8 + 127))
        set_led(nid, (n["intensity"] / 256, 0, t))
    else:
        set_led(0, (0, 0, 0))

    data = read()
    if data["id"] == 0: continue
    data["name"] = name(data["id"])
    data["name_parent"] = name(data.pop("id_parent"))
    #if i % 10 == 0: print data
    nodes[data["id"]] = {
        "name": data["name"],
        "tempo": data["tempo"],
        "intensity": data["intensity"],
        "parent": data["name_parent"],
    }
    sys.stdout.write(repr(nodes.values()) + "\r")
    sys.stdout.flush()
    c = data["intensity"] / 256
    time.sleep(0.01)
