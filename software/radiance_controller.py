import usb.core
import usb.util

dev = usb.core.find(idVendor=0xfffe, idProduct=0x13D5)

def send(data, *args, **kwargs):
    dev.write(0x01, data)
    return dev.read(0x82, 128)

print send("hello").tostring()
