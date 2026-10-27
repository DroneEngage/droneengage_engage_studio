#!/usr/bin/python3
"""Bluetooth SPP (RFCOMM serial) server on this host that relays to a SITL
(or any MAVLink) TCP port, so a unit can test its Bluetooth FC link against
SITL as if it were a Bluetooth telemetry radio.

Clients: the Andruav phone (FCB screen -> Bluetooth -> this host) or an RPi
(rfcomm connect, then de_mavlink on /dev/rfcomm0). See README.md.

usage: bt_spp_relay.py [tcp_port] [tcp_host] [--channel N]
       defaults: 6000 127.0.0.1, RFCOMM channel 22"""
import os, socket, sys, threading
import dbus, dbus.service, dbus.mainloop.glib
from gi.repository import GLib

args = [a for a in sys.argv[1:]]
CHANNEL = 22
if "--channel" in args:
    i = args.index("--channel"); CHANNEL = int(args[i + 1]); del args[i:i + 2]
TCP = (args[1] if len(args) > 1 else "127.0.0.1", int(args[0]) if args else 6000)
SPP = "00001101-0000-1000-8000-00805f9b34fb"

def pump(a, b, tag):
    n = 0
    try:
        while True:
            d = a.recv(4096)
            if not d: break
            b.sendall(d); n += len(d)
    except OSError:
        pass
    print(f"{tag} closed after {n} bytes", flush=True)
    for s in (a, b):
        try: s.shutdown(socket.SHUT_RDWR)
        except OSError: pass

class Profile(dbus.service.Object):
    @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}")
    def NewConnection(self, dev, fd, props):
        fd = fd.take()
        bt = socket.socket(fileno=fd)
        bt.setblocking(True)
        print("BT connected:", dev, flush=True)
        try:
            tcp = socket.create_connection(TCP, timeout=5); tcp.settimeout(None)
        except OSError as e:
            print("SITL connect failed:", e, flush=True); bt.close(); return
        print("relaying to SITL", TCP, flush=True)
        threading.Thread(target=pump, args=(bt, tcp, "phone->sitl"), daemon=True).start()
        threading.Thread(target=pump, args=(tcp, bt, "sitl->phone"), daemon=True).start()
    @dbus.service.method("org.bluez.Profile1", in_signature="o")
    def RequestDisconnection(self, dev): print("disconnect", dev, flush=True)
    @dbus.service.method("org.bluez.Profile1")
    def Release(self): pass

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()
Profile(bus, "/de/spp_sitl")
dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.ProfileManager1").RegisterProfile(
    "/de/spp_sitl", SPP, {"Name": "DE SITL SPP", "Role": "server", "Channel": dbus.UInt16(CHANNEL),
                          "RequireAuthentication": False, "RequireAuthorization": False, "AutoConnect": True})
print(f"SPP server up on RFCOMM channel {CHANNEL}, relaying to", TCP, flush=True)
GLib.MainLoop().run()
