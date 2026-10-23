#!/usr/bin/env python3
"""bt_spp_bridge - the SL-0105 `fcb_link: {kind: bt_spp}` endpoint.

A Bluetooth SPP (RFCOMM) server bridged to the unit's fcb proxy: a
phone (Android running Andruav, or a Pi) pairs with the host and dials
the Serial Port Profile on this channel like it would a drone's FC
port - every byte lands on tcp:127.0.0.1:<proxy listen port> and back,
so the proxy's cut/stall/corrupt/rate rules apply to the link as usual.

    bt_spp_bridge.py --channel 1 --target <proxy port> \
        [--ctrl <proxy ctrl port>] [--baud <baud>] --statedir <dir>

One RFCOMM client at a time (a real serial port has one master); when
the client drops, the next accept waits again. The process logs to
stdout - the launcher captures it as logs/bt_spp.log.

SPP discovery: `sdptool add --channel=<n> SP` publishes the Serial Port
Profile record so a phone's SPP-UUID lookup finds the channel. On
BlueZ >= 5 without --compat there is no local sdpd socket - the
registration fails and ./sim doctor prints the fix (the bridge itself
still serves the channel; only discovery is missing).

--statedir is unused beyond carrying the sim's var/run path on this
process's command line, so pid_is_ours/stray-sweep can see it (the
same convention as fcb_proxy's --statedir).
"""
import argparse
import atexit
import os
import socket
import subprocess
import sys
import threading
import time

RFCOMM_ANY = "00:00:00:00:00:00"   # bind on every local adapter
# python builds without bluetooth lack the constant - its value is
# stable on Linux so the module still imports for unit tests
AF_BLUETOOTH = getattr(socket, "AF_BLUETOOTH", 31)
BTPROTO_RFCOMM = getattr(socket, "BTPROTO_RFCOMM", 3)


def log(msg):
    print(f"[bt_spp] {msg}", flush=True)


def sdp_register(channel):
    """Publish the SPP record for `channel` via sdptool. Returns the
    failure string or None."""
    try:
        r = subprocess.run(["sdptool", "add", f"--channel={channel}",
                            "SP"], capture_output=True, text=True,
                           timeout=10)
    except (OSError, subprocess.SubprocessError) as e:
        return f"sdptool: {e}"
    if r.returncode == 0:
        return None
    return (r.stderr or r.stdout).strip() or f"sdptool exit {r.returncode}"


def sdp_unregister(channel):
    try:
        subprocess.run(["sdptool", "del", f"--channel={channel}"],
                       capture_output=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        pass


def arm_rate(ctrl_port, baud):
    """Pace the proxy link at `baud` (a real SPP hop is not free
    either). Re-sent on every client accept so a proxy restart is
    re-armed."""
    try:
        s = socket.create_connection(("127.0.0.1", ctrl_port), 3)
        s.sendall(f"rate {baud}\n".encode())
        s.close()
    except OSError as e:
        log(f"ctrl: could not arm rate {baud}: {e}")


def _pipe(src, dst):
    """Copy src -> dst until EOF/error; half-close dst so the far end
    sees a clean hang-up, not a vanished socket."""
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def pump(client, tcp):
    """Bridge client <-> tcp until either side ends. Blocks; both
    sockets are closed by the caller after return."""
    t = threading.Thread(target=_pipe, args=(client, tcp), daemon=True)
    t.start()
    _pipe(tcp, client)
    t.join(5)


def handle_client(client, target_port, ctrl_port, baud):
    """Dial the proxy and run the byte bridge for one client."""
    if baud and ctrl_port:
        arm_rate(ctrl_port, baud)
    try:
        tcp = socket.create_connection(("127.0.0.1", target_port), 5)
    except OSError as e:
        log(f"proxy 127.0.0.1:{target_port} unreachable: {e} - "
            "dropping client")
        return
    try:
        pump(client, tcp)
    finally:
        tcp.close()


def serve(channel, target_port, ctrl_port, baud):
    """The RFCOMM server: accept one client, bridge it, accept again."""
    srv = socket.socket(AF_BLUETOOTH, socket.SOCK_STREAM,
                        BTPROTO_RFCOMM)
    srv.bind((RFCOMM_ANY, channel))
    srv.listen(1)
    log(f"RFCOMM channel {channel} listening -> "
        f"tcp:127.0.0.1:{target_port}")
    while True:
        try:
            client, addr = srv.accept()
        except OSError as e:
            log(f"accept failed: {e} - exiting for the wrapper to retry")
            return 1
        log(f"client {addr} connected")
        try:
            handle_client(client, target_port, ctrl_port, baud)
        finally:
            client.close()
        log("client gone - listening again")


def _reexec_bluetooth_python():
    """When this interpreter lacks AF_BLUETOOTH (e.g. a conda build),
    re-exec under a system python that has it. The pid is unchanged, so
    the pidfile and group-kill semantics still hold."""
    if hasattr(socket, "AF_BLUETOOTH"):
        return
    probe = ("import socket,sys;"
             "sys.exit(0 if hasattr(socket,'AF_BLUETOOTH') else 1)")
    me = os.path.realpath(sys.executable)
    for cand in ("/usr/bin/python3.14", "/usr/bin/python3.13",
                 "/usr/bin/python3.12", "/usr/bin/python3.11",
                 "/usr/bin/python3"):
        if not os.path.isfile(cand) or os.path.realpath(cand) == me:
            continue
        if subprocess.call([cand, "-c", probe],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0:
            log(f"{sys.executable} lacks AF_BLUETOOTH - re-exec {cand}")
            os.execv(cand, [cand, os.path.abspath(sys.argv[0])]
                    + sys.argv[1:])
    sys.exit("no python with AF_BLUETOOTH found "
             "(candidates: /usr/bin/python3*)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--channel", type=int, required=True,
                    help="RFCOMM channel to serve (1-30)")
    ap.add_argument("--target", type=int, required=True,
                    help="the unit fcb proxy's listen port")
    ap.add_argument("--ctrl", type=int, default=0,
                    help="the proxy's ctrl port (for --baud rate arming)")
    ap.add_argument("--baud", type=int, default=0,
                    help="optional link pacing armed on the proxy")
    ap.add_argument("--statedir", default="",
                    help="proc state dir - marks the cmdline as ours")
    a = ap.parse_args()

    if not 1 <= a.channel <= 30:
        sys.exit(f"channel {a.channel} not in 1-30")
    _reexec_bluetooth_python()

    err = sdp_register(a.channel)
    if err:
        log(f"SPP record not published ({err}) - the channel still "
            "serves, but a phone cannot discover it; see ./sim doctor "
            "(BlueZ needs --compat for sdptool)")
    else:
        log(f"SPP record published on channel {a.channel}")
        atexit.register(sdp_unregister, a.channel)
    sys.exit(serve(a.channel, a.target, a.ctrl, a.baud))


if __name__ == "__main__":
    main()
