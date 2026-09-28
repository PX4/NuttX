==========================
Remote Proc Tunnel Drivers
==========================

RPTUN driver is used for multi-cores' communication.

Supported RPTUN drivers:

- RPMSG File System
- RPMSG domain (remote) sockets
- RPMSG UART Driver
- RPMSG net Driver
- RPMSG Usersock
- RPMSG Sensor Driver
- RPMSG RTC Driver
- RPMSG MTD
- RPMSG Device
- RPMSG Block Driver
- RPMSG IO expander
- RPMSG uinput
- RPMSG clk Driver
- RPMSG syslog
- RPMSG regulator

Stopping a remote
=================

``RPTUNIOC_STOP`` tears down every rpmsg endpoint, announcing each one to the
remote through the name service, then stops the remote. An announcement waits
for a TX buffer, so a remote that no longer returns buffers holds the caller
for the rpmsg send timeout per endpoint. Pass ``RPTUN_STOP_NO_NS`` as the
ioctl argument when the remote will not outlive the stop, for example a core
the backend holds in reset: the endpoints are then torn down without
name-service messages, as ``RPTUNIOC_START`` already does when it restarts a
running remote.
