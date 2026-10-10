# ExpressLRS Hardware & Firmware Setup for MAVLink

To use this library, your ExpressLRS Transmitter (TX) module must be properly configured to output Native MAVLink over its USB port.

## 1. Hardware Pinout (ESP32)

By default, the ExpressLRS firmware routes standard CRSF/MAVLink telemetry to the hardware UART pins on the back of the module bay. To route it through the USB-C port on the ESP32 module instead, you must ensure the ESP32's USB-to-Serial bridge is connected to the firmware's UART.

1. Connect to your TX module's Web UI (via WiFi).
2. Go to the **Hardware** tab.
3. Locate the **Serial RX** and **Serial TX** pin assignments.
4. Set them to:
   * **TX Pin:** `1`
   * **RX Pin:** `3`
5. Click **Save and Reboot**.

*(Note: Pins 1 and 3 are the standard ESP32 hardware UART0 pins connected to the internal CP2102/CH340 USB bridge).*

## 2. Setting Native MAVLink Mode

ExpressLRS (v3.5+) supports Native MAVLink routing. 

1. Ensure your TX module and RX module are bound and powered on.
2. Open the **ExpressLRS Lua Script** on your radio (or connect to the TX WebUI).
3. Change the **Link Mode** (or Serial Protocol) from `CRSF` to `MAVLink`.
4. **Auto-Detection:** In ExpressLRS v4, the receiver will automatically detect the TX mode change and instantly switch its own UART protocol to MAVLink. No manual configuration on the RX is necessary.
5. The USB VCP port on your TX module will now stream MAVLink frames at **460800 baud**.

## 2b. CRSF mode instead (the PC as the handset)

The library can also talk CRSF to the TX module, with `controller.setLinkMode(LinkMode::Crsf)`. Then:

1. Use the same pin assignment as above (**TX pin 1, RX pin 3**) so the module's CRSF UART is on the USB port. The two pins must be different pins (a full-duplex UART): a module whose CRSF line is a single shared half-duplex pin cannot be used this way.
2. Leave the TX module in its normal link mode (not MAVLink). The receiver stays in CRSF too.
3. Connect at **400000 baud** (the library's default for CRSF). If your module is configured for another CRSF speed, pass it to `connect()`. The USB-serial bridge must support that speed (the CP210x on ESP32 modules does).
4. The module only answers with telemetry (link statistics, battery, ...) once it has a link to a receiver; the library reports `LinkUp` when the link quality is above 0.

Not yet verified on a real module: the CRSF mode is tested against a loopback transport and against the PentaPod CRSF parser, not over a live radio link.

## 3. Windows Drivers

**Do not use Zadig to replace the Windows USB driver!**

The ExpressLRS ESP32 module relies on the standard Windows USB Serial Device driver (`usbser.sys` / CP210x / CH340). 
If you used Zadig to replace the driver with `WinUSB` or `libusbK` during a previous firmware flash, the COM port will not show up. 

If this happens:
1. Open Windows Device Manager.
2. Find the device (likely under Universal Serial Bus devices).
3. Right-click -> Uninstall Device (check "Attempt to remove the driver for this device").
4. Unplug and re-plug the USB module. Windows will automatically reinstall the correct standard COM port driver.
