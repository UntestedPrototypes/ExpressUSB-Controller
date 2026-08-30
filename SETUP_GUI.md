# ExpressLRS Transmitter Hardware Setup

Welcome to the ExpressLRS Hardware Configuration guide! To allow the **ExpressUSB-Controller** software to communicate with your TX module over a USB COM port, the internal hardware UART pins (GPIO 1 and 3) must be correctly mapped to the ESP32 in the ExpressLRS Web UI.

Follow the interactive setup wizard below to configure your module.

````carousel
## Step 1: Enter Wi-Fi Mode

To change the internal hardware pinout, we need to access the module's Web Interface.

1. Power on your ExpressLRS Transmitter module.
2. Wait ~60 seconds without a radio connection until the module's LED begins to blink rapidly. This indicates it has entered Wi-Fi mode.
3. On your computer or phone, connect to the **ExpressLRS TX** Wi-Fi network.

> [!TIP]
> The default Wi-Fi password is `expresslrs`.

<!-- slide -->
## Step 2: Open the Web Interface

Once connected to the module's Wi-Fi network, we can access the configuration portal.

1. Open your web browser.
2. Navigate to **`http://10.0.0.1`** to open the ExpressLRS Web UI.
3. Click on the **Hardware** tab located at the top of the page.

> [!NOTE]
> If the page does not load, ensure your computer hasn't automatically switched back to your home Wi-Fi network.

<!-- slide -->
## Step 3: Configure UART Pins

Scroll down to the pin mapping section. You must assign the hardware pins correctly so the internal USB-to-Serial chip can talk directly to the ESP32 processor.

Locate and configure the pins **exactly** as follows:
- **Serial RX Pin:** `3`
- **Serial TX Pin:** `1`

> [!IMPORTANT]
> GPIO 3 and GPIO 1 are the physical UART0 pins on the ESP32. If these are incorrectly assigned, the Windows COM port will open successfully, but the CRSF data will disappear into the void, causing the software to freeze or fail to connect.

<!-- slide -->
## Step 4: Save and Reboot

You're almost done! 

1. Scroll to the bottom of the Hardware page.
2. Click the **Save and Reboot** button.
3. Wait for the module's LED to turn off or reset, then disconnect from the Wi-Fi.
4. Unplug the TX module and plug it back into your PC via USB to establish a fresh connection.

**Success!** You are now ready to run the `expressusb_example.exe` software.
````
