------------------------------------------------------------------------------------
BT SDK - CYW43012
------------------------------------------------------------------------------------

Overview
--------
The Cypress CYW43012 is a 28nm ultra-low power dual-mode Bluetooth 5.0 wireless MCU
WiFi + Bluetooth Combo device. It has a stand-alone baseband processor with an
integrated 2.4 GHz transceiver supporting BR, EDR, and BLE.

SDK Software Features
----------------------
- Dual mode Bluetooth stack included in the ROM (BR/EDR/BLE).
- BT stack and profile level APIs for embedded BT application development.
- WICED HCI protocol to simplify host/MCU application development.
- APIs and drivers to access on board peripherals
- Bluetooth protocols include GAP, GATT, SMP, RFCOMM, SDP
- BLE and BR profile APIs, libraries and sample apps
- Documentation for APIs, datasheet, profiles and features.

Kits
----
CYW9M2BASE-43012BT:
    106 ball WLBGA package, Audio Arduino shield with on-board microphones, audio codec
    chip, and headphone output, expansion header (J5) for direct access to BT GPIO,
    i2S, and UART, user switches and LEDs, USB connector for power, programming
    and USB-UART bridge.
    Note: Max UART baud rate is 3M

Software Tools
--------------
The following applications are installed with ModusToolbox on your computer:

BT Spy:
    BTSpy is a trace viewer utility that can be used with WICED BT platforms to
    view protocol and application trace messages from the embedded device. The
    utility is located in the folder below. For more information, see readme.txt
    in the same folder.
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox IDE QuickPanel, or by right-clicking
    a project in the IDE Project Explorer pane and selecting the ModusToolbox
    context menu.
    It is supported on Windows, Linux and macOS.
    <Workspace Dir>\wiced_btsdk\tools\btsdk-utils\BTSpy

BT/BLE Profile Client Control:
    This application emulates host MCU applications for BLE and BR/EDR profiles.
    It demonstrates WICED BT APIs. The application communicates with embedded
    apps over the WICED HCI interface. The application is located in the folder
    below. For more information, see readme.txt in the same folder.
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox IDE QuickPanel, or by right-clicking
    a project in the IDE Project Explorer pane and selecting the ModusToolbox
    context menu.
    It is supported on Windows, Linux and macOS.
    <Workspace Dir>\wiced_btsdk\tools\btsdk-host-apps-bt-ble\client_control

Peer apps:
    Applications that run on Windows, iOS or Android and act as peer BT apps to
    demonstrate specific profiles or features, communicating with embedded apps
    over the air.
    BLE apps location:
    <Workspace Dir>\wiced_btsdk\tools\btsdk-peer-apps-ble

Tracing
-------
To view application traces, the application needs to configure the tracing.
This kit does not support the WICED Peripheral UART, so the only option
is to route traces to the WICED HCI UART.  On this kit, it will appear in
Windows as a COM port named "KitProg3 USB-UART", and will usually be the lower
numbered port.  Open this port on your computer using the Client Control
application mentioned above (usually using 3M baud rate). Then run the BT Spy
utility mentioned above.

Application Settings
--------------------
Application settings can be changed in the application makefile. Options listed
below are available for all applications. Other application specific options may
also be available and are documented in the readme.txt for those applications.

BT_DEVICE_ADDRESS
    Set BT device address for your BT device. The BT address is 6 bytes,
    for example 43012C00FFEE. By default, the SDK will set a BDA for your device
    by combining the 7 hex digit device ID with the last 5 hex digits of the
    host PC MAC address.
UART
    Configure the UART port you want the application to be downloaded. For
    example 'COM6' on Windows or '/dev/ttyWICED_HCI_UART0' on Linux or
    '/dev/tty.usbserial-000154' on macOS.
    By default, the SDK will auto detect the port.
ENABLE_DEBUG
    For HW debugging, see the document WICED-Hardware-Debugging.pdf for more
    information. Configuring this setting with value =1 configures GPIO for SWD.
    CYW9M2BASE-43012BT: SWD hardware debugging is not supported.

Downloading application to kit
------------------------------
This kit does not have SFLASH so image download and write to FLASH is not possible
as with other WICED boards.  Instead, the DIRECT_LOAD feature is the default/only
option for downloading new application FW.  The application 'make build' target
will produce a .hcd image file which can be used to download to RAM on the device
and then instruct the device to reboot directly from RAM instead of the read-only
ROM boot image.

Use the Client Control application mentioned above to Browse to a generated
application image .hcd file, and then click the Download button.

If you have issues downloading to the kit, power cycle the kit to boot from the ROM
image.


------------------------------------------------------------------------------------
