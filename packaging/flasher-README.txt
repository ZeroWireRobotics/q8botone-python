Q8botOne Firmware Flasher
=========================

Puts firmware on a Q8botOne robot or its handheld controller, over USB -
either updating one that already runs firmware, or a brand-new blank board.
Windows only.

What's in this folder
---------------------
  Q8botOne-Firmware-Flasher.exe   the flasher
  firmware\robot.bin              firmware for the robot
  firmware\controller.bin         firmware for the controller
  firmware\bootloader.bin         \
  firmware\partitions.bin          > startup files every board needs - the
  firmware\boot_app0.bin          /  same for robot and controller

Keep the "firmware" folder next to the .exe - that's where the flasher
looks. To share it, send the whole folder (zip it first).

Flashing
--------
1. Plug the robot or the controller into the computer with a USB cable.
   One board at a time. The robot doesn't need to be switched on.
2. Close anything else that might be using the board: the Q8botOne app,
   a serial monitor.
3. Run Q8botOne-Firmware-Flasher.exe.
   Windows may say it doesn't recognize the app: click "More info", then
   "Run anyway".
4. Under Device, pick Robot or Controller. Under Port, pick the board.
   The Firmware line shows the version that will be written.
5. Click Flash. It first asks the board what it is, and refuses to put
   robot firmware on a controller or the other way round. A blank board
   can't say what it is yet, so the flasher asks you to confirm which one
   it is. Writing takes about 10 seconds - don't unplug the board until it
   says it's done.

Newer firmware
--------------
Replace robot.bin and/or controller.bin in the firmware folder with the new
files. Keep the names exactly robot.bin and controller.bin. The flasher reads
the version built into each file and shows it, and it won't accept a file
built for the other board.

Leave the three startup files where they are - they hardly ever change.
Without them the flasher can still update a board that already runs
firmware, but a blank board won't start.

The firmware in this folder as it was put together: {firmware_version},
for both the robot and the controller.

Questions: https://github.com/ZeroWireRobotics/q8botone-firmware/issues
or yufeng.wu0902@gmail.com
