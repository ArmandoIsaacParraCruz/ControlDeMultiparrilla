Compilar con esta configuración:

[env:esp32doit-devkit-v1]
monitor_speed = 115200
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
lib_deps = 
	br3ttb/PID@^1.2.1
	adafruit/MAX6675 library@^1.1.0
	regenbogencode/ESPNowW@^1.0.2
