build:
	mkdir -p /var/tmp/arduino-build/sketch
	cp -p *.cpp *.h /var/tmp/arduino-build/sketch/
	mkdir -p /var/tmp/arduino-cache
	$(HOME)/arduino-1.8.3/arduino-builder -compile -logger=machine -hardware $(HOME)/arduino-1.8.3/hardware -tools $(HOME)/arduino-1.8.3/tools-builder -tools $(HOME)/arduino-1.8.3/hardware/tools/avr -built-in-libraries $(HOME)/arduino-1.8.3/libraries -libraries $(HOME)/Arduino/libraries -fqbn=teensy:avr:teensy31:usb=serialmidiaudio,speed=96,opt=o2std,keys=en-us -ide-version=10803 -build-path /var/tmp/arduino-build -warnings=more -build-cache /var/tmp/arduino-cache -verbose /usr/local/src/moneypit_v/mpv_teensy/mpv_teensy.ino 2>&1 | sed -e 's@/var/tmp/arduino-build/sketch/@@'
	$(HOME)/arduino-1.8.3/hardware/tools/teensy_post_compile -file=mpv_teensy.ino -path=/var/tmp/arduino-build -tools=$(HOME)/arduino-1.8.3/hardware/tools
	$(HOME)/arduino-1.8.3/hardware/tools/teensy_reboot

clean:
	rm -rf /var/tmp/arduino-build /var/tmp/arduino-cache
