all:
	make -C showip
	make -C camcam
	make -C mpv_teensy
	make -C training_data

clean:
	make -C showip clean
	make -C camcam clean
	make -C mpv_teensy clean
	make -C training_data clean
