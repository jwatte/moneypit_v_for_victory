
all:	training.csv

training.csv:	mktraining.sh ../camcam/mkdetect
	./mktraining.sh

../camcam/mkdetect:
	make -C ../camcam mkdetect

clean:
	rm -f training.csv training-old.csv
