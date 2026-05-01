build: ./segway_behavior_classifier.py
	echo "(No build required for these scripts)"

test: build
	echo "(No tests exist yet for these scripts)"

install: test ./setup.sh ./install.sh ./crash-detector.service
	./install.sh

package: test ./setup.sh ./install.sh ./crash-detector.service
	tar -czf ../crash-imu.tar.gz \
	--exclude ./crash-imu.tar.gz --exclude docs --exclude .git . && \
	mv ../crash-imu.tar.gz ./crash-imu.tar.gz
