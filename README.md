# How  to use?
This is a simple tool for recording rosbag with realsense, event camera, imu and vicon.

***IF YOU FOUND ANY ISSUE PLEASE CONTACT ME***

1. Environment clone

`git clone https://github.com/NAIL-HNU/record_utils --recursive`

2. Environment build
- If you can download dv-processing 1.7.9 from [official link](https://dv-processing.inivation.com/master/installation.html), then skip to Sec. Compile drivers, else continue.

- Run `bash env.sh` install all dependencies.

3. Compile

- Run `bash compile.sh`.

4. Check sensors setup

- Checkout scripts/sensors_start.sh. Make sure needed sensors are listed.

5. Sensor sensors start

- Run `bash scripts/sensors_start.sh`.
- Checkout ros topics.

6. Bag record
- Modify `scripts/record.sh` to fit your topics.
- Run `bash scripts/record.sh <bag_name>` to record a bag name after <bag_name>.
- Checkout bag info output

Enjoy.

