export AMAZEBOT_BASE_DIR=$PWD
cd $AMAZEBOT_BASE_DIR/src/xunfei/samples/iat_online_record_sample
source 64bit_make.sh
cd $AMAZEBOT_BASE_DIR/src/xunfei/samples/tts_online_sample
source 64bit_make.sh
cd $AMAZEBOT_BASE_DIR
source /opt/ros/humble/setup.bash
source install/setup.bash