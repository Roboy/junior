#/bin/bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <ip>"
	exit 1
fi

echo "Stream URL: rtsp://$1:8085/stream.sdp"
echo "Starting..."
cvlc alsa://hw:0,0 --sout "#transcode{acodec=mp2,ab=32}:rtp{dst=$1,port=1234,sdp=rtsp://$1:8085/stream.sdp}"

