#bash

for i in encoders decoders filters; do
    echo
    echo $i:; ffmpeg -hide_banner -${i} | egrep -i "npp|cuvid|nvenc|cuda|nvdec|opencl"
done

echo
echo "Please issue:"
echo " ffmpeg -h encoder/decoder/filer=xxx to check the detail info for xxx"
echo

