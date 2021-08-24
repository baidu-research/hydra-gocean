appname=`basename $0 | sed s,\.sh$,,`

dirname=`dirname $0`
tmp="${dirname#?}"

if [ "${dirname%$tmp}" != "/"  ]; then
    dirname=$PWD/$dirname
fi
LD_LIBRARY_PATH=$dirname/so
export LD_LIBRARY_PATH
./so/livox_viewer

