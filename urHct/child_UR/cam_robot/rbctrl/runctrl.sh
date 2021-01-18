if ps ax | grep -v grep | grep "rbcmgsvr.py" > /dev/null
then
  echo "rbcmgsvr service is run!!"
else
  nohup python -u rbcmgsvr.py </dev/null &>/dev/null &
fi
