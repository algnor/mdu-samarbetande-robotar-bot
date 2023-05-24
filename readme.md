# Hello, this is the readme!

## to controll the robot, send this

```sh
mosquitto_pub -h 10.22.4.26 -m "{'tpid': {'p':10, 'i': 0, 'd':1}, 'dpid': {'p': 20000, 'i': 0, 'd': 0}, 'target':{'x':0, 'y':1}}" -t tankCommandTopic
```
target coordinates are roughly in Meters
