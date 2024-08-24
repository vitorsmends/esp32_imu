# esp32_imu


## Connection using ubuntu

Identify the ESP Address, it must be a code like A0:A3:B3:89:25:A6. And run the command bellow:

```sh
sudo rfcomm bind /dev/rfcomm0 A0:A3:B3:89:25:A6
```

