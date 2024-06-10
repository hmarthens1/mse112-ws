After OS setup and installation, run the command below first:
```
sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED
```

git clone under the /home/pi directory the repo:

```
git clone git@github.com:hmarthens1/mse112-ws.git

```

go into the directory

```
cd ~/mse112-ws

```

And then follow the package installation procedures below 

# MasterPi Packages
install these packages

1. yaml handle
```
sudo pip install pyyaml
```

2.rpi_ws281x
```
sudo pip install rpi_ws281x
```

3. RPi.GPio
```
sudo pip install RPi.GPIO
```
4. smbus2
```
sudo pip install smbus2
```

Now that all the required packages are installed, test an example:

go into the directory

```
cd ~/mse112-ws/MasterPi/HiwonderSDK

```

# example for MasterPi
run the example for RGB light


```
sudo python RGBControlDemo.py
```

# yolov8 packages

```
sudo pip install opencv-python
```

```
sudo pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2
````

```
sudo pip install ultralytics
```

```
sudo pip install cvzone
```




