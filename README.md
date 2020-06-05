# IMU-F
IMU-F flight controller project

# setup
clone the project

```bash
brew install openocd python
curl -o ~/Downloads/gcc_arm-6-2017-q1.tar.bz2 https://developer.arm.com/-/media/Files/downloads/gnu-rm/6_1-2017q1/gcc-arm-none-eabi-6-2017-q1-update-mac.tar.bz2 
tar -zxvf ~/Downloads/gcc_arm-6-2017-q1.tar.bz2
sudo mkdir /usr/local/gcc_arm-6-2017-q1
sudo mv ~/Downloads/gcc_arm-6-2017-q1 /usr/local/gcc_arm-6-2017-q1
sudo ln -s /usr/local/gcc_arm-6-2017-q1 /usr/local/gcc_arm
echo 'export PATH="$PATH:/usr/local/gcc_arm/bin"' >> ~/.bash_profile 
```
After getting everything that you need to compile, run this command to compile the IMUF
python make.py -T F3


# updated/linux version of "setup"
```bash
## install and setup:
sudo apt -y install openocd git python3   #or python2
git clone https://github.com/emuflight/imu-f.git
cd imuf-f
curl -O https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
tar xvf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2

## build
# cd imu-f  #implied
export PATH="$PATH:$(pwd)/gcc-arm-none-eabi-9-2019-q4-major/bin/"
python make.py -T F3
```
