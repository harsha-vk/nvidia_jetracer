#!/bin/sh

set -e

password='jetson'

date

DIR="$( cd "$( dirname "$0" )" >/dev/null 2>&1 && pwd )"

# Keep updating the existing sudo time stamp
sudo -v
while true; do sudo -n true; sleep 120; kill -0 "$$" || exit; done 2>/dev/null &

echo "\e[104m Install pip and some python dependencies \e[0m"
sudo apt-get update
sudo apt-get install -y python3-pip python3-pil python3-smbus python3-matplotlib python3-scipy cmake
sudo -H pip3 install --upgrade pip
sudo -H pip3 install flask

echo "\e[104m Install jtop \e[0m"
sudo -H pip install jetson-stats

echo "\e[104m Install the pre-built TensorFlow pip wheel \e[0m"
sudo apt-get update
sudo apt-get install -y libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install -y python3-pip
sudo -H pip3 install -U pip testresources setuptools==49.6.0
sudo -H pip3 install -U numpy==1.16.1 protobuf
sudo -H pip3 install -U future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.1 keras_applications==1.0.8 gast==0.2.2 futures pybind11 h5py==2.10.0
sudo -H pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow==2.3.1+nv20.12

echo "\e[104m Install the pre-built PyTorch pip wheel  \e[0m"
cd
wget https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install -y python3-pip libopenblas-base libopenmpi-dev
sudo -H pip3 install Cython
sudo -H pip3 install torch-1.7.0-cp36-cp36m-linux_aarch64.whl

echo "\e[104m Install TensorFlow models repository \e[0m"
cd
git clone https://github.com/tensorflow/models.git tf_models
cd tf_models/research
git checkout 5b60084
wget -O protobuf.zip https://github.com/protocolbuffers/protobuf/releases/download/v3.14.0/protoc-3.14.0-linux-aarch_64.zip
unzip protobuf.zip
./bin/protoc object_detection/protos/*.proto --python_out=.
sudo -H python3 setup.py install
cd slim
sudo -H python3 setup.py install

echo "\e[104m Install torchvision package \e[0m"
cd
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone https://github.com/pytorch/vision torchvision
cd torchvision
git checkout tags/v0.8.1
sudo -H python3 setup.py install
cd
pip install 'pillow<7'

echo "\e[104m Installing torch2trt \e[0m"
cd
git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
sudo -H python3 setup.py install

echo "\e[104m Installing JetCam \e[0m"
cd
git clone https://github.com/NVIDIA-AI-IOT/jetcam
cd jetcam
sudo -H python3 setup.py install

echo "\e[104m Install traitlets \e[0m"
cd
sudo -H python3 -m pip install git+https://github.com/ipython/traitlets@dead2b8cdde5913572254cf6dc70b5a6065b86f8

echo "\e[104m Install Jupyter Lab \e[0m"
sudo apt-get install -y curl
curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get install -y nodejs libffi-dev libssl1.0-dev
sudo -H pip3 install jupyter jupyterlab
sudo -H jupyter labextension install @jupyter-widgets/jupyterlab-manager

jupyter notebook --generate-config
python3 -c "from notebook.auth.security import set_password; set_password('$password', '$HOME/.jupyter/jupyter_notebook_config.json')"

jupyter server --generate-config
python3 -c "from notebook.auth.security import set_password; set_password('$password', '$HOME/.jupyter/jupyter_server_config.json')"

# fix for Traitlet permission error
sudo chown -R jetson:jetson ~/.local/share/

echo "\e[104m Install jetcard \e[0m"
cd $DIR
pwd
sudo -H python3 setup.py install

echo "\e[104m Install jetcard display service \e[0m"
python3 -m jetcard.create_display_service
sudo mv jetcard_display.service /etc/systemd/system/jetcard_display.service
sudo systemctl enable jetcard_display
sudo systemctl start jetcard_display

echo "\e[104m Install jetcard jupyter service \e[0m"
python3 -m jetcard.create_jupyter_service
sudo mv jetcard_jupyter.service /etc/systemd/system/jetcard_jupyter.service
sudo systemctl enable jetcard_jupyter
sudo systemctl start jetcard_jupyter

echo "\e[104m Make swapfile \e[0m"
cd
if [ ! -f /var/swapfile ]; then
	sudo fallocate -l 4G /var/swapfile
	sudo chmod 600 /var/swapfile
	sudo mkswap /var/swapfile
	sudo swapon /var/swapfile
	sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'
else
	echo "Swapfile already exists"
fi

echo "\e[104m Install jupyter_clickable_image_widget \e[0m"
cd
git clone https://github.com/jaybdub/jupyter_clickable_image_widget
cd jupyter_clickable_image_widget
git checkout no_typescript
sudo -H pip3 install -e .
sudo jupyter labextension install js
sudo jupyter lab build

echo "\e[104m Install remaining dependencies for projects \e[0m"
cd
sudo apt-get install -y python-setuptools

echo "\e[104m Setup Jetson.GPIO \e[0m"
sudo -H pip3 uninstall -y Jetson.GPIO
sudo -H pip3 install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo cp /usr/local/lib/python3.6/dist-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "\e[104m Enable i2c permissions \e[0m"
sudo usermod -aG i2c $USER

echo "\e[104m Setup UART communication \e[0m"
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo udevadm trigger
sudo apt-get install -y python3-serial

echo "\e[104m Set Jetson Nano to 5W mode \e[0m"
sudo nvpmodel -m 1

echo "\e[104m All done! \e[0m"

date