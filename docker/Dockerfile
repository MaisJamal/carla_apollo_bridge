FROM auroai/cyber:v5.0.0
# cyber / bridge requirements
RUN apt update && \
    apt-get install -y gedit && \
    apt install -y python-pip && \
    pip install --upgrade pip && \
    #python -m pip install --upgrade --force-reinstall pip && \
    #pip install transforms3d && \
    pip install \
            numpy \
            opencv-python==4.2.0.32 \
            protobuf \
            pygame \
            pyproj \
            pyyaml \
            scipy

RUN echo "source /apollo/cyber/setup.bash" >> ~/.bashrc && \
    echo "CARLA_PYTHON_ROOT=~/carla_apollo_bridge_13/carla-python-0.9.13" >> ~/.bashrc && \
    echo "PYTHONPATH=/apollo/py_proto:\$PYTHONPATH" >> ~/.bashrc && \
    echo "PYTHONPATH=\$CARLA_PYTHON_ROOT/carla:\$CARLA_PYTHON_ROOT/carla/dist/carla-0.9.13-py2.7-linux-x86_64.egg:\$PYTHONPATH" >> ~/.bashrc


ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV SDL_VIDEO_GL_DRIVER = libGL.so.1.7.0  #libGL.so.1
ENV DISPLAY=$DISPLAY
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/nvidia"
ENV  __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia



# add user and his password
#ARG USER=docker_carla
#ARG UID=1021
#ARG GID=1021
# default password
#ARG PW=user
#RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd && adduser ${USER} sudo
#WORKDIR /home/${USER}
#USER ${UID}:${GID}