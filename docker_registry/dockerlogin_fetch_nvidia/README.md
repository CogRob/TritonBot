# Image for `dockerlogin` with Nvidia GPU Support

With these steps, you can run GUI applications (rviz) with Nvidia GPU support.

First, install `nvidia-docker` following instructions
[here](https://github.com/NVIDIA/nvidia-docker). Please use version 2.

Secondly, put the following content to your `~/.dockerloginrc`
```
DOCKER=nvidia-docker
DOCKERLOGIN_IMAGE=tritonbot.github.io/dockerlogin_fetch_nvidia
ENABLE_DOCKERLOGIN_X11=1
```

Last, you probably want to recreate your `dockerlogin` container:
```
touch ~/.DOCKERLOGIN_RECREATE
cogrob_dockerlogin
```

Then you should be able to use GUI applications in docker.
```
use_fetch33
rviz
```
