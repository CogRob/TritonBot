# Docker Registry Book

* By putting images in subdirectories with image name, we can avoid confiction.
* By putting images in subdirectories with image name and Dockerfile in a known
location, we can automate building process.
* By using private registry (e.g. `FROM tritonbot.github.io/cog_docker`), we can
automate dependency resolution.
