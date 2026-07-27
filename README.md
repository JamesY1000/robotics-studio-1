# robotics-studio-1
Repository for the UTS subject Robotics Studio 1


To use the devcontainer setup, first complete these steps:

Install docker:

*If your computer has nvidia gpus?
Install Nvidia runtime for docker


Create a folder ~/git


Create gitconfig or make sure you have it already?


Create ccache or make sure you have it already?

*If your computer does not have nvidia gpus and step x. Install nvidia runtime for docker failed, go to the .env file and change the container_runtime variable from nvidia to runc


Building the image:
- cd to .devcontainer folder
- First build the base image using: docker compose --profile devcontainer-build build rs1-devcontainer-base

- Then build the user image: docker compose build rs1-devcontainer-user

Then open vscode, click 'reopen in container'
Everytime in the future, you just need to open the repository to the homepage (RS1), and you click reopen in container.

Maybe in future we have a registry that we push to?