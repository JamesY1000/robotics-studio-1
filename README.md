# Robotics Studio 1

Repository for the UTS subject Robotics Studio 1 group 2.

Members are:
- James Yuan
- Timothy Ollerton
- Nikhil Kumar
- Noah Nourdine
- Thejas Madathil

This repository is a common place to store and maintain our code, documentation, and setup instructions for the project throughout the semester.

## Project Workflow (Quick Start)

1. Pull the latest changes from main.
2. Create a branch for your task.
3. Develop and test locally in the devcontainer.
4. Open a pull request with a clear description.
5. Get at least one teammate review.
6. Merge only when checks pass.

## Development Environment

I've already set up a docker devcontainer that will be able to handle everything this subject throws at us plus whatever we want to run/setup, but to be honest you can continue using whatever setup you've currently got if that's easier/more comfortable for you.

### Docker setup (if you want to use the devcontainer)

- Docker installed and running.
- VS Code with Dev Containers extension.
- A local workspace directory at ~/git.
- A valid git config on your machine.
- ccache installed/configured (recommended).

If your computer has an NVIDIA GPU:
- Install NVIDIA Container Toolkit (Docker NVIDIA runtime).

If your computer does not have an NVIDIA GPU, or NVIDIA runtime setup fails:
- Open the .env file.
- Change container_runtime from nvidia to runc.

To allow X11 forwarding to the container which you'll need for gui applications like rviz, run the command before rebuilding the devcontainer (in VSCode). This is a temporary solution till I figure out how to allow forwarding more cleanly:
'''bash
xhost +SI:localuser:$(whoami)
'''

### Build the Devcontainer Images

From the .devcontainer directory:

1. Build the base image:

```bash
docker compose --profile devcontainer-build build rs1-devcontainer-base
```

2. Build the user image:

```bash
docker compose build rs1-devcontainer-user
```

3. Open the repo in VS Code and select Reopen in Container.

After initial setup, you usually only need to open the repository in VS Code and reopen in container. If you prefer to run multiple terminal windows, you can use lazydocker and enter the container this way.

## Branching and Commit Conventions

Use one branch per issue or feature.

Suggested branch prefixes:
- feature/<short-description>
- fix/<short-description>
- test/<short-description>
- docs/<short-description>

Commit message style:
- Keep messages concise and descriptive.
- Start with an action verb.
- Mention subsystem when relevant.

Example:
- Add retry logic for UWB packet timeout

## Pull Request Guidelines

Before opening a PR:
- Rebase or merge latest main into your branch.
- Confirm your branch builds successfully.
- Update docs if behavior/setup changed.

PR expectations:
- Clearly defined and focused scope.
- Clear summary of what changed and why.
- At least one teammate review before merge.

## Team Rules

1. Never push directly to main.
2. Pull the latest main before starting work.
3. Use one branch per issue or feature.
4. Use descriptive branch and commit names.
5. Keep pull requests focused and reasonably small.
6. At least one teammate reviews each pull request.
7. Do not merge code that does not build.
8. Merge frequently instead of integrating everything near the deadline.
9. Document any dependency assumptions and setup procedures.
10. Never commit secrets, generated build folders, or large datasets.

## Definition of Done

A task is done when:
- Code is implemented (working) and reviewed.
- Code does not break any other part of overall system.
- Relevant docs are updated.