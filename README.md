# Midterm Project

ENPM808X - Mudit Singal, Abhishekh Reddy and Abhimanyu Saxena

### Project Status

![CICD Workflow status](https://github.com/armgits/808x-midterm/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![codecov](https://codecov.io/gh/armgits/808x-midterm/branch/development/graph/badge.svg)](https://codecov.io/gh/armgits/808x-midterm)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview

Inverse Kinematics solver software package for a 6 degrees of freedom articulated arm. See [Project Wiki](https://github.com/armgits/808x-midterm/wiki) to learn more.

## Setting up this project...

### Get started

Clone this repository to your favorite directory

```bash
git clone https://github.com/armgits/808x-midterm.git Midterm
```

Switch to development branch for the latest yet unstable version if you wish to...

```bash
git checkout development
```

### Create the build system

```bash
cmake -S ./ -B build/
```

### Build the project

Build in either way:

Normal build

```bash
cmake --build build/
```

Clean build

```bash
cmake --build build/ --clean-first
```

Verbose build

```bash
cmake --build build/ --verbose
```

To use `bear` just prepend `bear -- ` to either of the above two commands.

For example

```bash
bear -- cmake --build build/
```

### Run the program

TODO

### Test the program

```bash
ctest --test-dir build/
```

### Generate the documentation

Doxygen documentation is generated to the `docs/` directory. Open `index.html` file in that folder to browse through the project.

```bash
cmake --build build/ --target docs
```

### Clean up the project

Just clean the build artifacts while retaining the build system

```bash
cmake --build build/ --target clean
```

Remove the build system including the artifacts

```bash
rm -rf build/
```

Clean-up any other residual files and folders

```bash
rm -r .cache/ compile_commands.json
```

## Contributing

Never directly modify the contents of the main branch. Always create a development branch in which you can contribute your changes.

```bash
git checkout -b <your-development-branch>
```
>**Note:** Replace `<your-development-branch>` with a name you desire.

Commit messages should roughly be a sentence-two long and here's a template to follow:

```
Backlog <N.n.n> - <Your concise commit message>
```

Before pushing the changes, update your branch with the latest changes from the main branch. You can do this in two steps:

Switch to main branch and fetch the latest changes.

```bash
git checkout main && git fetch
```

Switch back to your development branch and merge the latest main branch. This does not delete your changes while updating your branch. Merge conflicts may occur sometimes and use the diff to resolve conflicts.

```bash
git checkout <your-development-branch> && git merge main
```

Now you can push your changes to your branch and create a pull request to merge your branch with the main branch.

<p align="center"><img src="https://media.tenor.com/-O_9bNdwqngAAAAC/all-is-well-all-izz-well.gif" height="250"></p>
