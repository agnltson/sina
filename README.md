# Spacial Indoor Navigation Anchor
This is the guide to build the system.

## How to build the project
The first step is to download the Apriltag library used as a git submodule:
To do that run:
```bash
git submodule update --init --recursive
```

Then you can build the whole project:
```bash
cargo build
```
This can take a long time.

## How to run the project
First you need to set yourself in you python virtual environment with the aria sdk installed.
We recommand to follow the [Aria Gen 1 Docs](https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/setup)
but to use anaconda for the env.

With anaconda:
```bash
conda activate aria_sdk_env
```

To run the project:
```bash
cargo run -- --config path/to/config/file.toml --nav path/to/navigation/folder
```

For clearer informations run:
```bash
cargo run -- --help
```

## How to use the preprocessing tools
Record a room with the glasses.
The inside [Aria studio](https://facebookresearch.github.io/projectaria_tools/docs/ARK/aria_studio) you can request SLAM on
your recording.

### SceneScript
The next step is to use [SceneScript](https://github.com/facebookresearch/scenescript) to create the ```semantic.txt``` file.
SceneScript requires an NVIDIA GPU and can be a real pain to install.
Inside SceneScript virtual environment you can use ```python/preprocessing/semantic_extract.py```.

### Apriltag position extraction
This tools needs the projectaria tools installed.

```bash
cd $HOME
git clone https://github.com/facebookresearch/projectaria_tools
cd projectaria_tools
mkdir build
cd build
cmake ..
make
```
Then to compile the extraction tool:
```bash
cd tools/preprocessing
mkdir build
cd build
cmake .. -DARIA_ROOT=$HOME/projectaria_tools
make
```
This can take a long time.
