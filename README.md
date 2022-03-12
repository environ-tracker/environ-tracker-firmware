# environ-tracker

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``environ-tracker`` and all Zephyr modules will be cloned. You can do
that by running:

```shell
# initialize my-workspace for the environ-tracker (main branch)
west init -m https://github.com/Hellymaw/environ-tracker --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

### Build & Run

The application can be built by running:

```shell
west build -s app
```

Note that `environ-tracker` has setup the `BOARD` and `SHIELD` internally via `CMakeLists.txt` directives. The `environ-tracker-v1` board found in this
repository is used.

```shell
west flash
```
