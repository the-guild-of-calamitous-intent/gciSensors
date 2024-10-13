# Raspberry Pi Pico

![](../docs/rp2040.png)

## `screen`

```
screen tty.usbmoden1234
Ctrl-a-k
```

## Build

Change these as necessary

```cmake
set(PICO_BOARD pico) # pico or pico_w
set(PICO_SDK_PATH "$ENV{HOME}/github/pico-sdk")
// set(ENV{PATH} "$ENV{HOME}/github/gcc-arm/bin:$ENV{PATH}")
```

- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [ARM GNU Tools 10.3-2021.10](https://developer.arm.com/downloads/-/gnu-rm/10-3-2021-10)