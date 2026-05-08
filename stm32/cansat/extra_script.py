Import("env")

# Генерирует firmware.bin из firmware.elf после сборки (нужно для dfu-util)
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "binary",
        "$BUILD_DIR/${PROGNAME}.elf",
        "$BUILD_DIR/${PROGNAME}.bin"
    ]), "Generating BIN")
)
