Import("env")

def after_build(source, target, env):
    print("Building firmware.bin")
    elf_path = target[0].get_path()
    bin_path = elf_path.replace(".elf", ".bin")
    env.Execute("$OBJCOPY -O binary %s %s" % (elf_path, bin_path))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)