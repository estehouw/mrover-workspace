import os
import shutil


def check_lcm(ctx):
    """
    Checks for the existence of LCM in the product venv.
    """
    # check for lcm-gen in both venvs
    if (not os.path.exists(ctx.get_product_file('bin', 'lcm-gen')) or
            not os.path.exists(ctx.get_jarvis_file('bin', 'lcm-gen'))):
        return False

    # check for liblcm.so
    libname = 'liblcm.so'

    if not os.path.exists(ctx.get_product_file('lib', libname)):
        return False

    # check that lcm can be imported in Python inside product venv
    with ctx.inside_product_env():
        try:
            ctx.run("python -c 'import lcm'", hide='both')
        except:
            return False

    return True


def ensure_lcm(ctx):
    """
    Installs LCM into the product venv. LCM is expected to be included as a
    git submodule or a subdirectory in the third-party directory.

    Also installs lcm-gen into the Jarvis venv.
    """
    if check_lcm(ctx):
        print("LCM already installed, skipping.")
        return

    lcmdir = os.path.join(ctx.third_party_root, 'lcm')
    with ctx.intermediate('lcm'):
        ctx.run("cp -r {}/* .".format(lcmdir))  # TODO: Use python's own cp
        print("Configuring LCM...")
        ctx.run("./bootstrap.sh", hide='both')
        ctx.run("./configure --prefix={}".format(ctx.product_env), hide='both')
        print("Building LCM...")
        ctx.run("make", hide='both')
        print("Installing LCM...")
        ctx.run("make install", hide='both')
        # Copy the lcm-gen binary into the Jarvis venv so it may be accessible
        # for other parts of the build process.
        shutil.copy("{}/bin/lcm-gen".format(ctx.product_env),
                    "{}/bin/lcm-gen".format(ctx.jarvis_env))

        # Install Python library
        with ctx.inside_product_env():
            with ctx.cd('lcm-python'):
                ctx.run("python setup.py install", hide='both')

    print("Finished installing LCM.")


def check_mbed_cli(ctx):
    """
    Checks for the existence of mbed CLI in the mbed venv.
    """
    if not os.path.exists(ctx.get_mbed_file('bin', 'mbed')):
        return False

    # TODO clean up
    if not os.path.exists(
            os.path.join(ctx.build_intermediate, 'mbed-project')):
        return False

    return True


def ensure_mbed_cli(ctx):
    """
    Installs mbed CLI into a custom Python 2 venv.
    """
    if check_mbed_cli(ctx):
        print("mbed CLI already installed, skipping.")
        return

    ctx.ensure_mbed_env()
    with ctx.inside_mbed_env():
        ctx.run("pip install mbed-cli")

        # Make a temporary mbed project in an intermediate dir
        # TODO clean up
        with ctx.intermediate('mbed-project'):
            ctx.run("mbed new .")
            ctx.run("mbed toolchain GCC_ARM")


def check_openocd(ctx):
    """
    Checks for the existence of openocd in the mbed venv.
    """
    return os.path.exists(ctx.get_mbed_file('bin', 'openocd'))


def ensure_openocd(ctx):
    """
    Installs openocd into the mbed venv.
    """
    if check_openocd(ctx):
        print("OpenOCD already installed, skipping.")
        return

    openocd_dir = os.path.join(ctx.third_party_root, 'openocd')
    ctx.ensure_mbed_env()
    with ctx.intermediate('openocd'):
        ctx.run("cp -r {}/* .".format(openocd_dir))
        print("Configuring OpenOCD...")
        ctx.run('./bootstrap nosubmodule', hide='both')
        ctx.run('./configure --prefix={}'.format(ctx.mbed_env), hide='both')
        print("Building OpenOCD...")
        ctx.run("make", hide='both')
        print("Installing OpenOCD...")
        ctx.run("make install", hide='both')

    print("Finished installing OpenOCD.")


def check_rapidjson(ctx):
    """
    Checks for the existence of RapidJson in the product venv.
    """
    return os.path.exists(ctx.get_product_file('include', 'rapidjson'))

def ensure_rapidjson(ctx):
    """
    Installs RapidJson into the product venv.
    """
    if check_rapidjson(ctx):
        print("RapidJson already installed, skipping.")
        return

    rapidjson_dir = os.path.join(ctx.third_party_root, 'rapidjson')
    ctx.ensure_product_env()
    with ctx.intermediate('rapidjson'):
        ctx.run("cp -r {}/* .".format(rapidjson_dir))
        print("Configuring rapidjson...")
        #ctx.run("mkdir -p build")
        #with ctx.cd("build"):
        ctx.run("cmake -DCMAKE_INSTALL_PREFIX={} .".format(
            ctx.product_env))
        print("Building rapidjson...")
        ctx.run("cmake --build .")
        print("Installing rapidjson...")
        ctx.run("cmake --build . --target install")
        print("Done")
