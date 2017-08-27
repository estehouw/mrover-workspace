from invoke import task
import shutil
import os

import config
import buildlib
import third_party


@task
def dep(ctx):
    """
    "Pins" external pip dependencies and installs them into the product venv.
    """
    buildlib.ensure_product_env()
    third_party.ensure_lcm(ctx)
    # TODO add other third_party deps
    with buildlib.workspace(ctx):
        print("Pinning pip dependencies...")
        ctx.run("pip-compile --output-file external_requirements.txt external_requirements.in")  # noqa
        print("Installing pip dependencies...")
        with buildlib.product_env(ctx):
            ctx.run("pip install -r external_requirements.txt", hide='out')
            ctx.run("pip install -r {}/requirements.txt".format(
                config.JARVIS_ROOT), hide='out')

    print("Done.")


@task
def clean(ctx):
    """
    Wipes away the product venv and hash store,
    enabling a completely new build environment.
    """
    if os.path.isdir(config.BUILD_ROOT):
        shutil.rmtree(config.BUILD_ROOT)


@task(dep)
def rover_msgs(ctx):
    """
    Builds the LCM datatypes in the `rover_msgs` directory.
    """
    if buildlib.files_changed('rover_msgs', ['.lcm']):
        print("rover_msgs unchanged, skipping.")
        return

    # Get all LCM files
    lcm_files = [
            f.path
            for f in os.scandir(config.LCM_TYPES_ROOT)
            if os.path.splitext(f.name)[1] == '.lcm'
    ]

    buildlib.ensure_product_env()
    # Generate the Python stuff
    with buildlib.build_intermediate_dir(ctx, 'rover_msgs_py') as intermediate:
        print("Generating Python package `rover_msgs`...")
        lcm_files_cmdline = ' '.join('"{}"'.format(v) for v in lcm_files)
        ctx.run("lcm-gen --python {} --ppath {}".format(
            lcm_files_cmdline, intermediate))

        # Generate a setup.py file
        with open(os.path.join(intermediate, 'setup.py'), 'w') as setup_py:
            setup_py.write(buildlib.generate_setup_py('rover_msgs', src=False))

        # Install the package
        with buildlib.product_env(ctx):
            buildlib.pyinstall(ctx)

    print("Finished generating Python package.")

    # TODO Generate C++ and install it

    buildlib.save_build_hash('rover_msgs', ['.lcm'])
    print("Done.")


@task(rover_msgs)
def rover_common(ctx):
    """
    Builds the `rover_common` library.
    """
    buildlib.build_python_package(ctx, 'rover_common', executable=False)


@task(rover_common)
def onboard_teleop(ctx):
    """
    Builds the `onboard_teleop` executable.
    """
    buildlib.build_python_package(ctx, 'onboard_teleop')


@task(rover_common)
def base_station_bridge(ctx):
    """
    Builds the `base_station_bridge` executable.
    """
    buildlib.build_python_package(ctx, 'base_station_bridge',
                                  directory=os.path.join(
                                      'base_station', 'bridge'))


@task(base_station_bridge)
def base_station_gui(ctx):
    """
    Builds the `base_station_gui` executable.
    """
    bsgui_root = os.path.join('base_station', 'gui')
    buildlib.ensure_product_env()
    with buildlib.build_intermediate_dir(
            ctx, 'base_station_gui') as intermediate:
        print("Building `base_station_gui` with yarn...")

        srcdir = os.path.join(config.ROOT, bsgui_root)
        shutil.copytree(srcdir,
                        os.path.join(intermediate, 'src_cpy'),
                        ignore=shutil.ignore_patterns('node_modules'))

        with ctx.cd("src_cpy"):
            ctx.run("yarn")
            ctx.run("yarn run build")

        bsgui_dir = os.path.join(config.PRODUCT_ENV, 'share', 'bsgui')
        if os.path.isdir(bsgui_dir):
            shutil.rmtree(bsgui_dir)
        shutil.copytree(os.path.join(intermediate, 'src_cpy', 'dist'),
                        os.path.join(bsgui_dir, 'dist'))
        shutil.copyfile(os.path.join(intermediate, 'src_cpy', 'index.html'),
                        os.path.join(bsgui_dir, 'index.html'))

        # Copy binary
        bsgui_exec_path = os.path.join(config.PRODUCT_ENV,
                                       'bin', 'base_station_gui')
        with open(bsgui_exec_path, 'w') as bsgui_exec:
            bsgui_exec.write(buildlib.template('base_station_gui',
                                               bsgui_dir=bsgui_dir))
        os.chmod(bsgui_exec_path, 0o755)

    print("Done.")
