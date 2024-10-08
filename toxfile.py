import tox.plugin


@tox.plugin.impl
def tox_on_install(tox_env, arguments, section, of_type):
    # Add extra installation stage to install Scenic *before* its dependencies,
    # so that if we install VerifAI we won't pull in a different version of
    # Scenic that might have different dependencies. (This is currently
    # necessary since Tox installs the dependencies and Scenic in 2 different
    # invocations of pip. See https://github.com/tox-dev/tox/discussions/3273
    # for a discussion.)
    if section == "RunToxEnv" and of_type == "package":
        assert len(arguments) == 1
        package = arguments[0]
        path = package.path
        install_args = ["--force-reinstall", "--no-deps", str(path)]
        tox_env.installer._execute_installer(install_args, "package_early")
