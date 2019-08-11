#!/usr/bin/env python3
import argparse

from src.config.config_funcs import get_configs
from src.harness.agentThread import AgentThread


def get_opt_args():
    """
    pass a config file and an appname as command line arguments.
    :return:
    """
    parser = argparse.ArgumentParser()
    group = parser.add_argument_group()
    group.add_argument('-a', '--app', action='store')
    group.add_argument('-c', '--config', action='store')
    args = parser.parse_args()
    appfile = args.app
    configfile = args.config
    return (appfile, configfile)


def get_app_name(appfile):
    """
    get the appname from the app file.
    :param appfile:
    :return:
    """
    appcode = open(appfile).read()
    end = appcode.find("(AgentThread)")
    begin = appcode[:end].rfind("class ") + 6
    appname = appcode[begin:end].strip()
    return appname


def import_app(appfile, appname):
    """
    dynamic import of the appname from the appfile.
    :param appfile:
    :param appname:
    :return:
    """
    from importlib import machinery
    loader = machinery.SourceFileLoader(appname, appfile)
    mod = loader.load_module()
    app = getattr(mod, appname)
    return app


def run_app(appfile, configfile) -> AgentThread:
    """
    run the app specified in the appfile , with configuration from the configfile.
    :param appfile:
    :param configfile:
    :return: instantiated AgentThread
    """
    appname = get_app_name(appfile)
    app = import_app(appfile, appname)
    ac, mc = get_configs(configfile)
    return app(ac, mc)


if __name__ == '__main__':
    """
    script to call the run_app functions
    """
    appfile = None
    configfile = None
    try:
        appfile = get_opt_args()[0]
        configfile = get_opt_args()[1]
        run_app(appfile, configfile)
    except IndexError:
        print("invalid app and config input. Use -h for help")
    if appfile is None or configfile is None:
        print("invalid app and config input. Use -h for help")
