from message import Message
from typing import Any, NoReturn


class Dsm(object):
    __varlist: dict
    __symtab: dict
    __typelist: dict
    __sharelist: dict

    def __init__(self):
        """
        create empty dsm
        """
        self.__symtab = {}
        self.__varlist = {}
        self.__typelist = {}
        self.__sharelist = {}

    @property
    def sharelist(self) -> dict:
        """
        getter method for symbol table
        :return:
        """
        return self.__sharelist

    @sharelist.setter
    def sharelist(self, sharelist: dict) -> NoReturn:
        """
        sharing dictionary setter
        :param sharelist:
        :return:
        """
        self.__sharelist = sharelist

    @property
    def symtab(self) -> dict:
        """
        getter method for symbol table
        :return:
        """
        return self.__symtab

    @symtab.setter
    def symtab(self, symtab: dict) -> NoReturn:
        """
        symbol table setter
        :param symtab:
        :return:
        """
        self.__symtab = symtab

    @property
    def varlist(self) -> dict:
        """
        getter method for variable values
        :return:
        """
        return self.__varlist

    @varlist.setter
    def varlist(self, varlist: dict) -> NoReturn:
        """
        variable value table setter
        :param varlist:
        :return:
        """
        self.__varlist = varlist

    @property
    def typelist(self) -> dict:
        """
        getter method for variable types
        :return:
        """
        return self.__typelist

    @typelist.setter
    def typelist(self, typelist: dict) -> NoReturn:
        """
        variable type table setter
        :param typelist:
        :return:
        """
        self.__typelist = typelist

    def mkawvar(self, dtype: enumerate, varname: str, val: Any = None) -> NoReturn:
        """
        Create allwrite variable
        :param dtype: data type
        :param varname: variable name
        :param val: value possibly assigned during declaration
        :return:
        """
        lastkey = len(list(self.varlist.keys()))
        self.symtab[varname] = lastkey
        self.typelist[lastkey] = dtype
        self.varlist[lastkey] = val
        self.sharelist[lastkey] = 'aw'

    def mkarvar(self, pid: int, numbots: int, dtype: enumerate, varname: str, val: Any = None):
        """
        Create all read variables
        :param pid: declarating robot's pid
        :param numbots : number of robots
        :param dtype: data type
        :param varname: variable name
        :param val: value possibly assigned during declaration
        :return:
        """

        lastkey = len(list(self.varlist.keys()))
        self.symtab[varname] = lastkey
        self.typelist[lastkey] = dtype
        valdict = {}
        for i in range(0, numbots):
            valdict[i] = None
        valdict[pid] = val
        self.varlist[lastkey] = valdict
        self.sharelist[lastkey] = 'ar'

    def update(self, pid: int, varname: str, val: Any) -> NoReturn:
        """
        update shared variable
        :param pid: int pid of updating
        :param varname: variable name
        :param val: value to be updated to
        :return:
        """
        try:
            key = self.symtab[varname]
            if self.sharelist[key] == 'ar':
                self.varlist[key][pid] = val
            else:
                self.varlist[key] = val
        except KeyError:
            print("possible error : variable entry not found")
