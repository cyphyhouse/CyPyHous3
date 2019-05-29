# from message import Message
from typing import Any, NoReturn


class Dsm(object):
    __var_list: dict
    __sym_tab: dict
    __type_list: dict
    __share_list: dict

    def __init__(self):
        """
        create empty dsm
        """
        self.__sym_tab = {}
        self.__var_list = {}
        self.__type_list = {}
        self.__share_list = {}

    @property
    def share_list(self) -> dict:
        """
        getter method for symbol table
        :return:
        """
        return self.__share_list

    @share_list.setter
    def share_list(self, share_list: dict) -> NoReturn:
        """
        sharing dictionary setter
        :param share_list:
        :return:
        """
        self.__share_list = share_list

    @property
    def sym_tab(self) -> dict:
        """
        getter method for symbol table
        :return:
        """
        return self.__sym_tab

    @sym_tab.setter
    def sym_tab(self, sym_tab: dict) -> NoReturn:
        """
        symbol table setter
        :param sym_tab:
        :return:
        """
        self.__sym_tab = sym_tab

    @property
    def var_list(self) -> dict:
        """
        getter method for variable values
        :return:
        """
        return self.__var_list

    @var_list.setter
    def var_list(self, var_list: dict) -> NoReturn:
        """
        variable value table setter
        :param var_list:
        :return:
        """
        self.__var_list = var_list

    @property
    def type_list(self) -> dict:
        """
        getter method for variable types
        :return:
        """
        return self.__type_list

    @type_list.setter
    def type_list(self, type_list: dict) -> NoReturn:
        """
        variable type table setter
        :param type_list:
        :return:
        """
        self.__type_list = type_list

    def mk_aw_var(self, d_type: enumerate, var_name: str, val: Any = None) -> NoReturn:
        """
        Create allwrite variable
        :param d_type: data type
        :param var_name: variable name
        :param val: value possibly assigned during declaration
        :return:
        """
        last_key = len(list(self.var_list.keys()))
        self.sym_tab[var_name] = last_key
        self.type_list[last_key] = d_type
        self.var_list[last_key] = val
        self.share_list[last_key] = 'aw'

    def mk_ar_var(self, pid: int, numbots: int, d_type: enumerate, var_name: str, val: Any = None):
        """
        Create all read variables
        :param pid: declarating robot's pid
        :param numbots : number of robots
        :param d_type: data type
        :param var_name: variable name
        :param val: value possibly assigned during declaration
        :return:
        """

        last_key = len(list(self.var_list.keys()))
        self.sym_tab[var_name] = last_key
        self.type_list[last_key] = d_type
        val_dict = {}
        for i in range(0, numbots):
            val_dict[i] = None
        val_dict[pid] = val
        self.var_list[last_key] = val_dict
        self.share_list[last_key] = 'ar'

    def update(self, pid: int, var_name: str, val: Any) -> NoReturn:
        """
        update shared variable
        :param pid: int pid of updating
        :param var_name: variable name
        :param val: value to be updated to
        :return:
        """
        try:
            key = self.sym_tab[var_name]
            if self.share_list[key] == 'ar':
                self.var_list[key][pid] = val
            else:
                self.var_list[key] = val
        except KeyError:
            print("possible error : variable entry not found")
