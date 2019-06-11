
class Mutex(object):
    def __init__(self, var_name: str):
        self.__var_name = var_name
        self.__requests = []
        self.__mutex_holder = None
        self.__release = []

    def __repr__(self):
        s = ("var_name: " + self.var_name+ "\n"+"requests: "+ str(self.requests)+"\n"
            +"mutex_holder"+ str(self.mutex_holder)+ "\nrelease"+ str(self.release))
        return s

    @property
    def var_name(self):
        """

        :return:
        """
        return self.__var_name

    @var_name.setter
    def var_name(self, var_name: str):
        """

        :param var_name:
        :return:
        """
        self.__var_name = var_name

    @property
    def requests(self) -> list:
        """

        :return:
        """
        return self.__requests

    @requests.setter
    def requests(self, requests: list) -> None:
        """

        :param requests:
        :return:
        """
        self.__requests = requests

    @property
    def release(self) -> list:
        """

        :return:
        """
        return self.__release

    @release.setter
    def release(self, release: list) -> None:
        """

        :param release:
        :return:
        """
        self.__release = release

    @property
    def mutex_holder(self) -> list:
        """

        :return:
        """
        return self.__mutex_holder

    @mutex_holder.setter
    def mutex_holder(self, mutex_holder: int) -> None:
        """

        :param mutex_holder:
        :return:
        """
        self.__mutex_holder = mutex_holder

    def has_mutex(self, pid):
        """

        :param pid:
        :return:
        """
        if self.mutex_holder == pid:
            return True
        else:
            return False


