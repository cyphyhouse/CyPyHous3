import time

from src.harness.agentThread import AgentThread
from src.objects.base_mutex import BaseMutex
from src.harness.comm_handler import CommHandler, CommTimeoutError
from src.harness.gvh import Gvh
from src.functionality.mutex_handler import BaseMutexHandler


class AgentCreation(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, pid, participants, receiver_ip, r_port):
        """
        parameters to instantiate the gvh and communication handler.
        :param pid:
        :param participants:
        :param r_port:
        """
        #config object.
        agent_gvh = Gvh(pid, participants)
        agent_gvh.port_list = [2000,2001,2002,2005,2004]
        if pid == 0:
            agent_gvh.is_leader = True
        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port)

        super(AgentCreation, self).__init__(agent_gvh, agent_comm_handler, mutex_handler)
        self.agent_comm_handler.agent_gvh = self.agent_gvh
        self.requestedlock = False
        self.req_num = 0

        self.start()

    def run(self):
        #initialize lock
        a = BaseMutex(1, [2000,2001,2002,2003,2004])
        self.agent_gvh.mutex_handler.add_mutex(a)
        a.agent_comm_handler = self.agent_comm_handler

        #initialize vars
        self.locals = {}
        self.create_aw_var('sum', int, 0)
        self.create_aw_var('numadded', int, 0)
        self.locals['added'] = False
        self.locals['finalsum'] = 0

        while not self.stopped():

            time.sleep(0.4)
            self.agent_gvh.flush_msgs()
            self.agent_comm_handler.handle_msgs()
            time.sleep(0.4)

            try:
                #loop body
                if not self.locals['added']:
                    #lock()
                    if not self.requestedlock:
                        a.request_mutex(self.req_num)
                        self.requestedlock = True
                        self.req_num += 1
                        continue
                    else:
                        if not self.agent_gvh.mutex_handler.has_mutex(a.mutex_id):
                            continue

                    self.agent_gvh.put('sum', self.agent_gvh.get('sum') + self.pid()*2)
                    self.agent_gvh.put('numadded', self.agent_gvh.get('numadded')+1)
                    self.locals['added'] = True


                    #unlock()
                    time.sleep(0.4)
                    a.release_mutex()
                    self.requestedlock = False

                if self.agent_gvh.get('numadded') >= self.agent_gvh.participants:
                    self.locals['finalsum'] = self.agent_gvh.get('sum')
                    print('final sum for',self.pid(),'is', self.locals['finalsum'])
                    self.stop()

            except CommTimeoutError:
                print("timed out on communication")
                self.stop()


b,c,d = AgentCreation(2, 3, "", 2001),AgentCreation(4, 3, "", 2002),AgentCreation(0, 3, "", 2000)


