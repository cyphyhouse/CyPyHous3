class Task(object):
    """
    task object, has a location, a task id, whether or not its assigned, and agent pid its assigned to.
    """

    def __init__(self, location, id, assigned, assigned_to):
        """

        :param location:
        :param id:
        :param assigned:
        :param assigned_to:
        """
        self.location = location
        self.id = id
        self.assigned = assigned
        self.assigned_to = assigned_to

    def __repr__(self):
        return str(self.id) + " assigned to " + str(self.assigned_to)


def get_tasks(taskfile):
    """
    TODO: add a repeated get_tasks.
    read a file to obtain tasks
    :param taskfile:
    :param repeat:
    :return:
    """
    import numpy as np
    from src.motion.pos import Pos

    tasks = []
    tasklocs = open(taskfile, "r").readlines()
    for i in range(len(tasklocs)):
        locxyz = tasklocs[i].split(',')
        locnew = Pos(np.array([float(locxyz[0]), float(locxyz[1]), float(locxyz[2])]))
        tasks.append(Task(locnew, i, False, None))
    return tasks
