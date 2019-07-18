class Task(object):

    def __init__(self, location, id, assigned, assigned_to):
        self.location = location
        self.id = id
        self.assigned = assigned
        self.assigned_to = assigned_to

    def __repr__(self):
        return str(self.id) + " assigned to " + str(self.assigned_to)
