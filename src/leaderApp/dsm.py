class dsmvar():
    def __init__(self,varname,owner,value,type,ts):
        self.varname = varname
        self.owner = owner
        self.value = value
        self.type = type
        self.ts = ts


class dsm():
    def __init__(self):
        self.varmap = {}

    def add(self,varname):
        if varname not in list(self.varmap.keys()):
            self.varmap[varname.varname] = varname


    def put(self,varname,value,ts):
        for var in list(self.varmap.keys()):
            if var == varname and ts >= self.varmap[var].ts:
                #print("timestamp of update is ", ts, " and timestamp of old value is ", self.varmap[var].ts)
                self.varmap[varname] = dsmvar(varname,self.varmap[varname].owner,value,self.varmap[varname].type,ts)

    def get(self,varname):
        #print(list(self.varmap.keys()))
        if varname in list(self.varmap.keys()):
            return self.varmap[varname].value
        else:
            return None
