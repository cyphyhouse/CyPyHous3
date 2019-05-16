#comms and commhandler

class dsmvar():
    def __init__(self,varname,owner,value,type,ts):
        self.varname = varname
        self.owner = owner
        self.value = value
        self.type = type
        self.ts = ts


class dsm():
    def __init__(self,n):
        self.varmap = {}
        self.numbots = n

    def add(self,varname,owner = -1):
        l = list(self.varmap.keys())
        v = varname.varname
        t = varname.type[1]

        if v not in l and varname.type[1] is 'aw':
            self.varmap[v] = varname
        if v not in l and varname.type[1] is 'ar':
            l1 = []
            for i in range(self.numbots):
                if varname.owner == i:
                    l1.append(varname)
                else:
                    l1.append(None)
            self.varmap[v] = l1

        if v in l and t is 'ar':
            #print ("owner",varname.owner)
            self.varmap[v][varname.owner] = varname




    def put(self,varname,value,ts,owner = -1):
        for var in list(self.varmap.keys()):
            if owner == -1:
                if var == varname and ts >= self.varmap[var].ts:
                    if self.varmap[varname].type[1] is 'aw':
                        self.varmap[varname] = dsmvar(varname,self.varmap[varname].owner,value,self.varmap[varname].type,ts)
            else:
                if var == varname and self.varmap[var][owner].ts <= ts:
                        self.varmap[varname][owner] = dsmvar(varname,owner,value,self.varmap[varname][owner].type,ts)


    def get(self,varname,owner = -1):
        if owner == -1:
            if varname in list(self.varmap.keys()):
                return self.varmap[varname].value
            else:
                return None
        else:
            for var in list(self.varmap.keys()):
                if var == varname:
                    if self.varmap[varname][owner] is not None:
                        return self.varmap[varname][owner].value

            return None


    def getallar(self,varname,numBots):
        values = []
        for var in list(self.varmap.keys()):
            if var == varname:
                for bot in range(numBots):
                    values.append(self.varmap[varname][bot].value)
        return values
