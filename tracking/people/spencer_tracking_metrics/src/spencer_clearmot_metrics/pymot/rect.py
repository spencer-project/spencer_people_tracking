import numpy

class Rect(object):
    """Common class for both ground truth objects and hypothesis objects"""

    def __init__(self, entity):
        """Constructor from dict with keys width, height, x, y, dco and id"""
        
        self.x_ = entity["x"]
        self.y_ = entity["y"]

        # Use dco, if found
        self.dco_ = entity.get("dco",False)
        
        # Use id. 
        assert "id" in entity
        self.id_= str(entity["id"]) # cast to string, in case gt is given as int
            
    def asNumpyArray(self):
        return numpy.array([ self.x_, self.y_])
        
    def isDCO(self):
        return self.dco_

    def getID(self):
        return self.id_
