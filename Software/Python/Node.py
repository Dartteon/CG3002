class NODE:
    '''Used instead of map data for ease of use'''
    def __init__(self, nodeId, x, y, nodeName, linkTo):
        '''Initialise node'''
        self.nodeId = int(nodeId)
        self.x = int(x)
        self.y = int(y)
        self.nodeName = nodeName
        self.linkTo = linkTo
        self._f = 0
        self._g = 0
        self._h = 0
        self.parent = None
        
    def __lt__(self, other):
        '''Return true if self node has smaller f than other node'''
        return self.f < other.f

    def __eq__(self,other):
        '''Return true if both nodes have the same node ID'''
        return self.nodeId == other.nodeId
    
    @property
    def f(self):
        '''Returns private f value'''
        return self._f
    
    @property
    def g(self):
        '''Returns private g value'''
        return self._g
    
    @g.setter
    def g(self, cost):
        '''Sets private g value and update private f value'''
        self._g = cost
        self._f = cost + self.h
        
    @property
    def h(self):
        '''Returns private h value'''
        return self._h
    
    @h.setter
    def h(self, heuristic):
        '''Sets private h value and update private f value'''
        self._h = heuristic
        self._f = self.g + heuristic