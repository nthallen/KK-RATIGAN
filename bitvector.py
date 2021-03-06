# Miles E. Allen, 12 October 2017

import operator

# A class to store a bit and a vector.
class BitVector():
    vector=None
    boolean=False
    
    def __init__(self,points):
        self.vector=points
        self.boolean=False

class BitSequenceHorizontal():
    n_divs=None
    cur_div=0
    
    def evaluate(self,az,el):
        # Keep azimuth at zero, raise elevation
        loc=[az*self.cur_div/self.n_divs,0]
        if (self.cur_div >= self.n_divs):
            self.cur_div = -self.n_divs
        else:
            self.cur_div = self.cur_div+1
        return loc

    def __init__(self,n_divs):
        self.n_divs = n_divs

class BitSequenceVertical():
    n_divs=None
    cur_div=0
    
    def evaluate(self,az,el):
        # Keep azimuth at zero, raise elevation
        loc=[0,el*self.cur_div/self.n_divs]
        if (self.cur_div >= self.n_divs):
            self.cur_div = -self.n_divs
        else:
            self.cur_div = self.cur_div+1
        return loc

    def __init__(self,n_divs):
        self.n_divs = n_divs

# A class to store a sequence of bit-vectors.
class BitSequence():
    sequence=None
    normalization_vector=None
    angle_vector=None
    
    def evaluate(self,az,el):
        self.angle_vector=[az,el]
        loc=[0,0]
        for item in self.sequence:
            if (item.boolean):
                loc=tuple(map(operator.add,loc,item.vector))
        loc=tuple(map(operator.sub,loc,self.normalization_vector))
        loc=tuple(map(operator.truediv,loc,self.normalization_vector))
        loc=tuple(map(operator.mul,loc,self.angle_vector))
        increment(self.sequence)
        
        # Subtract normalization vector from one pass results
        # Divide by normalization vector
        # Multiply by angle vector [az,el]
        
        return loc
    
    def __init__(self,m,n):
        self.sequence=[]
        self.normalization_vector=[((2**m-1)/2),((2**n-1)/2)]
        for i in range(max(m,n)):
            power=2**i
            coord=[power,0]
            alt_coord=[0,power]
            if (m>n):
                self.sequence.append(BitVector(coord))
                if (i<n):
                    self.sequence.append(BitVector(alt_coord))
            else:
                self.sequence.append(BitVector(alt_coord))
                if (i<m):
                    self.sequence.append(BitVector(coord))
        if (len(self.sequence)!=(m+n)):
            print("nope")
            exit(-1)

def print_bit_vectors(sequence):
    for item in sequence:
        print("[",item.vector,item.boolean,"]",end='')
    print()

def increment(sequence):
    last=len(sequence)-1
    
    while (last>=0 and sequence[last].boolean):
        sequence[last].boolean=False
        last-=1
    if (last>=0 and not sequence[last].boolean):
        sequence[last].boolean=True