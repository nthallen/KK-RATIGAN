# -*- coding: utf-8 -*-
"""
Created on Wed Oct 11 13:18:47 2017

@author: Miles Allen
"""

import operator



class BitVector():
    vector=None
    boolean=None
    
    def __init__(self,points,value):
        self.vector=points
        self.boolean=value
    
def print_bit_vectors(sequence):
    for item in sequence:
        print("[",item.boolean,"]",end='')
    print()
    
def increment(sequence):
    last=len(sequence)-1
    
    while (last>=0 and sequence[last].boolean):
        sequence[last].boolean=False
        last-=1
    if (last>=0 and not sequence[last].boolean):
        sequence[last].boolean=True
    
def main():
    bv_1=BitVector([1,0],False)
    bv_2=BitVector([0,1],False)
    bv_3=BitVector([2,0],False)
    bv_4=BitVector([0,2],False)

    sequence=(bv_1,bv_2,bv_3,bv_4)

    i=0
    while (i<16):
        print_bit_vectors(sequence)
        loc=[0,0]
        for item in sequence:
            if (item.boolean):
                loc=tuple(map(operator.add,loc,item.vector))
        print(loc)
        increment(sequence)
        i+=1
main()