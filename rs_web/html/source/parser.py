# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *

#####new parsing: more designator like

bq = Literal('{').suppress()
eq = Literal('}').suppress()
bl = Literal('[').suppress()
el = Literal(']').suppress()
delimiter = Literal(':').suppress()
separator = oneOf(', ;').suppress()

resultSpecifier = oneOf('scene hypotheses object')
key = oneOf('location shape color size detection id ts type value confidence') | resultSpecifier
value = Word(alphanums)

 
kvps = Forward() 
description = bl+ kvps +el 
kvp = Group(key+delimiter+value) | Group(key+delimiter+description)
kvps << OneOrMore(kvp+Optional(separator)) |kvps

query = bq+resultSpecifier+delimiter+description+eq

if __name__ == "__main__":  

    try:    
        s ='{object:[shape:blue, size:small]}'
        results = query.parseString(s)
        print s,'->', results
        
        s = '{scene:[object:[id:2],object:[id:4]]}'
        results = query.parseString(s)
        print s,'->', results    
        
        s = '{scene:[hypotheses:[id:2],hypotheses:[color:blue, shape:box, type:crap]]}'
        results = query.parseString(s)
        print s,'->', results 
        
        
    except (ParseException):
        print('Malformed query. RTFM')
    except:
        print('Something went horrably wrong!')
        
