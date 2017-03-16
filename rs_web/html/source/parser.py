# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *

#####new parsing: more designator like
class QueryHandler:
    def __init__(self):
        print "Init Query Handler"
#param is token found in string
    def kvp_cb_(self, toks):
        print "Token: ",toks
        #as long as we us add Parse action we don't need to return anything
        return toks    
    
    def res_specif_cp_(self,toks):
        #now search DB based on what is queried.
        print " : Result Specifier: ",toks
        return toks

    def descript_cb_(self,toks):
        print "description of a key: ",toks
        
qh = QueryHandler()

bq = Literal('{').suppress()
eq = Literal('}').suppress()
bl = Literal('[').suppress()
el = Literal(']').suppress()
bp = Literal('(').suppress()
ep = Literal(')').suppress()

delimiter = oneOf(':').suppress()
operator = oneOf('= < >')
separator = oneOf(', ;').suppress()

resultSpecifier = oneOf('scene hypotheses object')
key = oneOf('location shape color size detection id ts type value ratio confidence') | resultSpecifier

point = Literal('.')
number = Word(nums) 
plusorminus = Literal('+') | Literal('-')
integer = Combine( Optional(plusorminus) + number )
floatnumber = Combine( integer +
                       Optional( point + Optional(number) )
                     )

kvps = Forward() 
resultDescription = bl+ kvps +el
description = bp+kvps+ep

value = Word(alphanums) | floatnumber | description.addParseAction(qh.descript_cb_)
 
kvp = Group(key+delimiter+value).addParseAction(qh.kvp_cb_) | Group(key+delimiter+resultDescription)| Group(key+operator+value)
kvps << (OneOrMore(kvp+Optional(separator)) |kvps)

query = bq+resultSpecifier.addParseAction(qh.res_specif_cp_)+delimiter+resultDescription+eq

if __name__ == "__main__":  

    try:    
#         s ='{object:[shape:blue, size:small]}'
#         results = query.parseString(s)
#         print s,'->', results
#         
        s = '{scene:[object:[id:2],object:[id:4]]}'
        results = query.parseString(s)
        print s,'->', results    
        
        s = '{scene:[hypotheses:[type:Plate,shape:(value:blue, confidence=0)],hypotheses:[color:blue, shape:box, type:crap]]}'
        results = query.parseString(s)
        print s,'->', results 
        
#         {scene:[hypotheses:
#                     [type:Plate],
#                 hypotheses:
#                     [color:(value:blue,ratio=0.6) 
#                      shape:(value:box,confidence>0.5) 
#                      type:crap,
#                      classification:[confidence>0.6],
#                        detection:(source:X,)   
#                      ]]}

    except (ParseException):
        print('Malformed query. RTFM')
    except:
        print('Something went horrably wrong!')
        
