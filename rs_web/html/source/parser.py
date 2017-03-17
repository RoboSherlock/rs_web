# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

import sys
from pyparsing import *

#####new parsing: more designator like
class QueryHandler:
    
    def __init__(self):
        
        print "Init Query Handler"

    def kvp_cb_(self, toks):
        print "Token: %s." % toks
        

    def res_specif_cb_(self,toks):
        print " : Result Specifier : %s." % (toks)

    def operator_cb_(self,toks):
        print ":Operator call-back: %s" % (toks)


class RSQueryGrammar:
        
    def __init__(self):
        self.qh = QueryHandler()
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
        key = oneOf('location shape color size detection id ts type value ratio confidence') ^ resultSpecifier
        
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

        value = Word(alphanums) ^ description
        
        #e.g. simple symbolic assignment        
        simpleKvp = Group(Group(key+delimiter+value))        
        
        #numerical comparisson confidence > 0.0
        valueKvp = Group(key+operator+(floatnumber|value))
        nestedKvP = Group(key+delimiter+resultDescription)
        
#         kvp =  simpleKvp | nestedKvP | valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvp =  simpleKvp ^ nestedKvP ^ valueKvp #matchfirst sux...use or instead..which is end...FFS
        kvps << (OneOrMore(kvp+Optional(separator)) |kvps)

        query = bq+resultSpecifier+delimiter+resultDescription+eq
        
        resultSpecifier.setParseAction(self.qh.res_specif_cb_)
        simpleKvp.addParseAction(self.qh.kvp_cb_)
        valueKvp.addParseAction(self.qh.operator_cb_)
        
        self.query = query
        
    def parseQuery(self,q):
        print 'Parsing ',q
        res = self.query.parseString(q)
        return res
        
if __name__ == "__main__":  
    gr = RSQueryGrammar()
    try:    
        s ='{object:[shape:blue, size:small]}'
        results = gr.parseQuery(s)
        print s,'->', results
#         
#        s = '{scene:[object:[id:2],object:[id:4]]}'
#        results = gr.parseQuery(s)
#        print s,'->', results    
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        s = '{scene:[hypotheses:[type:Plate,shape:(value:blue, confidence=0.5)],hypotheses:[color:blue, shape:box, type:crap]]}'
        results = gr.parseQuery(s)
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
    except Exception:
        print 'Something went horrably wrong!'
 
        
