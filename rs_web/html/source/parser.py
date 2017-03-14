# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *

#key = oneOf('location shape color size detection id ts')
#separator = Literal(',').suppress()
#lpar = Literal('(').suppress()
#rpar = Literal(')').suppress()
#lbr = Literal('[').suppress()
#rbr = Literal(']').suppress()
#
#And = Literal('and').suppress()
#
#kvp = lpar + Group(key + separator + Word(alphanums)) + rpar
#interval = lpar + Group(Word(alphanums) + ':' + Word(alphanums))+rpar
#
#objProperty = kvp | interval | Empty()
#kvps = Forward()
#atom = objProperty | Group(kvps)
#kvps << atom + ZeroOrMore(separator + kvps)
#
#objDef = Group('object' +lpar + kvps +rpar)
#objs = Forward()
#elem = objDef | Group(objs)
#objs << elem + ZeroOrMore(And+objs)
#
#qType = oneOf('scenes views')
#query1 =  qType + lpar + objs + rpar
#query2 =  qType + lpar + kvps + rpar
#
#query = query1 | query2
##this sux but for now it should work
#emptyQuery  = qType+lpar+rpar

#####new parsing: more designator like

bq = Literal('{').suppress()
eq = Literal('}').suppress()
bl = Literal('[').suppress()
el = Literal(']').suppress()
delimiter = Literal(':').suppress()
separator = oneOf(', ;')

resultSpecifier = oneOf('scene hypotheses object')
key = oneOf('location shape color size detection id ts type value confidence') | resultSpecifier
value = Word(alphanums)

kvp=key+delimiter+value | key+delimiter + description
 
kvps = Forward() 
description = bl+ kvps +el
kvp = Group(key+delimiter+value) | Group(key+delimiter+description)
kvps<<kvp+separator|kvp

if __name__ == "__main__":  

    s ='object:[shape,box]'
    results = kvp.parseString(s)
    print s,'->', results
        
#        s ='scenes(object((shape,box))'+ \
#                    'and object((id,2))' + \
#                  ')'
#        results = query.parseString( s )
#        print s,'->', results
       
#        s ='views(object((shape,box)) and object((id,2)) and object((id,3)))'
#        results = query.parseString( s )
#        print s,'->', results
    
#    except:
#        print('Malformed query. RTFM')
        
