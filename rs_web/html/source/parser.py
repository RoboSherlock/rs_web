# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 11:19:44 2016

@author: ferenc
"""

from pyparsing import *



key = oneOf('location shape color size detection id ts')
separator = Literal(',').suppress()
lpar = Literal('(').suppress()
rpar = Literal(')').suppress()
lbr = Literal('[').suppress()
rbr = Literal(']').suppress()

And = Literal('and').suppress()

kvp = lpar + Group(key + separator + Word(alphanums)) + rpar
interval = lpar + Group(Word(alphanums) + ':' + Word(alphanums))+rpar

objProperty = kvp | interval | Empty()
kvps = Forward()
atom = objProperty | Group(kvps)
kvps << atom + ZeroOrMore(separator + kvps)

objDef = Group('object' +lpar + kvps +rpar)
objs = Forward()
elem = objDef | Group(objs)
objs << elem + ZeroOrMore(And+objs)

qType = oneOf('scenes views')
query1 =  qType + lpar + objs + rpar
query2 =  qType + lpar + kvps + rpar

query = query1 | query2
#this sux but for now it should work
emptyQuery  = qType+lpar+rpar

#def findObjInStoreById(origString,loc,tokens):
#    print 'Searching fro object in ObjStore with ID: '+ tokens[3] 
#    return tokens[3]
#
#obj.setParseAction(findObjInStoreById)

#show(views(object(id,2)))


if __name__ == "__main__":  
 
#    try:   
        s ='object((shape,box),(color, blue))'
        results = objDef.parseString( s )
        print s,'->', results
        
        s ='scenes(object((shape,box))'+ \
                    'and object((id,2))' + \
                  ')'
        results = query.parseString( s )
        print s,'->', results
       
        s ='views(object((shape,box)) and object((id,2)) and object((id,3)))'
        results = query.parseString( s )
        print s,'->', results
                
#        s ='object(())'
#        results = objDef.parseString( s )
#        print s,'->', results

#        s ='views((color,blue))'
#        
#        result= []
#        try:
#            result = emptyQuery.parseString(s)
#        except: 
#            try:
#                result = query.parseString(s)
#            except:
#                print 'Ok this sux'
#        print s,'->', result
        
    
#    except:
#        print('Malformed query. RTFM')
        
